"""PC playback derivatives and local image extraction; board originals stay intact."""
import pathlib,json,subprocess,time,shutil,urllib.parse,uuid,os
from preview_recorder import PreviewRecorder

class MediaLibrary:
    def __init__(self,root,preview_root=None):self.root=pathlib.Path(root).resolve();self.preview_root=pathlib.Path(preview_root).resolve() if preview_root else None
    def path(self,relative):
        rel=pathlib.PurePosixPath(relative);root=self.root
        if self.preview_root and rel.parts and rel.parts[0]=='preview_videos':root=self.preview_root;rel=pathlib.PurePosixPath(*rel.parts[1:])
        p=(root/str(rel)).resolve()
        if not p.is_relative_to(root) or not p.is_file():raise ValueError('本机媒体文件不存在或路径越界')
        if p.suffix!='.mp4':raise ValueError('仅允许播放本机 MP4')
        return p
    def _run(self,args,folder,cancelled=lambda:False):
        if shutil.disk_usage(folder).free<1024**3:raise ValueError('电脑磁盘需保留 1 GiB')
        log=folder/'conversion.log'
        with log.open('ab') as error:
            proc=subprocess.Popen([PreviewRecorder.ffmpeg(),'-v','error','-nostdin',*args],stdout=subprocess.DEVNULL,
                stderr=error,creationflags=0x08000000 if os.name=='nt' else 0)
            try:
                deadline=time.monotonic()+3600
                while proc.poll() is None:
                    if cancelled():raise ValueError('已取消本机转换；下载原文件已保留')
                    if time.monotonic()>deadline or shutil.disk_usage(folder).free<1024**3:raise ValueError('本机转换超时或磁盘余量不足')
                    time.sleep(.2)
                if proc.returncode:raise ValueError('本机视频转换失败，原文件已保留')
            finally:
                if proc.poll() is None:proc.terminate();proc.wait(timeout=10)
    def prepare(self,result,cancelled=lambda:False):
        media=[]
        for f in result['files']:
            source=pathlib.Path(result['folder'])/f['file']
            if source.suffix!='.mp4':continue
            source=self.path(str(source.resolve().relative_to(self.root)))
            output=source.with_name(source.stem+'.view.mp4');tmp=source.with_name(source.stem+'.view.tmp.mp4')
            sidecar=output.with_suffix('.json');binding={'source_sha256':f.get('sha256'),'source_bytes':source.stat().st_size,'source_mtime_ns':source.stat().st_mtime_ns}
            if not output.is_file() or not sidecar.is_file() or json.loads(sidecar.read_text())!=binding:
                if shutil.disk_usage(source.parent).free<max(1024**3,source.stat().st_size*2):raise ValueError('本机转换空间不足，下载原文件已保留')
                self._run(['-y','-i',str(source),'-an','-c:v','libx264','-crf','16','-preset','fast','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp)],source.parent,cancelled)
                tmp.replace(output);sidecar.write_text(json.dumps(binding),encoding='utf-8')
            relative=output.relative_to(self.root).as_posix()
            media.append({'path':str(output),'relative':relative,'url':'/api/media/'+urllib.parse.quote(relative),'name':source.name,'kind':result['kind']})
        return dict(result,media=media)
    def list(self):
        names={}
        try:names=json.loads((self.root/'scene_names.json').read_text(encoding='utf-8'))
        except (ValueError,OSError):pass
        rows=[{'relative':p.relative_to(self.root).as_posix(),'url':'/api/media/'+urllib.parse.quote(p.relative_to(self.root).as_posix()),
               'name':names.get(p.parent.parent.parent.name,p.parent.parent.parent.name)+' / '+p.name} for p in sorted(self.root.glob('*/exports/*/*.view.mp4'))]
        if self.preview_root:
            for p in sorted(self.preview_root.glob('preview_*/*.mp4')):
                relative='preview_videos/'+p.relative_to(self.preview_root).as_posix()
                rows.append({'relative':relative,'url':'/api/media/'+urllib.parse.quote(relative),'name':'电脑预览录像 / '+p.parent.name+' / '+p.name})
        return rows
    def scene_names(self,rows):
        self.root.mkdir(parents=True,exist_ok=True);path=self.root/'scene_names.json';temp=path.with_name('scene_names.'+uuid.uuid4().hex+'.tmp');names={}
        try:names=json.loads(path.read_text(encoding='utf-8'))
        except (ValueError,OSError):pass
        names.update({r['session_id']:r['display_name'] for r in rows})
        temp.write_text(json.dumps(names,ensure_ascii=False),encoding='utf-8');temp.replace(path)
    def open_folder(self,relative):
        source=self.path(relative)
        if os.name!='nt':raise ValueError('打开文件位置只用于Windows本机')
        subprocess.Popen(['explorer.exe',f'/select,{source}'],creationflags=0x08000000)
        return {'folder':str(source.parent),'file':str(source)}
    def extract(self,session_id,fps=2):
        import re
        if not re.fullmatch('[a-zA-Z0-9_-]{1,80}',session_id) or not .2<=fps<=15:raise ValueError('场次或抽帧参数无效')
        folder=self.root/session_id;videos=sorted(folder.glob('exports/*/rgb_video.mp4'))
        if not videos:raise ValueError('先下载该场次 RGB 视频，再在电脑抽帧')
        output=folder/('training_images_'+uuid.uuid4().hex[:8]);output.mkdir()
        for i,source in enumerate(videos,1):
            self.path(str(source.relative_to(self.root)))
            self._run(['-i',str(source),'-vf',f'fps={fps}','-vsync','0',str(output/f'segment_{i}_%06d.png')],output)
        return {'folder':str(output),'images':len(list(output.glob('*.png'))),'annotation_status':'unlabeled'}
