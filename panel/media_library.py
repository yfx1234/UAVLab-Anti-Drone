"""PC playback derivatives and local image extraction; board originals stay intact."""
import pathlib,json,subprocess,time,shutil,urllib.parse,uuid,os,re,contextlib
from preview_recorder import PreviewRecorder
from scene_registry import SceneRegistry

class MediaLibrary:
    def __init__(self,root,preview_root=None,registry=None):
        self.root=pathlib.Path(root).resolve();self.preview_root=pathlib.Path(preview_root).resolve() if preview_root else None
        self.registry=registry or SceneRegistry.shared(self.root,self.preview_root)
    def path(self,relative):
        with self.registry.lock:
            self.registry.discover()
            if not isinstance(relative,str) or '\\' in relative:raise ValueError('本机媒体路径无效')
            rel=pathlib.PurePosixPath(relative);root=self.root;entries=self.registry.scenes
            if rel.is_absolute() or '..' in rel.parts:raise ValueError('本机媒体路径越界')
            if self.preview_root and rel.parts and rel.parts[0]=='preview_videos':
                root=self.preview_root;entries=self.registry.previews;rel=pathlib.PurePosixPath(*rel.parts[1:])
            if len(rel.parts)<2:raise ValueError('本机媒体文件未登记')
            entry=entries.get(rel.parts[0]) or next((r for r in entries.values() if r['folder']==rel.parts[0]),None)
            if entry is None:raise ValueError('本机媒体场次未登记')
            base=self.registry.checked(root,entry['folder']);p=base.joinpath(*rel.parts[1:]).resolve()
            if not p.is_relative_to(base.resolve()) or not p.is_file():raise ValueError('本机媒体文件不存在或路径越界')
            if p.suffix!='.mp4':raise ValueError('仅允许播放本机 MP4')
            return p
    @contextlib.contextmanager
    def open_file(self,relative):
        # Resolve and acquire the handle atomically against folder moves.
        with self.registry.lock:
            source=self.path(relative);stream=source.open('rb')
        try:yield stream
        finally:stream.close()
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
        sid=result.get('session_id') or pathlib.Path(result['folder']).name
        with self.registry.operation(sid):return self._prepare(result,cancelled)
    def _prepare(self,result,cancelled):
        sid=result.get('session_id') or pathlib.Path(result['folder']).name
        folder=self.registry.folder(sid)
        media=[]
        for f in result['files']:
            source=folder/f['file']
            if source.suffix!='.mp4':continue
            source=self.path(source.resolve().relative_to(self.root).as_posix())
            output=source.with_name(source.stem+'.view.mp4');tmp=source.with_name(source.stem+'.view.tmp.mp4')
            sidecar=output.with_suffix('.json');binding={'source_sha256':f.get('sha256'),'source_bytes':source.stat().st_size,'source_mtime_ns':source.stat().st_mtime_ns}
            if not output.is_file() or not sidecar.is_file() or json.loads(sidecar.read_text())!=binding:
                if shutil.disk_usage(source.parent).free<max(1024**3,source.stat().st_size*2):raise ValueError('本机转换空间不足，下载原文件已保留')
                self._run(['-y','-i',str(source),'-an','-c:v','libx264','-crf','16','-preset','fast','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp)],source.parent,cancelled)
                tmp.replace(output);sidecar.write_text(json.dumps(binding),encoding='utf-8')
            relative=output.relative_to(self.root).as_posix()
            media.append(dict(self._row(self.registry.scenes[sid],sid,result['kind'],source.parent.name,relative,source.name),path=str(output)))
        return dict(result,folder=str(folder),session_id=sid,media=media)
    def list(self):
        with self.registry.lock:
            self.registry.discover();rows=[]
            for sid,entry in self.registry.scenes.items():
                folder=self.registry.checked(self.root,entry['folder'])
                for p in folder.glob('exports/*/*.view.mp4'):
                    if p.is_symlink():continue
                    relative=p.relative_to(self.root).as_posix();kind='depth' if p.name.startswith('depth_') else 'rgb'
                    rows.append(self._row(entry,sid,kind,p.parent.name,relative,p.name))
            if self.preview_root:
                for pid,entry in self.registry.previews.items():
                    if pid in self.registry.active_previews:continue
                    folder=self.registry.checked(self.preview_root,entry['folder'])
                    for p in folder.glob('*.mp4'):
                        if p.is_symlink():continue
                        relative='preview_videos/'+p.relative_to(self.preview_root).as_posix()
                        rows.append(self._row(entry,pid,'preview_'+entry.get('view_mode','rgb'),p.stem,relative,p.name))
            def order(row):
                number=re.search(r'(\d+)$',row['segment'])
                return (-row['created_at_unix_s'],row.get('source_session_id') or row['session_id'],int(number[1]) if number else 0,row['kind'],row['relative'])
            return sorted(rows,key=order)
    @staticmethod
    def _row(entry,sid,kind,segment,relative,filename):
        return {'session_id':sid,'source_session_id':entry.get('source_session_id'),'display_name':entry['display_name'],
            'created_at_unix_s':entry['created_at_unix_s'],'created_time_uncertain':entry.get('created_time_uncertain',True),
            'kind':kind,'segment':segment,'relative':relative,'url':'/api/media/'+urllib.parse.quote(relative),
            'name':entry['display_name']+' / '+('电脑预览 / ' if kind.startswith('preview_') else '')+segment+' / '+filename,
            'local_sync_error':entry.get('local_sync_error')}
    def scene_names(self,rows):
        self.registry.update(rows)
    def rename_scene(self,sid,name):return self.registry.rename(sid,name)
    def open_folder(self,relative):
        with self.registry.lock:
            source=self.path(relative)
            if os.name!='nt':raise ValueError('打开文件位置只用于Windows本机')
            subprocess.Popen(['explorer.exe',f'/select,{source}'],creationflags=0x08000000)
            return {'folder':str(source.parent),'file':str(source)}
    def extract(self,session_id,fps=2):
        if not re.fullmatch('[a-zA-Z0-9_-]{1,80}',session_id) or not .2<=fps<=15:raise ValueError('场次或抽帧参数无效')
        with self.registry.operation(session_id):return self._extract(session_id,fps)
    def _extract(self,session_id,fps):
        folder=self.registry.folder(session_id);videos=sorted(folder.glob('exports/*/rgb_video.mp4'),key=lambda p:int(re.search(r'(\d+)$',p.parent.name)[1]))
        if not videos:raise ValueError('先下载该场次 RGB 视频，再在电脑抽帧')
        output=folder/('training_images_'+uuid.uuid4().hex[:8]);output.mkdir()
        for i,source in enumerate(videos,1):
            self.path(source.relative_to(self.root).as_posix())
            self._run(['-i',str(source),'-vf',f'fps={fps}','-vsync','0',str(output/f'segment_{i}_%06d.png')],output)
        return {'folder':str(output),'images':len(list(output.glob('*.png'))),'annotation_status':'unlabeled'}
