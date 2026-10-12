"""SFTP download with resume, bounded throughput and registered session roots."""
import hashlib
import json
import pathlib
import re
import stat
import time
import threading
import shutil
from scene_registry import SceneRegistry

class SessionManager:
    def __init__(self,board,root,registry=None):
        self.board=board;self.root=pathlib.Path(root).resolve();self.lock=threading.Lock()
        self.registry=registry or SceneRegistry.shared(self.root)
    @staticmethod
    def select_files(files,kind):
        if kind=='all':return files
        if kind not in ('rgb','depth','stereo','images'):raise ValueError('未知下载类型')
        def selected(name):
            p=pathlib.PurePosixPath(name)
            if kind=='stereo':return p.parts[0]=='segments' and p.suffix in ('.svo','.svo2')
            if p.parts[0]!='exports':return False
            return (p.name==kind+'_video.mp4') if kind in ('rgb','depth') else p.suffix=='.png'
        return [f for f in files if selected(f['name'])]
    @staticmethod
    def transfer_limit(state):
        return 0 if state.get('camera_state')=='stopped' and not state.get('pid') else 256*1024
    def fetch_dataset_index(self,result):
        self.root.mkdir(parents=True,exist_ok=True)
        target=self.root/'dataset_index.json';part=target.with_suffix('.json.part')
        with self.lock,self.board.connection() as client:
            sftp=client.open_sftp();sftp.get('/home/uavlab/Jetson_ROS2_V0/perception_records/dataset_index.json',str(part))
            index=json.loads(part.read_text(encoding='utf-8'))
            if index.get('split_unit')!='session':raise ValueError('数据集分组清单无效')
            part.replace(target)
        return dict(result,local_index=str(target))
    def download(self,session_id,progress=lambda x:None,cancelled=lambda:False,kind='all'):
        with self.registry.operation(session_id):return self._download(session_id,progress,cancelled,kind)
    def _download(self,session_id,progress,cancelled,kind):
        if not re.fullmatch('[a-zA-Z0-9_-]{1,80}',session_id):raise ValueError('采集场次 ID 无效')
        sessions=self.board.call('list_sessions');session=next((s for s in sessions if s['session_id']==session_id),None)
        if session is None:raise ValueError('采集场次未登记')
        if session['summary'].get('state') in ('recording','finalizing') or session['summary'].get('tail_metadata_may_arrive') is not False:
            raise ValueError('场次元数据尚未可靠收尾；先结束并校验场次，再下载')
        files=self.select_files(session['files'],kind)
        if not files:raise ValueError('该场次还没有所选文件，请先停止相机后生成')
        self.registry.update([dict(session,display_name=session.get('display_name',session_id))])
        with self.lock,self.board.connection() as client:
            sftp=client.open_sftp();base='/home/uavlab/Jetson_ROS2_V0/perception_records/'+session_id
            dest=self.registry.folder(session_id,create=True);done=0;total=sum(f['bytes'] for f in files);checks=[]
            started=time.monotonic();last_poll=0;limit=256*1024;transferred=0
            for info in files:
                rel=pathlib.PurePosixPath(info['name'])
                if rel.is_absolute() or '..' in rel.parts:raise ValueError('非法文件名')
                remote=base+'/'+str(rel)
                for i in range(1,len(rel.parts)+1):
                    attr=sftp.lstat(base+'/'+str(pathlib.PurePosixPath(*rel.parts[:i])))
                    if stat.S_ISLNK(attr.st_mode):raise ValueError('不下载符号链接')
                target=dest/pathlib.Path(*rel.parts);target.parent.mkdir(parents=True,exist_ok=True)
                if not target.resolve().is_relative_to(dest.resolve()):raise ValueError('本机文件路径越界')
                source=self.board.call('session_checksum',{'session_id':session_id,'file':str(rel)})
                if target.is_file() and target.stat().st_size==info['bytes']:
                    existing_sha=hashlib.sha256()
                    with target.open('rb') as existing:
                        while chunk:=existing.read(1024*1024):existing_sha.update(chunk)
                    if existing_sha.hexdigest()==source['sha256']:
                        done+=info['bytes'];checks.append({'file':str(rel),'sha256':source['sha256'],'reused':True});continue
                part=target.with_name(target.name+'.part');offset=part.stat().st_size if part.exists() else 0
                if offset>info['bytes']:raise ValueError('续传文件大于源文件')
                if shutil.disk_usage(dest).free<info['bytes']-offset+1024**3:raise ValueError('电脑空间不足，需保留 1 GiB')
                sha=hashlib.sha256()
                with sftp.open(remote,'rb') as src:
                    if offset:
                        with part.open('rb') as existing:
                            while chunk:=existing.read(65536):sha.update(chunk)
                        src.seek(offset)
                    done+=offset
                    with part.open('ab') as out:
                        position=offset
                        while position<info['bytes']:
                            if time.monotonic()-last_poll>=1:
                                try:limit=self.transfer_limit(self.board.call('status'))
                                except Exception:limit=256*1024
                                last_poll=time.monotonic()
                            # Bounded 512 KiB readv batch, no whole-file prefetch cache.
                            reads=[(n,min(32768,info['bytes']-n)) for n in range(position,min(position+524288,info['bytes']),32768)]
                            for chunk in src.readv(reads):
                                if cancelled():raise ValueError('下载已取消，保留 .part 以便续传')
                                if not chunk:raise ValueError('源文件提前结束')
                                tick=time.monotonic();out.write(chunk);sha.update(chunk);done+=len(chunk);position+=len(chunk);transferred+=len(chunk)
                                if shutil.disk_usage(dest).free<1024**3:raise ValueError('电脑磁盘余量不足，保留续传文件')
                                if limit:time.sleep(max(0,len(chunk)/limit-(time.monotonic()-tick)))
                                rate=transferred/max(.001,time.monotonic()-started)
                                progress({'downloaded_bytes':done,'total_bytes':total,'file':str(rel),'bytes_s':rate,
                                          'eta_s':(total-done)/max(1,rate),'speed_mode':'采集中限速' if limit else '停止后高速'})
                if part.stat().st_size!=info['bytes']:raise ValueError('下载大小不匹配')
                # Source hash computed remotely through a fixed registered-path RPC.
                source=self.board.call('session_checksum',{'session_id':session_id,'file':str(rel)})
                if source['sha256']!=sha.hexdigest():raise ValueError('下载校验失败；保留临时文件')
                part.replace(target);checks.append({'file':str(rel),'sha256':sha.hexdigest()})
            (dest/('download_integrity_'+kind+'.json')).write_text(json.dumps(checks),encoding='utf-8')
            return {'folder':str(dest),'session_id':session_id,'files':checks,'kind':kind,'bytes':total,'elapsed_s':time.monotonic()-started}
