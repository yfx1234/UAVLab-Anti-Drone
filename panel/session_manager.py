"""SFTP download with resume, bounded throughput and registered session roots."""
import hashlib
import json
import pathlib
import re
import stat
import time
import threading

class SessionManager:
    def __init__(self,board,root):self.board=board;self.root=pathlib.Path(root);self.lock=threading.Lock()
    def fetch_dataset_index(self,result):
        self.root.mkdir(parents=True,exist_ok=True)
        target=self.root/'dataset_index.json';part=target.with_suffix('.json.part')
        with self.lock,self.board.connection() as client:
            sftp=client.open_sftp();sftp.get('/home/uavlab/Jetson_ROS2_V0/perception_records/dataset_index.json',str(part))
            index=json.loads(part.read_text(encoding='utf-8'))
            if index.get('split_unit')!='session':raise ValueError('数据集分组清单无效')
            part.replace(target)
        return dict(result,local_index=str(target))
    def download(self,session_id,progress=lambda x:None,cancelled=lambda:False):
        if not re.fullmatch('[a-zA-Z0-9_-]{1,80}',session_id):raise ValueError('采集场次 ID 无效')
        sessions=self.board.call('list_sessions');session=next((s for s in sessions if s['session_id']==session_id),None)
        if session is None:raise ValueError('采集场次未登记')
        if session['summary'].get('state') in ('recording','finalizing') or session['summary'].get('tail_metadata_may_arrive') is not False:
            raise ValueError('场次元数据尚未可靠收尾；先结束并校验场次，再下载')
        with self.lock,self.board.connection() as client:
            sftp=client.open_sftp();base='/home/uavlab/Jetson_ROS2_V0/perception_records/'+session_id
            dest=self.root/session_id;dest.mkdir(parents=True,exist_ok=True);done=0;total=session['bytes'];checks=[]
            for info in session['files']:
                rel=pathlib.PurePosixPath(info['name'])
                if rel.is_absolute() or '..' in rel.parts:raise ValueError('非法文件名')
                remote=base+'/'+str(rel)
                for i in range(1,len(rel.parts)+1):
                    attr=sftp.lstat(base+'/'+str(pathlib.PurePosixPath(*rel.parts[:i])))
                    if stat.S_ISLNK(attr.st_mode):raise ValueError('不下载符号链接')
                target=dest/pathlib.Path(*rel.parts);target.parent.mkdir(parents=True,exist_ok=True)
                part=target.with_name(target.name+'.part');offset=part.stat().st_size if part.exists() else 0
                if offset>info['bytes']:raise ValueError('续传文件大于源文件')
                sha=hashlib.sha256()
                with sftp.open(remote,'rb') as src:
                    if offset:
                        with part.open('rb') as existing:
                            while chunk:=existing.read(65536):sha.update(chunk)
                        src.seek(offset)
                    with part.open('ab') as out:
                        while chunk:=src.read(65536):
                            if cancelled():raise ValueError('下载已取消，保留 .part 以便续传')
                            tick=time.monotonic();out.write(chunk);sha.update(chunk);done+=len(chunk)
                            progress({'downloaded_bytes':done,'total_bytes':total,'file':str(rel)})
                            time.sleep(max(0,len(chunk)/(256*1024)-(time.monotonic()-tick)))
                if part.stat().st_size!=info['bytes']:raise ValueError('下载大小不匹配')
                # Source hash computed remotely through a fixed registered-path RPC.
                source=self.board.call('session_checksum',{'session_id':session_id,'file':str(rel)})
                if source['sha256']!=sha.hexdigest():raise ValueError('下载校验失败；保留临时文件')
                part.replace(target);checks.append({'file':str(rel),'sha256':sha.hexdigest()})
            (dest/'download_integrity.json').write_text(json.dumps(checks),encoding='utf-8')
            return {'folder':str(dest),'files':checks}
