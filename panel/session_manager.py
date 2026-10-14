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
    def __init__(self,board,root,registry=None,status_provider=None):
        self.board=board;self.root=pathlib.Path(root).resolve();self.lock=threading.Lock()
        self.registry=registry or SceneRegistry.shared(self.root)
        self.status_provider=status_provider
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
        if state.get('camera_state')=='stopped' and not state.get('pid'):return 0
        p=state.get('pipeline',{})
        def number(key):
            value=p.get(key)
            return value if isinstance(value,(int,float)) and not isinstance(value,bool) else float('nan')
        node_age=state.get('last_status_age_s')
        fresh_node_status=(isinstance(node_age,(int,float)) and not isinstance(node_age,bool) and 0<=node_age<=1.5)
        healthy_rgb_preview=(state.get('camera_state')=='running' and bool(state.get('pid')) and
            fresh_node_status and
            state.get('profile')=='rgb' and p.get('profile')=='rgb' and
            p.get('depth_enabled') is False and p.get('detector_enabled') is False and p.get('record_state')=='idle' and
            p.get('generic_detector_enabled',False) is False and
            number('capture_hz')>=13 and number('mem_available_kib')>=2*1024*1024 and
            0<=number('max_temperature_c')<65 and 0<=number('last_frame_age_s')<1)
        if p.get('module_sku')=='P3767-0000':
            # Only accelerate an idle preview. Detection and recording retain their protected budget.
            requested=number('configured_capture_fps')
            healthy_nx=(healthy_rgb_preview and p.get('platform_configuration_consistent') is True and
                requested in (15,30,60,120) and number('capture_hz')>=.9*requested and
                number('mem_available_kib')>=6*1024*1024)
            return 1024*1024 if healthy_nx else 256*1024
        known_nano=p.get('module_sku') in ('P3767-0003','P3767-0004','P3767-0005') and p.get('platform_configuration_consistent') is True
        return 768*1024 if healthy_rgb_preview and known_nano else 256*1024
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
        operation_started=time.monotonic();timings={};total=0
        def report(code,stage):
            progress({'phase_code':code,'stage':stage,'downloaded_bytes':0,'total_bytes':total,
                      'bytes_s':0,'eta_s':None,'speed_mode':'采集中限速','limit_bytes_s':256*1024,
                      'elapsed_s':time.monotonic()-operation_started})
        def measured(name,work):
            tick=time.monotonic()
            try:return work()
            finally:timings[name]=timings.get(name,0.0)+time.monotonic()-tick
        if not re.fullmatch('[a-zA-Z0-9_-]{1,80}',session_id):raise ValueError('采集场次 ID 无效')
        report('metadata','读取场次')
        sessions=measured('metadata_s',lambda:self.board.call('list_sessions'));session=next((s for s in sessions if s['session_id']==session_id),None)
        if session is None:raise ValueError('采集场次未登记')
        if session['summary'].get('state') in ('recording','finalizing') or session['summary'].get('tail_metadata_may_arrive') is not False:
            raise ValueError('场次元数据尚未可靠收尾；先结束并校验场次，再下载')
        files=self.select_files(session['files'],kind)
        if not files:raise ValueError('该场次还没有所选文件，请先停止相机后生成')
        measured('registry_s',lambda:self.registry.update([dict(session,display_name=session.get('display_name',session_id))]))
        total=sum(f['bytes'] for f in files)
        report('connect','连接传输')
        with self.lock:
            client=measured('ssh_connect_s',self.board.connection)
            with client as connected:
                result=self._transfer(connected,session_id,files,kind,cancelled,progress,measured,timings,operation_started)
            # Selective MP4 transfers still need the authoritative segment list
            # to assemble a complete scene. RPC already supplies these small
            # snapshots; no extra SVO/log download is required.
            result['session_summary']=session['summary']
            folder=pathlib.Path(result['folder'])
            for name,value in [('manifest.json',session.get('manifest',{})),('session_summary.json',session['summary'])]:
                temp=folder/(name+'.download.tmp')
                temp.write_text(json.dumps(value,ensure_ascii=False),encoding='utf-8');temp.replace(folder/name)
            return result

    def _transfer(self,client,session_id,files,kind,cancelled,progress,measured,timings,operation_started):
        # The status cache is provided by Panel's existing poller. Standalone CLI
        # callers use a separate RPC channel on this same authenticated transport.
        rpc=getattr(self.board,'call_on',None)
        def call(action,params=None):
            return rpc(client,action,params) if rpc else self.board.call(action,params)
        sftp=measured('sftp_open_s',client.open_sftp);base='/home/uavlab/Jetson_ROS2_V0/perception_records/'+session_id
        dest=self.registry.folder(session_id,create=True);done=0;total=sum(f['bytes'] for f in files);checks=[]
        transferred=0;transfer_elapsed=0.0;last_poll=None;limit=256*1024
        def report(code,stage,file=None):
            rate=transferred/max(.001,transfer_elapsed)
            # Keep transfer rate independent of remote hash and metadata stages.
            progress_value={'phase_code':code,'stage':stage,'downloaded_bytes':done,'total_bytes':total,
                'file':file,'bytes_s':rate,'eta_s':(total-done)/max(1,rate) if rate else None,
                'speed_mode':('停止后高速' if not limit else '预览中加速' if limit>=768*1024 else '采集中限速'),'limit_bytes_s':limit,
                'elapsed_s':time.monotonic()-operation_started}
            progress(progress_value)
        def update_limit():
            nonlocal last_poll,limit
            if self.status_provider or last_poll is None or time.monotonic()-last_poll>=1:
                try:
                    state=self.status_provider() if self.status_provider else measured('status_rpc_s',lambda:call('status'))
                    limit=self.transfer_limit(state)
                except Exception:limit=256*1024
                last_poll=time.monotonic()
        try:
            for info in files:
                rel=pathlib.PurePosixPath(info['name'])
                if rel.is_absolute() or '..' in rel.parts:raise ValueError('非法文件名')
                remote=base+'/'+str(rel)
                def checked_path():
                    for i in range(1,len(rel.parts)+1):
                        attr=sftp.lstat(base+'/'+str(pathlib.PurePosixPath(*rel.parts[:i])))
                        if stat.S_ISLNK(attr.st_mode):raise ValueError('不下载符号链接')
                measured('path_check_s',checked_path)
                target=dest/pathlib.Path(*rel.parts);target.parent.mkdir(parents=True,exist_ok=True)
                if not target.resolve().is_relative_to(dest.resolve()):raise ValueError('本机文件路径越界')
                if target.is_file() and target.stat().st_size==info['bytes']:
                    report('remote_hash','校验',str(rel))
                    source=measured('remote_checksum_s',lambda:call('session_checksum',{'session_id':session_id,'file':str(rel)}))
                    report('local_hash','校验',str(rel))
                    def existing_hash():
                        existing_sha=hashlib.sha256()
                        with target.open('rb') as existing:
                            while chunk:=existing.read(1024*1024):existing_sha.update(chunk)
                        return existing_sha.hexdigest()
                    if measured('local_hash_s',existing_hash)==source['sha256']:
                        done+=info['bytes'];checks.append({'file':str(rel),'sha256':source['sha256'],'reused':True})
                        report('reused','已存在',str(rel));continue
                part=target.with_name(target.name+'.part');offset=part.stat().st_size if part.exists() else 0
                if offset>info['bytes']:raise ValueError('续传文件大于源文件')
                if shutil.disk_usage(dest).free<info['bytes']-offset+1024**3:raise ValueError('电脑空间不足，需保留 1 GiB')
                sha=hashlib.sha256()
                with sftp.open(remote,'rb') as src:
                    if offset:
                        report('local_hash','检查续传',str(rel))
                        def partial_hash():
                            with part.open('rb') as existing:
                                while chunk:=existing.read(65536):sha.update(chunk)
                        measured('local_hash_s',partial_hash)
                        src.seek(offset)
                    done+=offset
                    with part.open('ab') as out:
                        position=offset
                        while position<info['bytes']:
                            # Bounded 512 KiB readv batch, no whole-file prefetch cache.
                            reads=[(n,min(32768,info['bytes']-n)) for n in range(position,min(position+524288,info['bytes']),32768)]
                            chunks=iter(src.readv(reads))
                            for _ in reads:
                                update_limit()
                                tick=time.monotonic();chunk=next(chunks)
                                if cancelled():raise ValueError('下载已取消，保留 .part 以便续传')
                                if not chunk:raise ValueError('源文件提前结束')
                                out.write(chunk);sha.update(chunk);done+=len(chunk);position+=len(chunk);transferred+=len(chunk)
                                if shutil.disk_usage(dest).free<1024**3:raise ValueError('电脑磁盘余量不足，保留续传文件')
                                if limit:
                                    delay=max(0,len(chunk)/limit-(time.monotonic()-tick))
                                    if delay:measured('throttle_s',lambda:time.sleep(delay))
                                transfer_elapsed+=time.monotonic()-tick
                                report('transfer','传输',str(rel))
                if part.stat().st_size!=info['bytes']:raise ValueError('下载大小不匹配')
                # Source hash computed remotely through a fixed registered-path RPC.
                report('remote_hash','校验',str(rel))
                source=measured('remote_checksum_s',lambda:call('session_checksum',{'session_id':session_id,'file':str(rel)}))
                if source['sha256']!=sha.hexdigest():raise ValueError('下载校验失败；保留临时文件')
                measured('local_publish_s',lambda:part.replace(target));checks.append({'file':str(rel),'sha256':sha.hexdigest()})
            (dest/('download_integrity_'+kind+'.json')).write_text(json.dumps(checks),encoding='utf-8')
            timings.update(transfer_s=transfer_elapsed,total_download_s=time.monotonic()-operation_started)
            report('complete','下载完成')
            return {'folder':str(dest),'session_id':session_id,'files':checks,'kind':kind,'bytes':total,
                    'elapsed_s':timings['total_download_s'],'timings_s':timings,'transfer_bytes':transferred,
                    'transfer_bytes_s':transferred/max(.001,transfer_elapsed)}
        finally:
            if hasattr(sftp,'close'):sftp.close()
