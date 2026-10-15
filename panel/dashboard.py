"""Perception platform on loopback 8767. Camera lifetime belongs to the board."""
import hmac
import http.server
import json
import math
import os
import pathlib
import re
import secrets
import threading
import time
import uuid
import urllib.parse
from board_client import BoardClient,HOST
from frame_receiver import FrameReceiver
from job_manager import JobManager
from preview_recorder import PreviewRecorder
from session_manager import SessionManager
from media_library import MediaLibrary
from scene_registry import SceneRegistry

BASE=pathlib.Path(__file__).resolve().parent
OUTPUT=pathlib.Path(os.environ.get('UAVLAB_OUTPUT_DIRECTORY',str(BASE.parents[1]/'outputs'/'perception_data'))).resolve()
BOARD_ACTIONS={'check_device','start_capture','stop_capture','set_detector','start_raw_record','stop_raw_record',
               'list_sessions','list_models','test_model','select_model','save_calibration','export_session','export_diagnostics','prepare_dataset','rename_session','delete_session','save_session_metadata'}
LOCAL_ACTIONS={'connect','disconnect','start_preview_record','stop_preview_record','download_session','download_asset','extract_local_frames','cancel_job','open_media_folder',
               'interceptor_start','interceptor_stop'}
INTERCEPTOR_STATUS_CMD='python3 /home/uavlab/interceptor_ws/deploy/interceptor_status.py'
INTERCEPTOR_CONTROL_CMD='bash /home/uavlab/interceptor_ws/deploy/run_interceptor_stack.sh'

def valid_request(host,origin,token,expected):
    return host in ('127.0.0.1:8767','localhost:8767') and origin=='http://'+host and isinstance(token,str) and hmac.compare_digest(token,expected)

class Panel:
    def __init__(self):
        self.token=secrets.token_urlsafe(32);self.instance_id=uuid.uuid4().hex;self.lock=threading.RLock();self.board=BoardClient();self.jobs=JobManager()
        self.connected=False;self.state={};self.state_observed_at=0.0;self.control_roundtrip_ms=None;self.error=None;self.closed=threading.Event();self.local_revision=0
        registry=SceneRegistry.shared(OUTPUT/'board_sessions',OUTPUT/'preview_videos')
        self.receiver=FrameReceiver();self.recorder=PreviewRecorder(OUTPUT/'preview_videos',registry);self.receiver.listener=self.recorder.consume
        self.sessions=SessionManager(self.board,OUTPUT/'board_sessions',registry,self.transfer_state)
        self.media=MediaLibrary(OUTPUT/'board_sessions',OUTPUT/'preview_videos',registry)
        self.cancelled=set();self.progress={};self.events=[];self.scene_jobs={}
        self.interceptor={};self.interceptor_error=None;self.interceptor_observed_at=0.0
        self.thread=threading.Thread(target=self._poll,daemon=True);self.thread.start()
        self.interceptor_thread=threading.Thread(target=self._interceptor_poll,daemon=True);self.interceptor_thread.start()
    def _poll(self):
        while not self.closed.wait(1):
            if not self.connected:continue
            try:
                started=time.monotonic();state=self.board.call('status',timeout=12)
                with self.lock:
                    if not self.connected:continue
                    if state.get('boot_id')==self.state.get('boot_id') and state.get('revision',0)<self.state.get('revision',0):continue
                    self.observe_state(state,started);self.error=None
                self.receiver.set_enabled(bool(state.get('pid')))
            except Exception as e:
                with self.lock:self.error=str(e)
    def _interceptor_poll(self):
        """2s 轮询拦截链状态（~/interceptor_ws 部署，只读脚本，日志解析在板上完成）。"""
        while not self.closed.wait(2):
            if not self.connected:continue
            try:
                raw=self.board.exec_command(INTERCEPTOR_STATUS_CMD,timeout=12)
                data=json.loads(raw.strip().splitlines()[-1])
                if not isinstance(data,dict) or 'running' not in data:raise ValueError('拦截链状态格式无效')
                with self.lock:self.interceptor=data;self.interceptor_error=None;self.interceptor_observed_at=time.monotonic()
            except Exception as e:
                with self.lock:self.interceptor_error=str(e)
    def observe_state(self,state,request_started):
        received=time.monotonic();transit=max(0,received-request_started)
        state=state.copy()
        def age(value):
            return value+transit if isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>=0 else None
        if 'last_status_age_s' in state:state['last_status_age_s']=age(state['last_status_age_s'])
        if isinstance(state.get('hardware'),dict):state['hardware']=dict(state['hardware'],sample_age_s=age(state['hardware'].get('sample_age_s')))
        self.receiver.observe_board_clock(state,request_started)
        with self.lock:
            changed=state.get('run_id')!=getattr(self,'state',{}).get('run_id')
            self.state=state;self.state_observed_at=received;self.control_roundtrip_ms=transit*1000
        if changed and hasattr(self.receiver,'cache'):self.receiver.cache.clear()
    def transfer_state(self):
        """Never trust an idle state after polling stalls or the panel disconnects."""
        with self.lock:
            elapsed=time.monotonic()-self.state_observed_at
            if not self.connected or self.error or elapsed>3:return {}
            state=self.state.copy();node_age=state.get('last_status_age_s')
            if isinstance(node_age,(int,float)) and not isinstance(node_age,bool) and node_age>=0:
                state['last_status_age_s']=node_age+elapsed
            return state
    def close(self):
        self.closed.set();self.receiver.close()
        if self.recorder.active:self.recorder.stop()
    def snapshot(self):
        with self.lock:
            state=self.state.copy();node_age=state.get('last_status_age_s')
            if isinstance(node_age,(int,float)) and not isinstance(node_age,bool) and node_age>=0:
                state['last_status_age_s']=node_age+max(0,time.monotonic()-self.state_observed_at)
            hardware=state.get('hardware')
            if isinstance(hardware,dict):
                age=hardware.get('sample_age_s')
                state['hardware']=dict(hardware,sample_age_s=age+max(0,time.monotonic()-self.state_observed_at) if isinstance(age,(int,float)) and math.isfinite(age) and age>=0 else None)
            return {'api_version':2,'instance_id':self.instance_id,'host':HOST,'connected':self.connected,'state':state,'error':self.error,
                    'control_roundtrip_ms':getattr(self,'control_roundtrip_ms',None),
                    'local_revision':self.local_revision,'preview':self.receiver.snapshot(),'preview_recording':self.recorder.snapshot(),
                    'interceptor':self.interceptor,'interceptor_error':self.interceptor_error,
                    'interceptor_age_s':max(0.0,time.monotonic()-self.interceptor_observed_at) if self.interceptor_observed_at else None,
                    'events':self.events[-15:],'output_directory':str(OUTPUT)}
    def submit(self,data):
        action=data.get('action');params=data.get('params',{})
        if action not in BOARD_ACTIONS|LOCAL_ACTIONS:raise ValueError('仅支持已批准的感知动作；旧飞行动作已移除')
        if not isinstance(params,dict):raise ValueError('参数必须是对象')
        rid=data.get('request_id',uuid.uuid4().hex)
        if not isinstance(rid,str) or not re.fullmatch('[a-zA-Z0-9_-]{1,80}',rid):raise ValueError('请求 ID 无效')
        # Reserve at submission, including queued work, so delete cannot race a transfer.
        scene_action=action in ('download_session','download_asset','export_session','extract_local_frames','rename_session','delete_session','save_session_metadata')
        sid=params.get('session_id') if scene_action else None
        with self.lock:
            with self.jobs.lock:
                if rid in self.jobs.ids:return self.jobs.ids[rid]
            if sid:
                if sid in self.scene_jobs:raise ValueError('该场次正在下载、转换或修改，请完成后再操作')
                self.scene_jobs[sid]=rid
        def work():
            if action=='connect':
                password=data.get('password')
                if password is not None and not isinstance(password,str):raise ValueError('密码字段无效')
                self.board.password=password
                state=self.board.call('status')
                request=dict(request_id=uuid.uuid4().hex,**{'expected_'+k:state[k] for k in ('boot_id','run_id','revision')})
                started=time.monotonic();state=self.board.call('set_clock_reference',{'pc_unix_s':time.time()},request)['state']
                with self.lock:self.connected=True;self.observe_state(state,started);self.error=None
                self.receiver.set_enabled(bool(state.get('pid')));return {'connected':True}
            if action=='disconnect':
                with self.lock:self.connected=False;self.board.password=None
                self.receiver.set_enabled(False);return {'connected':False,'board_capture_continues':True}
            if action=='cancel_job':
                with self.lock:self.cancelled.add(params['request_id'])
                return {'cancel_requested':True}
            if action in ('start_preview_record','stop_preview_record'):
                with self.lock:
                    if data.get('expected_local_revision')!=self.local_revision:raise ValueError('本机录像状态已改变，请刷新')
                    self.local_revision+=1
                if action=='start_preview_record':
                    p=self.receiver.cache.latest()
                    if p is None or time.monotonic()-p.received_at>3:raise ValueError('当前没有新鲜图像')
                    with self.lock:source_session_id=self.state.get('pipeline',{}).get('session_id') or None
                    if source_session_id and source_session_id not in self.media.registry.scenes:
                        self.media.scene_names(self.board.call('list_sessions'))
                    return self.recorder.start(params.get('view_mode','rgb'),source_session_id)
                return self.recorder.stop()
            if action=='extract_local_frames':return self.media.extract(params['session_id'],params.get('fps',2))
            if action=='open_media_folder':return self.media.open_folder(params['relative'])
            if not self.connected:raise ValueError('请先连接板子')
            if action in ('interceptor_start','interceptor_stop'):
                # 拦截链启停：常量脚本路径经 SSH 执行；启动 2s 后栈自检，
                # 输出末行含 pid/日志路径或失败原因
                out=self.board.exec_command(
                    INTERCEPTOR_CONTROL_CMD+' '+action.removeprefix('interceptor_'),timeout=45)
                lines=[line for line in out.splitlines() if line.strip()]
                return {'output':lines[-3:]}
            if action in ('download_session','download_asset'):
                download_started=time.monotonic();timings={}
                def progress(value):
                    with self.lock:self.progress[rid]=value
                kind=params.get('kind','all') if action=='download_asset' else 'all'
                self.sessions.select_files([],kind)
                if kind in ('rgb','depth'):
                    progress({'phase_code':'ready_check','stage':'检查视频'})
                    tick=time.monotonic()
                    ready=self.board.call('ready_media',{'session_id':params['session_id'],'mode':kind+'_video'})
                    timings['ready_check_s']=time.monotonic()-tick
                    if not ready['ready']:
                        fresh=self.board.call('status')
                        if fresh.get('pid'):raise ValueError('该视频尚未生成；请先停止相机，再下载。已生成的视频可在采集中下载。')
                        progress({'phase_code':'board_convert','stage':'板上转换','cancel_after_stage':True})
                        request=dict(request_id=uuid.uuid4().hex,**{'expected_'+k:fresh[k] for k in ('boot_id','run_id','revision')})
                        tick=time.monotonic()
                        converted=self.board.call('export_session',{'session_id':params['session_id'],'mode':kind+'_video','fps':15},request,timeout=3700)
                        ready=converted['result']
                        timings['board_convert_s']=time.monotonic()-tick
                        if converted['result']['state']!='complete':raise ValueError('场次不完整，不能作为有效素材下载：'+'；'.join(converted['result']['integrity_errors']))
                if rid in self.cancelled:raise ValueError('已取消下载；板上转换结果保留')
                result=self.sessions.download(params['session_id'],progress,lambda:rid in self.cancelled,kind)
                if kind=='depth':
                    reports=ready.get('segments',[])
                    result['depth_quality']={'all_invalid_segments':[r['segment_id'] for r in reports if r.get('depth_valid_sample_fraction')==0],
                                             'meaning':'black pixels are invalid or outside the configured depth range; no distances were invented'}
                result.setdefault('timings_s',{}).update(timings)
                if kind in ('rgb','depth'):
                    progress({'phase_code':'local_prepare','stage':'本机准备'})
                    result=self.media.prepare(result,lambda:rid in self.cancelled)
                result.setdefault('timings_s',{})['total_job_s']=time.monotonic()-download_started
                return result
            allowed={'action','request_id','expected_boot_id','expected_run_id','expected_revision','params'}
            request={key:value for key,value in data.items() if key in allowed}
            started=time.monotonic();result=self.board.call(action,params,request,timeout=3700 if action=='export_session' else 60)
            if action=='list_sessions':self.media.scene_names(result)
            if action=='rename_session':
                try:local_sync=self.media.rename_scene(params['session_id'],result['result']['display_name'])
                except (ValueError,OSError) as e:local_sync={'ok':False,'error':str(e)}
                result['result']['local_sync']=local_sync
            if action=='prepare_dataset':
                result['result']=self.sessions.fetch_dataset_index(result['result'])
            with self.lock:
                if isinstance(result,dict) and 'state' in result:
                    self.observe_state(result['state'],started)
            return result
        def guarded_work():
            try:return work()
            finally:
                with self.lock:
                    if sid and self.scene_jobs.get(sid)==rid:self.scene_jobs.pop(sid,None)
        try:
            with self.lock:
                with self.jobs.lock:existing=self.jobs.ids.get(rid)
                if existing:
                    if sid and self.scene_jobs.get(sid)==rid:self.scene_jobs.pop(sid,None)
                    return existing
                return self.jobs.submit(rid,guarded_work)
        except Exception:
            with self.lock:
                if sid and self.scene_jobs.get(sid)==rid:self.scene_jobs.pop(sid,None)
            raise

def make_handler(panel):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,body,kind='application/json; charset=utf-8',code=200):
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Referrer-Policy','no-referrer');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def js(self,value,code=200):self.send(json.dumps(value,ensure_ascii=False,allow_nan=False).encode(),code=code)
        def do_GET(self):
            if self.headers.get('Host') not in ('127.0.0.1:8767','localhost:8767'):self.js({'error':'仅接受本机访问'},403);return
            path=self.path.split('?',1)[0]
            if path=='/':self.send((BASE/'index.html').read_text(encoding='utf-8').replace('__TOKEN__',panel.token).replace('__INSTANCE__',panel.instance_id).encode(),'text/html; charset=utf-8')
            elif path=='/api/status':self.js(panel.snapshot())
            elif path=='/api/media-list':self.js(panel.media.list())
            elif path.startswith('/api/media/'):
                try:
                    with panel.media.open_file(urllib.parse.unquote(path.removeprefix('/api/media/'))) as stream:
                        size=os.fstat(stream.fileno()).st_size;start=0;end=size-1
                        byte_range=self.headers.get('Range')
                        if byte_range:
                            match=re.fullmatch(r'bytes=(\d+)-(\d*)',byte_range)
                            if not match:raise ValueError('无效视频范围')
                            start=int(match[1]);end=min(end,int(match[2])) if match[2] else end
                            if start>end:raise ValueError('视频范围越界')
                        self.send_response(206 if byte_range else 200);self.send_header('Content-Type','video/mp4')
                        self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
                        self.send_header('X-Content-Type-Options','nosniff')
                        if byte_range:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
                        self.end_headers()
                        stream.seek(start);remaining=end-start+1
                        while remaining:
                            chunk=stream.read(min(262144,remaining))
                            if not chunk:break
                            self.wfile.write(chunk);remaining-=len(chunk)
                except (BrokenPipeError,ConnectionResetError):pass
                except (ValueError,OSError):self.js({'error':'媒体读取失败'},404)
            elif path=='/api/frame/meta':
                p=panel.receiver.cache.latest()
                if p is None or time.monotonic()-p.received_at>3:self.js({'error':'无新鲜图像'},503);return
                meta=p.metadata.copy();meta['image_url']='/api/frame/'+meta['frame_key']+'.jpg';meta['received_age_s']=time.monotonic()-p.received_at;self.js(meta)
            elif re.fullmatch(r'/api/frame/[a-zA-Z0-9_-]+\.jpg',path):
                p=panel.receiver.cache.get(path.removeprefix('/api/frame/').removesuffix('.jpg'))
                if p is None:self.js({'error':'帧已过期，请重新读取元数据'},410)
                else:self.send(p.jpeg,'image/jpeg')
            elif path.startswith('/api/jobs/'):
                try:
                    job=panel.jobs.get(path.rsplit('/',1)[1]);job['progress']=panel.progress.get(job['request_id']);self.js(job)
                except KeyError:self.js({'error':'任务不存在'},404)
            elif path=='/api/export':self.js(panel.snapshot())
            else:self.js({'error':'未找到'},404)
        def do_POST(self):
            if not valid_request(self.headers.get('Host'),self.headers.get('Origin'),self.headers.get('X-Panel-Token'),panel.token):self.js({'error':'请求来源或会话校验失败'},403);return
            try:
                if self.path!='/api/action':self.js({'error':'未找到'},404);return
                size=int(self.headers.get('Content-Length','0'))
                if self.headers.get('Content-Type')!='application/json' or not 0<size<=65536:raise ValueError('请求类型/长度无效')
                data=json.loads(self.rfile.read(size))
                allowed={'action','params','request_id','expected_revision','expected_boot_id','expected_run_id','expected_local_revision','password'}
                if not isinstance(data,dict) or set(data)-allowed:raise ValueError('请求字段无效')
                if 'password' in data and data.get('action')!='connect':raise ValueError('仅连接动作接受密码')
                jid=panel.submit(data);self.js({'operation_id':jid},202)
            except Exception as e:self.js({'error':str(e)},400)
    return Handler

if __name__=='__main__':
    panel=Panel();server=http.server.ThreadingHTTPServer(('127.0.0.1',8767),make_handler(panel))
    print('PERCEPTION_PANEL http://127.0.0.1:8767/ api_version=2',flush=True)
    try:server.serve_forever()
    finally:server.server_close();panel.close()
