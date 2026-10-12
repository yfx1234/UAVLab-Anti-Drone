#!/usr/bin/env python3
"""Fixed perception-only RPC over SSH stdin. Does not invoke PX4 or flight scripts."""
import contextlib
import hashlib
import json
import math
import os
import pathlib
import re
import shlex
import signal
import shutil
import subprocess
import sys
import time
import uuid

WS=pathlib.Path(__file__).resolve().parents[3]
ROOT=pathlib.Path('/tmp/zed_perception')
RECORDS=WS.parent/'perception_records'
EXE=WS/'install_perception/zed_tracking/lib/zed_tracking/zed_perception_standalone'
BOOT_FILE=pathlib.Path('/proc/sys/kernel/random/boot_id')
SCHEMAS={
 'status':set(),'check_device':set(),'start_capture':{'profile','model_id','capture_fps'},'stop_capture':set(),
 'set_detector':{'enabled'},'start_raw_record':{'label','scene','lighting','size_definition','known_distance_m','distance_source','client_started_at_unix_s'},
 'stop_raw_record':set(),'list_sessions':set(),'list_models':set(),'test_model':{'model_id'},
 'select_model':{'model_id'},'save_calibration':{'target_size_m','size_definition','samples'},
 'export_session':{'session_id','fps','mode'},'export_diagnostics':set(),
 'session_checksum':{'session_id','file'},'ready_media':{'session_id','mode'},
 'prepare_dataset':set(),
 'set_clock_reference':{'pc_unix_s'},'rename_session':{'session_id','name'},'delete_session':{'session_id','confirmation'},
}
class Conflict(ValueError):pass

def validate_action(action,params):
    if action not in SCHEMAS or not isinstance(params,dict) or set(params)-SCHEMAS[action]:raise ValueError('unsupported perception action/fields')
    if action=='start_capture' and params.get('profile') not in ('rgb','perception'):raise ValueError('profile must be rgb/perception')
    if action=='start_capture' and (type(params.get('capture_fps',15)) is not int or params.get('capture_fps',15) not in (15,30)):
        raise ValueError('capture FPS must be15/30')
    if action=='set_detector' and type(params.get('enabled')) is not bool:raise ValueError('enabled must be bool')
    if 'known_distance_m' in params:
        d=params['known_distance_m']
        if isinstance(d,bool) or not isinstance(d,(int,float)) or not math.isfinite(d) or d<=0:raise ValueError('distance must be positive finite')
    for key in ('label','scene','lighting','size_definition','distance_source'):
        if key in params and (not isinstance(params[key],str) or len(params[key])>300):raise ValueError('invalid metadata string')
    if 'client_started_at_unix_s' in params:
        from clock_reference import reference
        reference(params['client_started_at_unix_s'],'validate')

def session_path(root,session_id):
    if not isinstance(session_id,str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}',session_id):raise ValueError('invalid session ID')
    return root/session_id

def validate_expectation(state,request):
    for key in ('boot_id','run_id','revision'):
        if request.get('expected_'+key)!=state.get(key):raise Conflict('state changed; refresh before action')

def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with tmp.open('w',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,allow_nan=False)
    os.replace(tmp,path)

def read_json(path,default=None):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return {} if default is None else default

@contextlib.contextmanager
def state_lock():
    import fcntl
    ROOT.mkdir(parents=True,exist_ok=True)
    with (ROOT/'control.lock').open('a') as f:
        fcntl.flock(f,fcntl.LOCK_EX)
        try:yield
        finally:fcntl.flock(f,fcntl.LOCK_UN)

def proc_start(pid):
    try:return pathlib.Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except OSError:return None

def owns(state):
    pid=state.get('pid')
    if not pid or state.get('boot_id')!=BOOT_FILE.read_text().strip() or proc_start(pid)!=state.get('process_start'):return False
    try:
        cmd=pathlib.Path(f'/proc/{pid}/cmdline').read_bytes().decode(errors='replace')
        return state['run_id'] in cmd and 'zed_perception_standalone' in cmd
    except OSError:return False

def state():
    boot=BOOT_FILE.read_text().strip();s=read_json(ROOT/'supervisor.json')
    if s.get('boot_id')!=boot:s={'boot_id':boot,'run_id':'','revision':0,'pid':None}
    if not owns(s):s.update(pid=None,run_id='',camera_state='stopped')
    node=read_json(ROOT/'status.json')
    if s.get('run_id') and node.get('run_id')==s['run_id']:
        s['pipeline']=node;s['camera_state']=node.get('camera_state','starting')
        s['last_status_age_s']=max(0,time.monotonic()-node.get('monotonic_s',0))
        if s['last_status_age_s']>4:s['camera_state']='unresponsive'
    elif s.get('pid'):s['camera_state']='starting'
    s['api_version']=2
    from clock_reference import estimate
    estimated=estimate(read_json(ROOT/'clock_reference.json'),boot)
    s['clock']={'board_unix_s':time.time(),'estimated_unix_s':estimated,'status':'pc_referenced' if estimated else 'board_clock',
                'offset_from_pc_s':None if estimated is None else time.time()-estimated}
    return s

def models():
    paths=sorted((WS.parent/'weights/candidates').glob('**/*.engine'))
    result=[]
    for p in paths:
        # A basename is never treated as an arbitrary browser-supplied path.
        sha=hashlib.sha256()
        with p.open('rb') as stream:
            while block:=stream.read(1024*1024):sha.update(block)
        mid=sha.hexdigest()
        side=read_json(p.with_suffix('.manifest.json'))
        result.append({'model_id':mid,'engine_sha256':mid,'name':p.name,'path':str(p),'bytes':p.stat().st_size,
                       'recognition_validated':False,'manifest':side,
                       'default':p.name=='exp_opt3.engine'})
    return result

def model_by_id(mid=None):
    candidates=models()
    for m in candidates:
        if m['model_id']==mid or (mid is None and m['default']):return m
    raise ValueError('model not registered')

def run_sdk_tool(args,timeout=45):
    # RPC starts from system Python, without an interactive ROS shell.
    setup=' && '.join('source '+shlex.quote(str(p)) for p in
        [pathlib.Path('/opt/ros/humble/setup.bash'),WS/'install/setup.bash',WS/'install_perception/setup.bash'])
    return subprocess.run(['bash','-c',setup+' && exec '+shlex.join([str(a) for a in args])],
                          capture_output=True,text=True,timeout=timeout)

def stage_model(model):
    folder=WS.parent/'perception_config/models';folder.mkdir(parents=True,exist_ok=True)
    target=folder/(model['engine_sha256']+'.engine')
    if not target.exists():
        tmp=target.with_suffix('.tmp');shutil.copyfile(model['path'],tmp)
        sha=hashlib.sha256()
        with tmp.open('rb') as f:
            while block:=f.read(1024*1024):sha.update(block)
        if sha.hexdigest()!=model['engine_sha256']:
            tmp.unlink(missing_ok=True);raise ValueError('engine changed during snapshot; retry')
        os.replace(tmp,target);target.chmod(0o444)
    return str(target)

def node_command(s,action,params,rid):
    if not owns(s) or s.get('camera_state')!='running':raise Conflict('camera must be running')
    result_path=ROOT/'responses'/f'{rid}.json'
    result_path.unlink(missing_ok=True)
    node_revision=read_json(ROOT/'node_revision.json')
    if node_revision.get('run_id')!=s['run_id']:raise Conflict('node revision unavailable for this run')
    atomic_json(ROOT/'command.json',{'request_id':rid,'run_id':s['run_id'],'expected_config_revision':node_revision['node_config_revision'],'operation':action,'params':params})
    end=time.monotonic()+25
    while time.monotonic()<end:
        if result_path.exists():
            result=read_json(result_path)
            if not result.get('ok'):raise ValueError(result.get('error','node command failed'))
            return result
        if not owns(s):raise ValueError('camera process ended before command completed')
        time.sleep(.05)
    raise TimeoutError('node command timeout; recheck actual state')

def start(s,params):
    if owns(s):raise Conflict('camera already owned')
    if not EXE.is_file():raise ValueError('standalone executable not built')
    m=model_by_id(params.get('model_id')) if params['profile']=='perception' else None
    run=uuid.uuid4().hex
    args=[str(EXE),'--ros-args','-p',f'run_id:={run}','-p',f'profile:={params["profile"]}',
          '-p',f'capture_fps:={params.get("capture_fps",15)}']
    if m:args.extend(['-p',f'engine_path:={stage_model(m)}','-p',f'model_id:={m["model_id"]}'])
    command=f'source /opt/ros/humble/setup.bash && source {shlex.quote(str(WS/"install/setup.bash"))} && source {shlex.quote(str(WS/"install_perception/setup.bash"))} && exec '+shlex.join(args)
    log=ROOT/f'capture_{run}.log'
    with log.open('ab') as f:
        proc=subprocess.Popen(['bash','-c',command],stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)
    s.update(run_id=run,pid=proc.pid,process_start=proc_start(proc.pid),profile=params['profile'],model_id=m['model_id'] if m else '',camera_state='starting')
    return {'run_id':run,'camera_state':'starting'}

def stop(s):
    if not owns(s):return {'stopped':True}
    os.kill(s['pid'],signal.SIGINT)
    end=time.monotonic()+35
    while time.monotonic()<end:
        if not owns(s):s.update(pid=None,run_id='',camera_state='stopped');return {'stopped':True}
        time.sleep(.1)
    raise TimeoutError('owned process did not finalize; no broad/forced kill performed')

def device_info():
    mem={line.split(':')[0]:line.split(':')[1].strip() for line in pathlib.Path('/proc/meminfo').read_text().splitlines()}
    caps={'memory':mem,'video_devices':[str(p) for p in pathlib.Path('/dev').glob('video*')],
          'sdk_version':'5.5','default_profile':'nano_lossless','nvenc_verified':False,'nx16_ready':False}
    try:caps['l4t']=pathlib.Path('/etc/nv_tegra_release').read_text().strip()
    except OSError:pass
    # Fixed, bounded read-only diagnostics; never accept a command from the UI.
    for key,args in {
        'network':['ip','-j','address'],
        'routes':['ip','-j','route'],
        'camera_services':['systemctl','is-active','zed_x_daemon','IMU_Daemon','nvargus-daemon'],
        'camera_service_logs':['journalctl','-u','nvargus-daemon','-u','zed_x_daemon','-u','IMU_Daemon','-n','45','--no-pager'],
    }.items():
        try:
            probe=subprocess.run(args,capture_output=True,text=True,timeout=4)
            caps[key]={'returncode':probe.returncode,'output':probe.stdout[-24000:],'error':probe.stderr[-1000:]}
        except (OSError,subprocess.TimeoutExpired) as e:caps[key]={'error':str(e)}
    caps['board_wall_time']=time.strftime('%Y-%m-%d %H:%M:%S %Z')
    caps['clock_note']='Use boot_id/run_id/monotonic time for frame identity; verify wall clock before field capture.'
    return caps

def sessions():
    from session_catalog import list_sessions
    return list_sessions(RECORDS)

def dispatch(request):
    action=request.get('action');params=request.get('params',{});validate_action(action,params)
    rid=request.get('request_id',uuid.uuid4().hex)
    if not isinstance(rid,str) or not re.fullmatch('[a-zA-Z0-9_-]{1,80}',rid):raise ValueError('invalid request ID')
    readonly={'status','check_device','list_sessions','list_models','export_diagnostics','session_checksum','ready_media'}
    if action in readonly:
        if action=='status':return state()
        if action=='check_device':return device_info()
        if action=='list_sessions':return sessions()
        if action=='list_models':return models()
        if action=='ready_media':
            from export_session import ready_media
            return ready_media(params.get('session_id'),params.get('mode'))
        if action=='session_checksum':
            folder=session_path(RECORDS,params.get('session_id'))
            rel=pathlib.PurePosixPath(params.get('file',''))
            if not rel.parts or rel.is_absolute() or '..' in rel.parts:raise ValueError('invalid file')
            path=folder.joinpath(*rel.parts)
            if not path.resolve().is_relative_to(folder.resolve()):raise ValueError('file escapes session')
            if not path.is_file():raise ValueError('file missing')
            sha=hashlib.sha256()
            with path.open('rb') as f:
                while block:=f.read(1024*1024):sha.update(block)
            return {'sha256':sha.hexdigest(),'bytes':path.stat().st_size}
        return {'state':state(),'device':device_info(),'sessions':sessions()}
    with state_lock():
        s=state();cached=read_json(ROOT/'requests.json')
        if rid in cached:return cached[rid]
        validate_expectation(s,request)
        if action=='start_capture':result=start(s,params)
        elif action=='stop_capture':result=stop(s)
        elif action=='set_clock_reference':
            from clock_reference import reference
            anchor=reference(params.get('pc_unix_s'),s['boot_id']);atomic_json(ROOT/'clock_reference.json',anchor);result={'clock_reference':anchor}
            helper=pathlib.Path('/usr/local/libexec/zed-clock-sync')
            if helper.is_file() and not s.get('pid') and abs(time.time()-anchor['pc_unix_s'])>2:
                synced=subprocess.run(['sudo','-n',str(helper)],input=json.dumps({'pc_unix_s':anchor['pc_unix_s'],'boot_id':s['boot_id']}),capture_output=True,text=True,timeout=12)
                try:result['system_clock_sync']=json.loads(synced.stdout)
                except ValueError:result['system_clock_sync']={'synced':False,'error':synced.stderr[-500:]}
            else:result['system_clock_sync']={'synced':False,'reason':'camera_active' if s.get('pid') else 'helper_not_installed_or_clock_close'}
        elif action=='rename_session':
            from session_catalog import rename
            result=rename(RECORDS,params.get('session_id'),params.get('name'))
        elif action=='delete_session':
            from session_catalog import delete
            result=delete(RECORDS,params.get('session_id'),params.get('confirmation'),s.get('pipeline',{}).get('session_id',''))
        elif action in ('set_detector','start_raw_record','stop_raw_record','save_calibration'):
            if action=='set_detector' and params['enabled']:
                model=model_by_id(s.get('model_id') or None)
                if s.get('pipeline',{}).get('record_state')=='recording' and model['model_id']!=s.get('model_id'):
                    raise Conflict('finish recording before changing model identity')
                params=dict(params,engine_path=stage_model(model),model_id=model['model_id'])
            result=node_command(s,action,params,rid)
            if action=='set_detector' and params['enabled']:s['model_id']=params['model_id']
        elif action in ('test_model','select_model'):
            if s.get('pipeline',{}).get('record_state') not in (None,'idle','error'):raise Conflict('finish recording before changing models')
            model=model_by_id(params.get('model_id'))
            if action=='test_model' and owns(s):raise Conflict('stop camera before engine validation')
            previous=s.get('model_id');previous_profile=s.get('profile','rgb');was_running=owns(s)
            if action=='select_model':stop(s)
            try:
                tested=run_sdk_tool([EXE,'--check-model',model['path']])
                if tested.returncode:raise ValueError('engine validation failed: '+tested.stderr[-1200:])
                result={'model':model,'engine_test':json.loads(tested.stdout.strip().splitlines()[-1])}
                if action=='select_model':result.update(start(s,{'profile':'perception','model_id':model['model_id']}))
            except Exception:
                if action=='select_model' and was_running:
                    restore={'profile':previous_profile}
                    if previous:restore['model_id']=previous
                    start(s,restore)
                    s['revision']=s.get('revision',0)+1
                    atomic_json(ROOT/'supervisor.json',{k:v for k,v in s.items() if k not in ('pipeline','last_status_age_s')})
                raise
        elif action=='prepare_dataset':
            if owns(s):raise Conflict('stop camera before preparing dataset index')
            from export_session import prepare_dataset_index
            index=prepare_dataset_index(RECORDS)
            result={'images':len(index['images']),'splits':index['splits'],'index':str(RECORDS/'dataset_index.json')}
        elif action=='export_session':
            if owns(s):raise Conflict('stop camera before offline SVO export')
            session_path(RECORDS,params.get('session_id'))
            tool=pathlib.Path(__file__).with_name('export_session.py')
            run=run_sdk_tool([sys.executable,tool,'--session',params['session_id'],'--fps',str(params.get('fps',2)),
                                '--mode',params.get('mode','frames')],timeout=3600)
            if run.returncode:raise ValueError(run.stderr[-1500:])
            result=json.loads(run.stdout.strip().splitlines()[-1])
        else:raise ValueError('action unavailable')
        s['revision']=s.get('revision',0)+1
        atomic_json(ROOT/'supervisor.json',{k:v for k,v in s.items() if k not in ('pipeline','last_status_age_s')})
        result={'ok':True,'result':result,'state':state()};cached[rid]=result
        atomic_json(ROOT/'requests.json',dict(list(cached.items())[-64:]))
        return result

if __name__=='__main__':
    try:
        raw=sys.stdin.buffer.read(65537)
        if len(raw)>65536:raise ValueError('RPC request too large')
        response=dispatch(json.loads(raw));print(json.dumps(response,ensure_ascii=False,allow_nan=False))
    except Exception as e:
        print(json.dumps({'ok':False,'error':str(e),'error_code':'conflict' if isinstance(e,Conflict) else 'error'}));sys.exit(1)
