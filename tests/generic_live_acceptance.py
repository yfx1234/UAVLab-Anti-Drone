"""Opt-in bench acceptance: one SVO across independent detector transitions.

Run manually with --live on the configured PC. Requires an idle/stopped camera;
does not operate a vehicle or require a physical target. Evidence stays local.
"""
import argparse
import json
import pathlib
import sys
import time
import urllib.request
import uuid

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from board_client import BoardClient

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--live',action='store_true')
    parser.add_argument('--output',default=str(ROOT/'test-results/generic_live_acceptance.json'))
    args=parser.parse_args()
    if not args.live:parser.error('--live is required for hardware actions')
    sys.stdout.reconfigure(encoding='utf-8')
    board=BoardClient();initial=board.call('status');evidence={'initial':initial,'actions':[],'stages':[]}
    if initial.get('pid'):raise RuntimeError('Release camera before running bench acceptance')
    run=None;sid=None
    def status():return board.call('status')
    def command(action,params=None):
        state=status()
        response=board.call(action,params or {},dict(request_id=uuid.uuid4().hex,
            **{'expected_'+key:state[key] for key in ('boot_id','run_id','revision')}),timeout=180 if action=='export_session' else 50)
        evidence['actions'].append({'action':action,'params':params,'response':response})
        return response
    def ready(yolo,generic):
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            state=status();p=state.get('pipeline',{})
            if state.get('camera_state')=='error':raise RuntimeError(str(p.get('errors')))
            if state.get('camera_state')=='running' and p.get('capture_frames',0)>10 and \
                p.get('detector_enabled') is yolo and p.get('generic_detector_enabled') is generic and \
                (not yolo or p.get('detector_state')==2) and (not generic or p.get('generic_detector_state')==2):
                return state
            time.sleep(.3)
        raise TimeoutError('detector state did not become ready')
    def sample(label,yolo,generic,seconds=5):
        stage={'label':label,'states':[],'frames':[]};evidence['stages'].append(stage)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            state=status();p=state['pipeline']
            assert state['run_id']==run and state['pid'],'Camera owner changed during mode switch'
            assert p['record_state']=='recording' and p['session_id']==sid,'Recording changed during detection toggle'
            assert p['depth_enabled'] is (yolo or generic)
            assert p['detector_enabled'] is yolo and p['generic_detector_enabled'] is generic
            assert p.get('mem_available_kib',0)>=1500000 and p.get('max_temperature_c',99)<80
            stage['states'].append(state)
            try:
                with urllib.request.urlopen('http://127.0.0.1:8767/api/frame/meta',timeout=2) as response:
                    meta=json.load(response)
                if meta.get('run_id')==run and meta.get('detector_generation')==p['detector_generation']:
                    assert meta.get('generic_detector_enabled') is generic and meta.get('yolo_enabled') is yolo
                    if generic and not yolo:
                        for target in meta.get('targets',[]):
                            assert target['class_id']==-1 and target['class_confirmed'] is False
                            assert target['score_kind']=='contrast_score' and target['track_id']>=0x80000000
                            assert target['size_estimate_calibrated'] is False
                    if not (yolo or generic):assert not meta.get('targets')
                    stage['frames'].append(meta)
            except urllib.error.HTTPError:pass
            time.sleep(.4)
        assert stage['frames'],'No current-generation preview received'
        state=stage['states'][-1]['pipeline']
        print(json.dumps({'stage':label,'capture_hz':round(state['capture_hz'],2),
              'inference_hz':round(state['inference_hz'],2),'generic_ms':state.get('generic_detector_ms'),
              'yolo_ms':state.get('yolo_detector_ms'),'imu_valid':state.get('camera_motion_valid'),
              'memory_available_gib':round(state['mem_available_kib']/1048576,2),
              'temperature_c':state['max_temperature_c']},ensure_ascii=False),flush=True)
    try:
        command('start_capture',{'profile':'rgb','camera_resolution':'HD1080','capture_fps':15})
        state=ready(False,False);run=state['run_id']
        sid=command('start_raw_record',{'label':'Codex_通用检测连续录像验证','scene':'bench_software_check',
                    'client_started_at_unix_s':time.time()})['result']['session_id']
        sample('RGB',False,False,2)
        command('set_generic_detector',{'enabled':True,'sensitivity':'standard'});ready(False,True)
        sample('generic',False,True,7)
        command('set_detector',{'enabled':True});ready(True,True)
        sample('combined',True,True,7)
        command('set_generic_detector',{'enabled':False});ready(True,False)
        sample('YOLO',True,False,4)
        command('set_detector',{'enabled':False});ready(False,False)
        sample('RGB resumed',False,False,2)
        command('stop_raw_record');command('stop_capture')
        result=command('export_session',{'session_id':sid,'mode':'verify','fps':2})['result']
        evidence['session_id']=sid;evidence['recording_validation']=result
        assert result['state']=='complete',result
        with board.connection() as client,client.open_sftp() as ftp:
            prefix='/home/uavlab/Jetson_ROS2_V0/perception_records/'+sid+'/'
            frames=[json.loads(line) for line in ftp.open(prefix+'frames.jsonl').read().decode().splitlines()]
            observations=[json.loads(line) for line in ftp.open(prefix+'observations.jsonl').read().decode().splitlines()]
            comparison=[json.loads(line) for line in ftp.open(prefix+'detector_comparison.jsonl').read().decode().splitlines()]
            events=[json.loads(line) for line in ftp.open(prefix+'events.jsonl').read().decode().splitlines()]
            assert comparison and len(comparison)==len(observations)
            assert all('camera_quaternion_wxyz' in frame for frame in frames)
            evidence['summary']={'recorded_frames':len(frames),'observations':len(observations),
                'comparison_rows':len(comparison),'imu_valid_frames':sum(f['camera_motion_valid'] for f in frames),
                'toggle_events':sum(event.get('event')=='detector_change' for event in events)}
            assert evidence['summary']['toggle_events']==4
        print('PASS: one camera, one complete SVO, RGB/generic/combined/YOLO/RGB and comparison logs',flush=True)
    finally:
        current=status()
        if run and current.get('run_id')==run and current.get('pid'):
            if current.get('pipeline',{}).get('record_state')=='recording':command('stop_raw_record')
            command('stop_capture')
        output=pathlib.Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
