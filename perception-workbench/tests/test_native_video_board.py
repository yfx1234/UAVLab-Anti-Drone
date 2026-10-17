"""Opt-in Nano30fps RGB/SVO and native RGB/depth video acceptance; restores prior idle camera."""
import sys,pathlib,json,time,uuid,statistics
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'panel'))
from board_client import BoardClient

def run():
    b=BoardClient();evidence={'actions':[],'metrics':[]};prior=b.call('status');sid=None
    p=prior.get('pipeline',{})
    assert p.get('record_state','idle')=='idle','User recording active: do not interrupt'
    restore=bool(prior.get('pid'))
    def call(action,params=None):
        s=b.call('status');result=b.call(action,params or {},dict(request_id=uuid.uuid4().hex,**{'expected_'+k:s[k] for k in ('boot_id','run_id','revision')}),timeout=3700 if action=='export_session' else 60)
        evidence['actions'].append({'action':action,'result':result});return result
    def ready():
        for _ in range(80):
            s=b.call('status')
            if s['camera_state']=='running':return s
            if s['camera_state']=='error':raise ValueError(str(s))
            time.sleep(.5)
        raise TimeoutError('camera readiness')
    try:
        if restore:call('stop_capture')
        call('start_capture',{'profile':'rgb','capture_fps':30});ready()
        sid=call('start_raw_record',{'label':'Codex_native_30fps_rgb_trial','scene':'bench_background'})['result']['session_id']
        start=time.monotonic()
        while time.monotonic()-start<30:
            evidence['metrics'].append(b.call('status'));time.sleep(2)
        call('stop_raw_record');call('stop_capture')
        for mode in ('verify','rgb_video','depth_video'):
            result=call('export_session',{'session_id':sid,'mode':mode,'fps':2})['result']
            assert result['state']=='complete',result
            for segment in result['segments']:
                if mode!='verify':
                    assert (segment['video_width'],segment['video_height'],segment['video_fps'])==(1920,1080,30),segment
                    assert segment['video_frames']>0
        evidence['session_id']=sid;evidence['export_result']='pass'
        metrics=call('export_session',{'session_id':sid,'mode':'verify','fps':2})['result']['segments']
        evidence['short_30fps_qualified']=all(m['unique_capture_hz']>=28 for m in metrics)
        print('PASS: native video export integrity; actual30fps qualification recorded separately',flush=True)
    finally:
        current=b.call('status')
        if current.get('pid') and current['run_id']!=prior.get('run_id'):call('stop_capture')
        if restore and not b.call('status').get('pid'):
            call('start_capture',{'profile':prior.get('profile','rgb'),'capture_fps':p.get('configured_capture_fps',15)})
            ready()
            if prior.get('profile')=='rgb' and prior.get('model_id'):
                call('set_detector',{'enabled':True})
                if not p.get('detector_enabled'):call('set_detector',{'enabled':False})
        evidence['restored_status']=b.call('status')
        (ROOT/'test-results/native_video_board.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':run()
