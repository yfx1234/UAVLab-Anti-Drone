"""Opt-in actual camera: finalized sessions stay immutable across later calibration."""
import sys,pathlib,json,time,uuid,hashlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'panel'))
from board_client import BoardClient
def run():
    b=BoardClient();evidence=[];owned=None
    def call(action,params=None):
        s=b.call('status');result=b.call(action,params or {},dict(request_id=uuid.uuid4().hex,
            expected_boot_id=s['boot_id'],expected_run_id=s['run_id'],expected_revision=s['revision']))
        evidence.append({'action':action,'result':result});return result
    def wait_ready():
        for _ in range(60):
            s=b.call('status')
            if s.get('camera_state')=='running':return s
            if s.get('camera_state')=='error':raise RuntimeError(json.dumps(s))
            time.sleep(.5)
        raise TimeoutError('camera startup')
    def files(sid):
        with b.connection() as c:
            ftp=c.open_sftp();root='/home/uavlab/Jetson_ROS2_V0/perception_records/'+sid
            return {name:ftp.open(root+'/'+name).read() for name in
                    ('manifest.json','calibration.json','session_summary.json','frames.jsonl','events.jsonl')}
    try:
        assert not b.call('status').get('pid'),'preexisting camera'
        call('start_capture',{'profile':'rgb'});owned=wait_ready()['run_id']
        result=call('start_raw_record',{'label':'Codex_finalization_rgb','scene':'bench_background'})
        sid=result['result']['session_id'];time.sleep(2)
        try:call('set_detector',{'enabled':True})
        except ValueError:evidence.append({'check':'rgb_record_model_change_rejected','pass':True})
        else:raise AssertionError('RGB recording allowed model identity change')
        call('stop_raw_record');before=files(sid)
        summary=json.loads(before['session_summary.json']);cal=json.loads(before['calibration.json']);manifest=json.loads(before['manifest.json'])
        assert summary['metadata_complete'] and summary['tail_metadata_may_arrive'] is False
        assert 'saved_calibration' in cal and 'model_id' in cal and 'calibrated' in cal
        assert hashlib.sha256(before['calibration.json']).hexdigest()==manifest['calibration_sha256']
        call('save_calibration',{'target_size_m':.30,'size_definition':'approximately_30cm_pending_field_validation','samples':[]})
        time.sleep(2);assert before==files(sid),'finished session changed after calibration save'
        call('stop_capture');owned=None
        result=call('export_session',{'session_id':sid,'mode':'verify','fps':2})
        assert result['result']['state']=='complete',result
        call('start_capture',{'profile':'perception'});owned=wait_ready()['run_id']
        result=call('start_raw_record',{'label':'Codex_finalization_perception','scene':'bench_background'})
        sid=result['result']['session_id'];time.sleep(4)
        call('set_detector',{'enabled':False});call('set_detector',{'enabled':True});time.sleep(2)
        call('stop_raw_record');before=files(sid)
        assert json.loads(before['session_summary.json'])['metadata_complete']
        call('save_calibration',{'target_size_m':.30,'size_definition':'approximately_30cm_pending_field_validation','samples':[]})
        time.sleep(2);assert before==files(sid),'perception tail metadata still changed'
        call('stop_capture');owned=None
        for mode in ('verify','rgb_video','yolo_video'):
            result=call('export_session',{'session_id':sid,'mode':mode,'fps':2})
            assert result['result']['state']=='complete',result
        print('PASS: recording model guard, calibration snapshot hash, drained immutable metadata, complete stereo playback',flush=True)
    finally:
        if owned:
            if b.call('status').get('run_id')==owned:call('stop_capture')
        out=pathlib.Path(__file__).resolve().parents[1]/'test-results';out.mkdir(exist_ok=True)
        (out/'session_finalization.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
if __name__=='__main__':run()
