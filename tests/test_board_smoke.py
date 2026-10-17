"""Explicit opt-in real board acceptance, never started by ordinary unit discovery."""
import json
import pathlib
import sys
import time
import uuid
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'panel'))
from board_client import BoardClient
from frame_receiver import FrameReceiver

def run(profile='rgb',record_seconds=5):
    board=BoardClient();receiver=FrameReceiver();evidence=[];active=False
    def call(action,params=None):
        state=board.call('status')
        request={'request_id':uuid.uuid4().hex,'expected_boot_id':state['boot_id'],
                 'expected_run_id':state['run_id'],'expected_revision':state['revision']}
        result=board.call(action,params,request);evidence.append({'action':action,'result':result});return result
    try:
        old=board.call('status')
        if old.get('pid'):raise RuntimeError('preexisting board capture; smoke test will not stop someone else')
        call('start_capture',{'profile':profile});active=True
        receiver.set_enabled(True)
        for _ in range(40):
            state=board.call('status')
            if state.get('camera_state')=='error':raise RuntimeError(json.dumps(state))
            if receiver.cache.latest() and state.get('camera_state')=='running':break
            time.sleep(.5)
        else:raise RuntimeError('no fresh shared preview')
        frame=receiver.cache.latest();evidence.append({'preview_metadata':frame.metadata})
        print('PREVIEW_OK',frame.metadata['frame_key'],flush=True)
        call('start_raw_record',{'label':'Codex_acceptance_'+profile,'scene':'bench','lighting':'current room','size_definition':'approx_30cm_pending_field_definition'})
        time.sleep(record_seconds)
        receiver.set_enabled(False);time.sleep(2)
        state=board.call('status')
        if state['pipeline']['record_state']!='recording':raise RuntimeError('recording stopped with receiver disconnect')
        call('stop_raw_record');call('stop_capture');active=False
        sessions=board.call('list_sessions');sid=next(s['session_id'] for s in sessions if s['manifest'].get('metadata',{}).get('label')=='Codex_acceptance_'+profile)
        call('export_session',{'session_id':sid,'fps':2,'mode':'verify'})
        print('RECORD_PLAYBACK_OK',sid,flush=True)
        return evidence
    finally:
        receiver.close()
        if active:
            try:call('stop_capture')
            except Exception as e:print('CLEANUP_ERROR',str(e),flush=True)

if __name__=='__main__':
    result=run(sys.argv[1] if len(sys.argv)>1 else 'rgb',float(sys.argv[2]) if len(sys.argv)>2 else 5)
    out=pathlib.Path(__file__).resolve().parents[1]/'test-results';out.mkdir(exist_ok=True)
    (out/('board_'+(sys.argv[1] if len(sys.argv)>1 else 'rgb')+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
