"""Opt-in functional/parallel acceptance against 8767 and the current Nano."""
import json,pathlib,re,time,urllib.request,uuid,sys
URL='http://127.0.0.1:8767'
def get(path):
    with urllib.request.urlopen(URL+path,timeout=10) as r:return json.load(r)
def run(seconds=300):
    page=urllib.request.urlopen(URL,timeout=10).read().decode()
    token=re.search(r"const TOKEN='([^']+)'",page).group(1)
    evidence={'duration_requested_s':seconds,'actions':[],'samples':[],'checks':{}}
    owned=None;local=False;session=None
    def call(action,params=None,expect_error=False):
        s=get('/api/status');b=s.get('state',{})
        data={'action':action,'params':params or {},'request_id':uuid.uuid4().hex,
              'expected_boot_id':b.get('boot_id'),'expected_run_id':b.get('run_id'),
              'expected_revision':b.get('revision'),'expected_local_revision':s.get('local_revision')}
        request=urllib.request.Request(URL+'/api/action',data=json.dumps(data).encode(),
            headers={'Content-Type':'application/json','Origin':URL,'X-Panel-Token':token})
        with urllib.request.urlopen(request,timeout=10) as r:jobid=json.load(r)['operation_id']
        end=time.monotonic()+3700
        while time.monotonic()<end:
            job=get('/api/jobs/'+jobid)
            if job['state'] in ('complete','error'):
                evidence['actions'].append({'action':action,'job':job})
                if expect_error:
                    if job['state']!='error':raise AssertionError(action+' incorrectly accepted')
                    return job
                if job['state']=='error':raise RuntimeError(action+': '+job['error'])
                return job['result']
            time.sleep(.25)
        raise TimeoutError(action)
    def await_camera():
        for _ in range(80):
            b=get('/api/status')['state']
            if b.get('camera_state')=='error':raise AssertionError(json.dumps(b))
            if b.get('camera_state')=='running' and b.get('pipeline',{}).get('detector_state')==2:return b
            time.sleep(.5)
        raise AssertionError('camera/detector did not become ready')
    try:
        call('connect')
        if get('/api/status')['state'].get('pid'):raise RuntimeError('preexisting camera run, refusing to stop it')
        call('start_capture',{'profile':'perception'});owned=await_camera()['run_id']
        baseline_end=time.monotonic()+15
        while time.monotonic()<baseline_end:
            evidence['samples'].append({'phase':'baseline','status':get('/api/status')});time.sleep(1)
        call('start_preview_record',{'view_mode':'yolo'});local=True
        result=call('start_raw_record',{'label':'Codex_parallel_acceptance','scene':'bench_background','lighting':'room',
             'size_definition':'approximately_30cm_not_field_calibrated'})
        session=result['result']['session_id'];evidence['session_id']=session
        model=get('/api/status')['state']['model_id']
        call('select_model',{'model_id':model},expect_error=True)
        call('export_session',{'session_id':session,'mode':'verify','fps':2},expect_error=True)
        end=time.monotonic()+seconds;disconnected=False;paused=False
        while time.monotonic()<end:
            elapsed=seconds-(end-time.monotonic());s=get('/api/status');p=s['state'].get('pipeline',{})
            evidence['samples'].append({'phase':'parallel','elapsed_s':elapsed,'status':s})
            if p.get('record_state')!='recording':raise AssertionError('raw recording unexpectedly stopped')
            if elapsed>35 and not disconnected:
                call('disconnect');time.sleep(5);call('connect');disconnected=True
                evidence['checks']['receiver_disconnect_survives']=True
            if elapsed>65 and not paused:
                call('set_detector',{'enabled':False});time.sleep(2)
                call('set_detector',{'enabled':True});await_camera();paused=True
                evidence['checks']['detector_pause_preserves_recording']=True
            print('PARALLEL',round(elapsed),round(p.get('capture_hz',0),1),round(p.get('inference_hz',0),1),p.get('record_state'),flush=True)
            time.sleep(5)
        call('stop_raw_record');call('stop_preview_record');local=False
        evidence['local_recording']=get('/api/status')['preview_recording']
        call('stop_capture');owned=None
        call('export_session',{'session_id':session,'mode':'verify','fps':2})
        for mode in ('rgb_video','yolo_video'):
            call('export_session',{'session_id':session,'mode':mode,'fps':.2})
        # Ten lifecycle transitions are split between depth and RGB profiles.
        for i in range(10):
            profile='rgb' if i%2==0 else 'perception'
            call('start_capture',{'profile':profile})
            for _ in range(60):
                b=get('/api/status')['state']
                if b.get('camera_state')=='running':break
                if b.get('camera_state')=='error':raise RuntimeError('toggle failure '+json.dumps(b))
                time.sleep(.5)
            else:raise TimeoutError('toggle start')
            owned=b['run_id'];call('stop_capture');owned=None
            print('LIFECYCLE_OK',i+1,flush=True)
        evidence['checks']['ten_lifecycle_transitions']=True
        call('prepare_dataset');evidence['final_status']=get('/api/status')
        evidence['result']='pass'
        return evidence
    except Exception as e:
        evidence['result']='fail';evidence['error']=str(e);raise
    finally:
        if local:
            try:call('stop_preview_record')
            except Exception as e:evidence['cleanup_local_error']=str(e)
        if owned:
            try:
                if get('/api/status')['state'].get('run_id')==owned:call('stop_capture')
            except Exception as e:evidence['cleanup_board_error']=str(e)
        out=pathlib.Path(__file__).resolve().parents[1]/'test-results';out.mkdir(exist_ok=True)
        (out/'live_panel.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
if __name__=='__main__':run(float(sys.argv[1]) if len(sys.argv)>1 else 300)
