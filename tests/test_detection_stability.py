import pathlib,shutil,subprocess,sys,threading,unittest
from types import SimpleNamespace
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from dashboard import Panel
HTML=(ROOT/'panel/index.html').read_text(encoding='utf-8')

class DetectionStabilityTests(unittest.TestCase):
    def run_node(self,code):
        node=shutil.which('node')
        if not node:self.skipTest('Node required')
        result=subprocess.run([node,'-e',code],capture_output=True,text=True,encoding='utf-8',timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_normal_three_layer_polling_latency_keeps_fresh_detections(self):
        function='function currentTargets('+HTML.split('function currentTargets(',1)[1].split('function draw()',1)[0]
        self.run_node(function+r'''
const assert=require('node:assert/strict');
const state={connected:true,observed_at_browser_s:10,preview:{frame_age_s:.05},state:{pid:10,run_id:'r',boot_id:'b',camera_state:'running',last_status_age_s:1.65,pipeline:{detector_generation:7,targets_valid:true,detector_enabled:true,generic_detector_enabled:false}}};
const packet={run_id:'r',boot_id:'b',observed_at_browser_s:10.8,source_age_s:.1,received_age_s:.1,detector_generation:7,detector_enabled:true,yolo_enabled:true,targets:[{track_id:59,detector_source:'yolo'}]};
assert.equal(currentTargets(packet,state,10.9).length,1,'fresh image must survive normal node/SSH/browser polling skew');
state.error='temporary SSH status failure';
assert.equal(currentTargets(packet,state,10.9).length,1,'a failed control poll must not hide an independently fresh, matching image');
const stale=structuredClone(state);stale.state.last_status_age_s=4.01;
assert.equal(currentTargets(packet,stale,10.9).length,0,'unresponsive node must still clear targets');
packet.observed_at_browser_s=8;assert.equal(currentTargets(packet,state,10.9).length,0,'old browser images remain rejected');
''')

    def test_snapshot_ages_the_cached_board_status_without_mutating_it(self):
        p=Panel.__new__(Panel);p.lock=threading.RLock();p.instance_id='one';p.connected=True;p.error=None
        p.state={'last_status_age_s':.9};p.state_observed_at=10;p.local_revision=0;p.events=[]
        p.receiver=SimpleNamespace(snapshot=lambda:{});p.recorder=SimpleNamespace(snapshot=lambda:{})
        with patch('dashboard.time.monotonic',return_value=11.2):snapshot=p.snapshot()
        self.assertAlmostEqual(snapshot['state']['last_status_age_s'],2.1)
        self.assertEqual(p.state['last_status_age_s'],.9)

    def test_repainting_same_image_does_not_reset_canvas_dimensions(self):
        function='function draw()'+HTML.split('function draw()',1)[1].split('function renderTargets(',1)[0]
        self.run_node(r'''
const assert=require('node:assert/strict');let resets=0,draws=0;
const image={naturalWidth:640,naturalHeight:360},meta={},overlay=false;
const canvas={get width(){return 640;},set width(v){resets++;},get height(){return 360;},set height(v){resets++;},getContext(){return {drawImage(){draws++;}};}};
const byId=()=>canvas;
'''+function+r'''
draw();draw();assert.equal(draws,2);assert.equal(resets,0,'same-size updates must preserve the canvas backing store');
''')

if __name__=='__main__':unittest.main()
