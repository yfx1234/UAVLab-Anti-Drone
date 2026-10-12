import copy
import pathlib
import shutil
import subprocess
import sys
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from dashboard import BOARD_ACTIONS
from session_manager import SessionManager
HTML=(ROOT/'panel/index.html').read_text(encoding='utf-8')

class GenericPanelTests(unittest.TestCase):
    def test_generic_is_an_allowed_panel_action(self):
        self.assertIn('set_generic_detector',BOARD_ACTIONS)

    def test_generic_cannot_receive_idle_rgb_download_acceleration(self):
        good={'pid':123,'camera_state':'running','profile':'rgb','last_status_age_s':.2,
              'pipeline':{'profile':'rgb','depth_enabled':False,'detector_enabled':False,
               'generic_detector_enabled':False,'record_state':'idle','capture_hz':15,
               'mem_available_kib':4*1024*1024,'max_temperature_c':56,'last_frame_age_s':.1}}
        self.assertEqual(SessionManager.transfer_limit(good),768*1024)
        for value in (True,None,'unknown'):
            busy=copy.deepcopy(good);busy['pipeline']['generic_detector_enabled']=value
            with self.subTest(value=value):self.assertEqual(SessionManager.transfer_limit(busy),256*1024)

    def test_old_generation_wrong_owner_disabled_and_stale_targets_are_hidden(self):
        self.assertTrue('function currentTargets(' in HTML,'freshness function missing')
        function='function currentTargets('+HTML.split('function currentTargets(',1)[1].split('function draw()',1)[0]
        node=shutil.which('node')
        if not node:self.skipTest('Node required')
        result=subprocess.run([node,'-e',function+r'''
const assert=require('node:assert/strict');
const s={connected:true,preview:{frame_age_s:.1},state:{pid:10,run_id:'r',boot_id:'b',camera_state:'running',last_status_age_s:.2,pipeline:{detector_generation:3,detector_enabled:false,generic_detector_enabled:true,targets_valid:true}}};
const packet={run_id:'r',boot_id:'b',detector_generation:3,detector_enabled:true,yolo_enabled:false,generic_detector_enabled:true,received_age_s:.1,source_age_s:.1,targets:[{track_id:2147483649,detector_source:'generic'}]};
assert.equal(currentTargets(packet,s).length,1,'generic-only observations must appear');
for(const mutate of [p=>p.detector_generation=2,p=>p.run_id='old',p=>p.boot_id='old',p=>p.received_age_s=2,p=>p.source_age_s=2,p=>p.detector_enabled=false,p=>p.generic_detector_enabled=false]){
 const p=structuredClone(packet);mutate(p);assert.equal(currentTargets(p,s).length,0);
}
for(const mutate of [x=>x.connected=false,x=>x.state.camera_state='stopped',x=>x.state.last_status_age_s=5,x=>x.state.pipeline.generic_detector_enabled=false,x=>x.state.pipeline.targets_valid=false,x=>x.preview.frame_age_s=2]){
 const x=structuredClone(s);mutate(x);assert.equal(currentTargets(packet,x).length,0);
}
'''],capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_generic_controls_do_not_start_yolo_or_restart_camera(self):
        self.assertTrue("byId('enableGeneric').onclick=" in HTML,'generic controls missing')
        function=HTML.split("byId('enableGeneric').onclick=",1)[1].split("byId('disableGeneric').onclick=",1)[0]
        self.assertIn("'set_generic_detector'",function)
        self.assertNotIn("'set_detector'",function)
        self.assertNotIn("'start_capture'",function)

if __name__=='__main__':unittest.main()
