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
    def test_generic_is_retired_from_operator_panel(self):
        self.assertNotIn('set_generic_detector',BOARD_ACTIONS)
        for control in ('genericStatus','genericSensitivity','enableGeneric','disableGeneric'):
            self.assertNotIn('id="'+control+'"',HTML)

    def test_generic_cannot_receive_idle_rgb_download_acceleration(self):
        good={'pid':123,'camera_state':'running','profile':'rgb','last_status_age_s':.2,
              'pipeline':{'profile':'rgb','depth_enabled':False,'detector_enabled':False,
               'module_sku':'P3767-0003','platform_configuration_consistent':True,
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
const assert=require('node:assert/strict');let now=10;const performance={now:()=>now*1000};
const s={observed_at_browser_s:10,connected:true,preview:{frame_age_s:.1},state:{pid:10,run_id:'r',boot_id:'b',camera_state:'running',last_status_age_s:.2,pipeline:{detector_generation:3,detector_enabled:true,generic_detector_enabled:false,targets_valid:true}}};
const packet={observed_at_browser_s:10,run_id:'r',boot_id:'b',detector_generation:3,detector_enabled:true,yolo_enabled:true,generic_detector_enabled:false,received_age_s:.1,source_age_s:.1,targets:[{track_id:1,detector_source:'yolo'}]};
assert.equal(currentTargets(packet,s).length,1,'fresh YOLO observations must appear');
const retired=structuredClone(packet);retired.targets[0].detector_source='generic';assert.equal(currentTargets(retired,s).length,0);
for(const mutate of [p=>p.detector_generation=2,p=>p.run_id='old',p=>p.boot_id='old',p=>p.received_age_s=2,p=>p.source_age_s=2,p=>p.detector_enabled=false,p=>p.yolo_enabled=false]){
 const p=structuredClone(packet);mutate(p);assert.equal(currentTargets(p,s).length,0);
}
for(const mutate of [x=>x.connected=false,x=>x.state.camera_state='stopped',x=>x.state.last_status_age_s=5,x=>x.state.pipeline.detector_enabled=false,x=>x.state.pipeline.targets_valid=false,x=>x.preview.frame_age_s=2]){
 const x=structuredClone(s);mutate(x);assert.equal(currentTargets(packet,x).length,0);
}
// The PC can keep receiving new board frames while this browser's JPEG/meta fetches fail.
now=12;const liveState=structuredClone(s);liveState.observed_at_browser_s=now;
assert.equal(currentTargets(packet,liveState).length,0,'old browser frame must age even if PC receiver is fresh');
const newPacket=structuredClone(packet);newPacket.observed_at_browser_s=now;
now=15;newPacket.observed_at_browser_s=now;
assert.equal(currentTargets(newPacket,s).length,0,'old status must age even if browser frames arrive');
now=70;assert.equal(currentTargets(packet,s).length,0,'all local requests failing must expire cached data');
'''],capture_output=True,text=True,encoding='utf-8',timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_retired_generic_detector_cannot_be_enabled_through_board_rpc(self):
        sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
        from perception_supervisor import validate_action
        with self.assertRaisesRegex(ValueError,'停用'):validate_action('set_generic_detector',{'enabled':True})
        validate_action('set_generic_detector',{'enabled':False})

    def test_expiry_tick_clears_overlays_without_waiting_for_network(self):
        self.assertTrue('function expireTargets(' in HTML,'independent expiry tick missing')
        function='function expireTargets('+HTML.split('function expireTargets(',1)[1].split('function draw()',1)[0]
        node=shutil.which('node')
        if not node:self.skipTest('Node required')
        result=subprocess.run([node,'-e',r'''
const assert=require('node:assert/strict');let expired=false,rendered=null,draws=0;
const meta={},state={},currentTargets=()=>expired?[]:[{track_id:1}],renderTargets=t=>rendered=t,draw=()=>draws++;
'''+function+r'''
expireTargets();assert.equal(rendered,null);assert.equal(draws,0);
expired=true;expireTargets();assert.deepEqual(rendered,[]);assert.equal(draws,1);
'''],capture_output=True,text=True,encoding='utf-8',timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue('setInterval(expireTargets,250)' in HTML,'expiry must run independently of fetch completion')

    def test_stalled_browser_image_ages_delay_and_exits_live_view(self):
        function='function expireTargets('+HTML.split('function expireTargets(',1)[1].split('function draw()',1)[0]
        function='function previewFrameAge('+HTML.split('function previewFrameAge(',1)[1].split('function currentTargets(',1)[0]+function
        node=shutil.which('node')
        if not node:self.skipTest('Node required')
        result=subprocess.run([node,'-e',r'''
const assert=require('node:assert/strict');const performance={now:()=>60000};
const meta={observed_at_browser_s:0,received_age_s:.1,source_age_s:.1},state={state:{pid:1}};
const elements={viewer:{live:true,classList:{remove(){elements.viewer.live=false;}}},frameAge:{textContent:'0.1'},emptyTitle:{textContent:''},emptyHelp:{textContent:''},fullscreenPreview:{disabled:false}};
const byId=id=>elements[id],number=(n,d)=>n.toFixed(d),currentTargets=()=>[],renderTargets=()=>{},draw=()=>{};
'''+function+r'''
expireTargets();assert.equal(elements.viewer.live,false,'a 60-second-old image cannot retain the live badge');
assert.equal(Number(elements.frameAge.textContent),60.2,'displayed delay must grow with the retained browser image');
assert.equal(elements.fullscreenPreview.disabled,true);
state.state.pid=null;elements.frameAge.textContent='—';elements.emptyTitle.textContent='相机尚未启动';
expireTargets();assert.equal(elements.frameAge.textContent,'—','stopped camera keeps its empty-state metrics');
assert.equal(elements.emptyTitle.textContent,'相机尚未启动');
'''],capture_output=True,text=True,encoding='utf-8',timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
