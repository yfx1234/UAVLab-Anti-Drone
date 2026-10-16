import pathlib
import shutil
import subprocess
import unittest

HTML=(pathlib.Path(__file__).resolve().parents[1]/'panel/index.html').read_text(encoding='utf-8')


class FrontendRangeSizeTests(unittest.TestCase):
    def run_browser_logic(self, assertions):
        self.assertTrue('// Size estimate controls' in HTML,'Size estimate controls are missing')
        script=HTML.split('// Size estimate controls',1)[1].split('// End size estimate controls',1)[0]
        if not shutil.which('node'):self.skipTest('Node required')
        harness="""
const assert=require('node:assert/strict'),elements={};
const byId=id=>elements[id]||(elements[id]={value:'30',textContent:'',disabled:false});
const document={activeElement:null};let busy=false,controlBusy=false,restarted=false;
let state={connected:true,error:null,state:{pid:123,camera_state:'running',pipeline:{record_state:'idle',target_size_m:.3,size_estimate_calibrated:false}}};
const messages=[],message=(...args)=>messages.push(args),calls=[];
let response={ok:true};const action=async(name,params,options)=>{calls.push({name,params,options});return response;};
"""
        result=subprocess.run(['node','-e',harness+script+"\n(async()=>{\n"+assertions+"\n})().catch(e=>{console.error(e);process.exitCode=1;});"],capture_output=True,text=True,encoding='utf-8',timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_presets_convert_centimetres_and_wait_for_confirmed_state(self):
        self.run_browser_logic("""
updateRangeSizeControl();assert.equal(byId('rangeSizeCm').value,'30');
byId('rangeSize247').onclick();assert.equal(byId('rangeSizeCm').value,'24.7');assert.equal(calls.length,0);
await byId('saveRangeSize').onclick();
assert.equal(calls[0].name,'save_calibration');assert.equal(calls[0].params.target_size_m,.247);
assert.deepEqual(calls[0].params.samples,[]);assert.match(calls[0].params.size_definition,/manual/);
updateRangeSizeControl();assert.equal(byId('rangeSizeCm').value,'24.7','old poll must not replace the saved value');
state.state.pipeline.target_size_m=.247;updateRangeSizeControl();assert.match(byId('rangeSizeStatus').textContent,/24.7/);
byId('rangeSize30').onclick();await byId('saveRangeSize').onclick();assert.equal(calls[1].params.target_size_m,.3);
assert.match(byId('rangeSizeStatus').textContent,/未标定/);
""")

    def test_poll_does_not_overwrite_draft_or_focused_input(self):
        self.run_browser_logic("""
updateRangeSizeControl();byId('rangeSizeCm').value='26.5';byId('rangeSizeCm').oninput();
state.state.pipeline.target_size_m=.247;updateRangeSizeControl();assert.equal(byId('rangeSizeCm').value,'26.5');
document.activeElement=byId('rangeSizeCm');updateRangeSizeControl();assert.equal(byId('rangeSizeCm').value,'26.5');
response=null;await byId('saveRangeSize').onclick();updateRangeSizeControl();assert.equal(byId('rangeSizeCm').value,'26.5');
assert.match(byId('rangeSizeStatus').textContent,/未保存/);
""")

    def test_invalid_values_never_reach_board(self):
        self.run_browser_logic("""
for(const value of ['', '0', '-1', '500.1', 'Infinity', 'abc']){
 byId('rangeSizeCm').value=value;byId('rangeSizeCm').oninput();await byId('saveRangeSize').onclick();
}
assert.equal(calls.length,0);assert.match(messages.at(-1)[0],/1.*500/);
""")

    def test_recording_disconnected_and_stopped_camera_block_save(self):
        self.run_browser_logic("""
for(const phase of ['starting','recording','finalizing']){
 state.state.pipeline.record_state=phase;updateRangeSizeControl();assert.equal(byId('saveRangeSize').disabled,true);
 await byId('saveRangeSize').onclick();
}
state.state.pipeline.record_state='idle';state.connected=false;updateRangeSizeControl();await byId('saveRangeSize').onclick();
assert.equal(byId('rangeSizeCm').disabled,true);state.connected=true;state.state.pid=null;state.state.camera_state='stopped';
updateRangeSizeControl();await byId('saveRangeSize').onclick();assert.equal(calls.length,0);
assert.match(byId('rangeSizeStatus').textContent,/启动相机/);
""")


if __name__=='__main__':unittest.main()
