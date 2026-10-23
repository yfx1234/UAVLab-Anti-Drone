"""面板「拦截任务 → 安装参数」：草稿保护、保存流程、生效状态与枪轴读数。"""
import pathlib
import shutil
import subprocess
import unittest

HTML = (pathlib.Path(__file__).resolve().parents[1] / 'panel/index.html').read_text(encoding='utf-8')


class FrontendMountingTests(unittest.TestCase):
    def run_browser_logic(self, assertions):
        self.assertIn('// Mounting geometry controls', HTML)
        block = HTML.split('// Mounting geometry controls', 1)[1].split('// End mounting geometry controls', 1)[0]
        status = 'function setStatus(' + HTML.split('function setStatus(', 1)[1].split('function syncFireDialog(', 1)[0]
        if not shutil.which('node'):
            self.skipTest('Node required')
        harness = """
const assert=require('node:assert/strict'),elements={};
const byId=id=>elements[id]||(elements[id]={id,value:'',textContent:'',title:'',disabled:false,open:true,classList:{names:new Set(),toggle(name,on){if(on)this.names.add(name);else this.names.delete(name);},contains(name){return this.names.has(name);}}});
const document={activeElement:null};let busy=false,controlBusy=false,restarted=false,meta={},targets=[];
const defaults={camera_center_m:[0,.06,0],camera_right_eye:'forward',rtk_antenna_m:[0,.06,.085],flight_controller_m:[0,0,0],gun_axis_tilt_deg:[0,0]};
const copy=value=>JSON.parse(JSON.stringify(value));
let state={connected:true,error:null,interceptor_age_s:1,state:{camera_state:'running'},interceptor:{running:false,geometry:{saved:{ok:true,source:'default',saved_at:null,config:copy(defaults)}}}};
const number=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—',currentTargets=()=>targets;
const messages=[],message=(...args)=>messages.push(args),calls=[];
let response=params=>({ok:true,source:'file',config:copy(params.config)});
const action=async(name,params,options)=>{calls.push({name,params:copy(params),options});return typeof response==='function'?response(params):response;};
""" + status + block
        script = harness + "\n(async()=>{\n" + assertions + "\n})().catch(e=>{console.error(e);process.exitCode=1;});"
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_defaults_hydrate_and_show_derived_values(self):
        self.run_browser_logic("""
updateMountingControl();
assert.deepEqual(['mountCameraX','mountCameraY','mountCameraZ','mountRtkY','mountRtkZ','mountFcX'].map(id=>byId(id).value),['0','6','0','6','8.5','0']);
assert.equal(byId('mountRightEye').value,'forward');assert.equal(byId('mountTiltForward').value,'0');
assert.equal(byId('mountingState').textContent,'默认值（未保存）');assert.ok(byId('mountingState').classList.contains('warn'));
assert.deepEqual(['mountLeftX','mountLeftY','mountLeftZ'].map(id=>byId(id).textContent),['-2.5','6','0']);
assert.deepEqual(['mountGpsX','mountGpsY','mountGpsZ'].map(id=>byId(id).textContent),['0.000 m','0.060 m','0.085 m']);
assert.equal(byId('saveMounting').disabled,false,'defaults can be persisted explicitly');
assert.equal(byId('revertMounting').disabled,true);
""")

    def test_save_sends_metres_and_keeps_draft_until_board_confirms(self):
        self.run_browser_logic("""
updateMountingControl();
byId('mountCameraZ').value='-1.5';byId('mountCameraZ').oninput();byId('mountTiltForward').value='1.25';byId('mountTiltForward').oninput();
assert.equal(byId('mountingState').textContent,'未保存修改');assert.equal(byId('mountMuzzleX').textContent,'0');assert.equal(byId('mountLeftZ').textContent,'-1.5');
await byId('saveMounting').onclick();
assert.equal(calls.length,1);assert.equal(calls[0].name,'interceptor_geometry_save');
assert.deepEqual(calls[0].params,{config:{camera_center_m:[0,.06,-.015],camera_right_eye:'forward',rtk_antenna_m:[0,.06,.085],flight_controller_m:[0,0,0],gun_axis_tilt_deg:[1.25,0]}});
assert.match(messages.at(-1)[0],/下次启动拦截链生效/);
updateMountingControl();assert.equal(byId('mountCameraZ').value,'-1.5','stale poll must not replace the saved draft');
assert.equal(byId('saveMounting').disabled,true,'no duplicate save while waiting for the board');
state.interceptor.geometry.saved={ok:true,source:'file',saved_at:'2026-10-22T10:00:00+08:00',config:copy(calls[0].params.config)};
updateMountingControl();assert.equal(byId('mountingState').textContent,'已保存');assert.match(byId('mountingNote').textContent,/2026-10-22 10:00:00/);
assert.equal(byId('saveMounting').disabled,true);
state.interceptor.geometry.saved.config.gun_axis_tilt_deg=[2,0];updateMountingControl();assert.equal(byId('mountTiltForward').value,'2','clean form follows the board');
document.activeElement=byId('mountTiltForward');state.interceptor.geometry.saved.config.gun_axis_tilt_deg=[3,0];updateMountingControl();
assert.equal(byId('mountTiltForward').value,'2','focused field is never overwritten');
""")

    def test_invalid_values_never_reach_board(self):
        self.run_browser_logic("""
updateMountingControl();
for(const value of ['', 'abc', '50.01', '-50.5', '1.234', '1e1', 'Infinity']){
 byId('mountRtkZ').value=value;byId('mountRtkZ').oninput();assert.notEqual(byId('mountingError').textContent,'',value);
 assert.equal(byId('mountGpsZ').textContent,'—');await byId('saveMounting').onclick();
}
byId('mountRtkZ').value='8.5';byId('mountRtkZ').oninput();assert.equal(byId('mountingError').textContent,'');
for(const value of ['10.01','-11','0.123']){byId('mountTiltRight').value=value;byId('mountTiltRight').oninput();await byId('saveMounting').onclick();}
byId('mountRightEye').value='up';byId('mountRightEye').onchange();await byId('saveMounting').onclick();
assert.equal(calls.length,0);assert.match(messages.at(-1)[0],/右目朝向/);
byId('mountRightEye').value='right';byId('mountTiltRight').value='−2';byId('mountTiltRight').oninput();
await byId('saveMounting').onclick();assert.deepEqual(calls[0].params.config.gun_axis_tilt_deg,[0,-2]);assert.equal(calls[0].params.config.camera_right_eye,'right');
""")

    def test_running_chain_reports_whether_saved_values_are_applied(self):
        self.run_browser_logic("""
state.interceptor={running:true,geometry:{saved:{ok:true,source:'file',saved_at:'2026-10-22T10:00:00+08:00',config:copy(defaults)},applied:{config:copy(defaults)}}};
updateMountingControl();assert.equal(byId('mountingState').textContent,'运行中 · 已生效');assert.ok(byId('mountingState').classList.contains('good'));
state.interceptor.geometry.applied.config.gun_axis_tilt_deg=[1,0];updateMountingControl();
assert.equal(byId('mountingState').textContent,'重启后生效');assert.match(byId('mountingNote').textContent,/重启拦截链后生效/);
delete state.interceptor.geometry.applied;updateMountingControl();assert.equal(byId('mountingState').textContent,'运行中参数未知');
state.interceptor_age_s=30;updateMountingControl();assert.equal(byId('mountingState').textContent,'已保存','stale status is not treated as running');
""")

    def test_missing_invalid_or_disconnected_states_block_saving(self):
        self.run_browser_logic("""
state.interceptor={running:false,geometry:{saved:{ok:false,error:'安装参数文件无效（/home/uavlab/netgun_config/mounting_geometry.json）：内容不是对象'}}};
updateMountingControl();assert.equal(byId('mountingState').textContent,'参数文件无效');assert.ok(byId('mountingState').classList.contains('bad'));
assert.match(byId('mountingNote').textContent,/拒绝启动/);assert.equal(byId('mountCameraY').value,'6','defaults are offered for repair');
assert.equal(byId('saveMounting').disabled,false);
state.interceptor={running:false};updateMountingControl();assert.equal(byId('mountingState').textContent,'板上未更新');
assert.equal(byId('saveMounting').disabled,true);assert.equal(byId('mountCameraY').disabled,true);
state.interceptor={running:false,geometry:{saved:{ok:true,source:'file',config:copy(defaults)}}};busy=true;updateMountingControl();
assert.equal(byId('saveMounting').disabled,true);busy=false;
state.connected=false;updateMountingControl();assert.equal(byId('mountingState').textContent,'连接后读取');
assert.equal(byId('saveMounting').disabled,true);assert.equal(byId('mountCameraY').disabled,true);
await byId('saveMounting').onclick();assert.equal(calls.length,0);
""")

    def test_readout_uses_only_depth_targets_and_fills_gun_axis_tilt(self):
        self.run_browser_logic("""
updateMountingControl();
const tilt=2*Math.PI/180,range=6,body=[range*Math.tan(tilt),0,-range],left=[-.025,.06,0];
const optical=[body[0]-left[0],-(body[1]-left[1]),-(body[2]-left[2])];
targets=[{track_id:7,confidence:.95,mode:'size_estimate',position_camera:[0,0,5]},{track_id:3,confidence:.6,mode:'depth',position_camera:optical}];
updateMountingControl();
assert.equal(byId('mountTargetX').textContent,'0.21');assert.equal(byId('mountTargetY').textContent,'0.00');assert.equal(byId('mountTargetZ').textContent,'6.00');
assert.equal(byId('mountTargetForward').textContent,'2.00');assert.equal(byId('mountTargetRight').textContent,'0.00');assert.equal(byId('mountTargetError').textContent,'2.00');
assert.match(byId('mountReadoutNote').textContent,/Y3/);assert.equal(byId('fillMountTilt').disabled,false);
byId('fillMountTilt').onclick();assert.equal(byId('mountTiltForward').value,'2');assert.equal(byId('mountTiltRight').value,'0');
assert.equal(byId('mountingState').textContent,'未保存修改');assert.equal(byId('mountTargetError').textContent,'0.00');assert.equal(calls.length,0,'filling never saves');
byId('revertMounting').onclick();assert.equal(byId('mountTiltForward').value,'0');assert.equal(byId('mountingState').textContent,'默认值（未保存）');
targets=[{track_id:3,confidence:.6,mode:'depth',position_camera:[Math.tan(15*Math.PI/180)*6+.025,.06,6]}];updateMountingControl();
assert.equal(byId('fillMountTilt').disabled,true);assert.match(byId('mountReadoutNote').textContent,/超出/);
targets=[{track_id:3,confidence:.6,mode:'bearing_only',position_camera:null}];updateMountingControl();
assert.match(byId('mountReadoutNote').textContent,/暂无有效深度目标/);assert.equal(byId('mountTargetForward').textContent,'—');
state.state.camera_state='stopped';updateMountingControl();assert.match(byId('mountReadoutNote').textContent,/启动相机/);
""")

    def test_right_eye_orientation_changes_body_readout(self):
        self.run_browser_logic("""
updateMountingControl();
targets=[{track_id:1,confidence:.9,mode:'depth',position_camera:[1.005,0,6]}];
const readout=eye=>{byId('mountRightEye').value=eye;byId('mountRightEye').onchange();return ['mountTargetX','mountTargetY'].map(id=>byId(id).textContent);};
assert.deepEqual(readout('forward'),['0.98','0.06']);
assert.deepEqual(readout('backward'),['-0.98','0.06']);
assert.deepEqual(readout('right'),['0.00','1.04']);
assert.deepEqual(readout('left'),['0.00','-0.92']);
""")


if __name__ == '__main__':
    unittest.main()
