"""面板「拦截任务 → 安装参数」：草稿保护、保存流程、生效状态；面板不再常驻说明文字与枪轴核对。"""
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
const document={activeElement:null};let busy=false,controlBusy=false,restarted=false;
const defaults={camera_top_m:[0,.06,0],camera_right_eye:'forward',rtk_antenna_m:[0,.06,.122],flight_controller_m:[0,0,.18]};
const copy=value=>JSON.parse(JSON.stringify(value));
let state={connected:true,error:null,interceptor_age_s:1,interceptor:{running:false,geometry:{saved:{ok:true,source:'default',saved_at:null,config:copy(defaults)}}}};
const messages=[],message=(...args)=>messages.push(args),calls=[];
let response=params=>({ok:true,source:'file',config:copy(params.config)});
const action=async(name,params,options)=>{calls.push({name,params:copy(params),options});return typeof response==='function'?response(params):response;};
""" + status + block
        script = harness + "\n(async()=>{\n" + assertions + "\n})().catch(e=>{console.error(e);process.exitCode=1;});"
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_defaults_hydrate_and_show_rtk_offset(self):
        self.run_browser_logic("""
updateMountingControl();
assert.deepEqual(['mountCameraX','mountCameraY','mountCameraZ','mountRtkY','mountRtkZ','mountFcX','mountFcZ'].map(id=>byId(id).value),['0','6','0','6','12.2','0','18']);
assert.equal(byId('mountRightEye').value,'forward');
assert.equal(byId('mountingState').textContent,'默认值（未保存）');assert.ok(byId('mountingState').classList.contains('warn'));
assert.match(byId('mountingState').title,/默认值/,'details stay in the hover hint');assert.equal(byId('mountingError').textContent,'');
assert.deepEqual(['mountGpsX','mountGpsY','mountGpsZ'].map(id=>byId(id).textContent),['0.000 m','0.060 m','-0.058 m']);
assert.equal(byId('saveMounting').disabled,false,'defaults can be persisted explicitly');
assert.equal(byId('revertMounting').disabled,true);
""")

    def test_panel_has_no_resident_explanations_or_gun_axis_check(self):
        body = HTML.split('<details id="mountingDetails"', 1)[1].split('</details>', 1)[0]
        for gone in ('class="help"', 'mountingNote', '枪轴', 'mountTilt', 'fillMountTilt', 'mountTarget', '拦截链换算', 'QGC'):
            self.assertNotIn(gone, body)
        for kept in ('相机顶面', 'RTK 天线', '飞控', 'mountRightEye', 'saveMounting', 'EKF2_GPS_POS_Z'):
            self.assertIn(kept, body)
        self.assertNotIn('updateMountReadout', HTML)

    def test_save_sends_metres_and_keeps_draft_until_board_confirms(self):
        self.run_browser_logic("""
updateMountingControl();
byId('mountCameraZ').value='-1.5';byId('mountCameraZ').oninput();byId('mountFcZ').value='17.25';byId('mountFcZ').oninput();
assert.equal(byId('mountingState').textContent,'未保存修改');assert.equal(byId('mountGpsZ').textContent,'-0.051 m');
await byId('saveMounting').onclick();
assert.equal(calls.length,1);assert.equal(calls[0].name,'interceptor_geometry_save');
assert.deepEqual(calls[0].params,{config:{camera_top_m:[0,.06,-.015],camera_right_eye:'forward',rtk_antenna_m:[0,.06,.122],flight_controller_m:[0,0,.1725]}});
assert.match(messages.at(-1)[0],/下次启动拦截链生效/);
updateMountingControl();assert.equal(byId('mountCameraZ').value,'-1.5','stale poll must not replace the saved draft');
assert.equal(byId('saveMounting').disabled,true,'no duplicate save while waiting for the board');
state.interceptor.geometry.saved={ok:true,source:'file',saved_at:'2026-10-22T10:00:00+08:00',config:copy(calls[0].params.config)};
updateMountingControl();assert.equal(byId('mountingState').textContent,'已保存');assert.match(byId('mountingState').title,/2026-10-22 10:00:00/);
assert.equal(byId('saveMounting').disabled,true);
state.interceptor.geometry.saved.config.flight_controller_m=[0,0,.2];updateMountingControl();assert.equal(byId('mountFcZ').value,'20','clean form follows the board');
document.activeElement=byId('mountFcZ');state.interceptor.geometry.saved.config.flight_controller_m=[0,0,.21];updateMountingControl();
assert.equal(byId('mountFcZ').value,'20','focused field is never overwritten');
""")

    def test_invalid_values_never_reach_board(self):
        self.run_browser_logic("""
updateMountingControl();
for(const value of ['', 'abc', '50.01', '-50.5', '1.234', '1e1', 'Infinity']){
 byId('mountRtkZ').value=value;byId('mountRtkZ').oninput();assert.notEqual(byId('mountingError').textContent,'',value);
 assert.equal(byId('mountGpsZ').textContent,'—');await byId('saveMounting').onclick();
}
byId('mountRtkZ').value='12.2';byId('mountRtkZ').oninput();assert.equal(byId('mountingError').textContent,'');
byId('mountRightEye').value='up';byId('mountRightEye').onchange();await byId('saveMounting').onclick();
assert.equal(calls.length,0);assert.match(messages.at(-1)[0],/右目朝向/);
byId('mountRightEye').value='right';byId('mountFcY').value='−2';byId('mountFcY').oninput();
await byId('saveMounting').onclick();assert.deepEqual(calls[0].params.config.flight_controller_m,[0,-.02,.18]);assert.equal(calls[0].params.config.camera_right_eye,'right');
""")

    def test_running_chain_reports_whether_saved_values_are_applied(self):
        self.run_browser_logic("""
state.interceptor={running:true,geometry:{saved:{ok:true,source:'file',saved_at:'2026-10-22T10:00:00+08:00',config:copy(defaults)},applied:{config:copy(defaults)}}};
updateMountingControl();assert.equal(byId('mountingState').textContent,'运行中 · 已生效');assert.ok(byId('mountingState').classList.contains('good'));
state.interceptor.geometry.applied.config.flight_controller_m=[0,0,0];updateMountingControl();
assert.equal(byId('mountingState').textContent,'重启后生效');assert.match(byId('mountingState').title,/重启拦截链后生效/);
delete state.interceptor.geometry.applied;updateMountingControl();assert.equal(byId('mountingState').textContent,'运行中参数未知');
state.interceptor_age_s=30;updateMountingControl();assert.equal(byId('mountingState').textContent,'已保存','stale status is not treated as running');
""")

    def test_missing_invalid_or_disconnected_states_block_saving(self):
        self.run_browser_logic("""
state.interceptor={running:false,geometry:{saved:{ok:false,error:'安装参数文件无效（/home/uavlab/netgun_config/mounting_geometry.json）：内容不是对象'}}};
updateMountingControl();assert.equal(byId('mountingState').textContent,'参数文件无效');assert.ok(byId('mountingState').classList.contains('bad'));
assert.match(byId('mountingError').textContent,/拒绝启动/);assert.equal(byId('mountCameraY').value,'6','defaults are offered for repair');
assert.equal(byId('saveMounting').disabled,false);
state.interceptor={running:false};updateMountingControl();assert.equal(byId('mountingState').textContent,'板上未更新');
assert.equal(byId('saveMounting').disabled,true);assert.equal(byId('mountCameraY').disabled,true);
state.interceptor={running:false,geometry:{saved:{ok:true,source:'file',config:copy(defaults)}}};busy=true;updateMountingControl();
assert.equal(byId('saveMounting').disabled,true);busy=false;
state.connected=false;updateMountingControl();assert.equal(byId('mountingState').textContent,'连接后读取');
assert.equal(byId('saveMounting').disabled,true);assert.equal(byId('mountCameraY').disabled,true);
await byId('saveMounting').onclick();assert.equal(calls.length,0);
""")


if __name__ == '__main__':
    unittest.main()
