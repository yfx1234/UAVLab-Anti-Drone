"""面板「拦截任务 → 击发参数」（2026-10-24）：待命低位/击发高位/高位保持/低位确认的回填、预检、保存流程，
以及解锁/击发中/服务未运行/面板后台旧版/参数文件无效时的状态。范围与默认值以板上 options 为准。"""
import pathlib
import shutil
import subprocess
import unittest

HTML = (pathlib.Path(__file__).resolve().parents[1] / 'panel/index.html').read_text(encoding='utf-8')


class FrontendTriggerParamsTests(unittest.TestCase):
    def run_browser_logic(self, assertions):
        self.assertIn('// Trigger parameter controls', HTML)
        block = HTML.split('// Trigger parameter controls', 1)[1].split('// End trigger parameter controls', 1)[0]
        mounting = HTML.split('// Mounting geometry controls', 1)[1].split('// End mounting geometry controls', 1)[0]
        status = 'function setStatus(' + HTML.split('function setStatus(', 1)[1].split('function syncFireDialog(', 1)[0]
        seconds = 'function trigSec(' + HTML.split('function trigSec(', 1)[1].split('\n', 1)[0]
        if not shutil.which('node'):
            self.skipTest('Node required')
        harness = """
const assert=require('node:assert/strict'),elements={};
const byId=id=>elements[id]||(elements[id]={id,value:'',textContent:'',title:'',disabled:false,open:true,classList:{names:new Set(),toggle(name,on){if(on)this.names.add(name);else this.names.delete(name);},contains(name){return this.names.has(name);}}});
const document={activeElement:null};let busy=false,controlBusy=false,restarted=false;
const copy=value=>JSON.parse(JSON.stringify(value));
const options=(extra={})=>Object.assign({low_us:1000,high_us:2000,hold_s:1,low_s:0.3,source:'default',path:'/home/uavlab/netgun_config/netgun_pwm.json',saved_at:null,error:null,
 defaults:{low_us:1000,high_us:2000,hold_s:1,low_s:0.3},limits:{low_us:[1000,1250],high_us:[1750,2000],hold_s:[0.3,2.5],low_s:[0.2,2]}},extra);
const trigger=(extra={})=>Object.assign({ok:true,ready:true,link:{alive:true,fc:'2/1',armed:false},config:{ok:true},options:options(),idle:{latched:true,low_us:1000},busy:false},extra);
let state={connected:true,error:null,interceptor_error:null,interceptor_age_s:1,features:['trigger_params'],interceptor:{running:false,trigger:trigger()}};
const messages=[],message=(...args)=>messages.push(args),calls=[];
let response=params=>({ok:true,applied:true,service:true,options:options(Object.assign({source:'file',saved_at:'2026-10-24T00:40:00+08:00'},params.config))});
const action=async(name,params,opts)=>{calls.push({name,params:copy(params),options:opts});return typeof response==='function'?response(params):response;};
const values=()=>['trigLowUs','trigHighUs','trigHoldS','trigLowS'].map(id=>byId(id).value);
const pill=()=>[byId('triggerParamState').textContent,['good','bad','warn','idle'].find(n=>byId('triggerParamState').classList.contains(n))||''];
""" + status + seconds + "\n" + mounting + block
        script = harness + "\n(async()=>{\n" + assertions + "\n})().catch(e=>{console.error(e);process.exitCode=1;});"
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_editor_markup_and_update_hook(self):
        body = HTML.split('<details id="triggerDetails"', 1)[1].split('</details>', 1)[0]
        for kept in ('击发参数', 'triggerParamState', 'trigLowUs', 'trigHighUs', 'trigHoldS', 'trigLowS',
                     'triggerParamError', 'saveTrigger', 'revertTrigger', 'defaultTrigger', '>µs<', '>s<'):
            self.assertIn(kept, body)
        self.assertNotIn('class="help"', body, 'explanations live in hover hints, not resident text')
        self.assertIn(' updateMountingControl();\n updateTriggerControl();\n}', HTML)

    def test_defaults_hydrate_with_board_ranges_in_hover_hints(self):
        self.run_browser_logic("""
updateTriggerControl();
assert.deepEqual(values(),['1000','2000','1.0','0.3']);
assert.deepEqual(pill(),['默认值','']);assert.match(byId('triggerParamState').title,/待命 1000 µs → 击发 2000 µs 保持 1\\.0 s，低位确认 0\\.3 s；板上未保存参数文件/);
assert.equal(byId('trigLowUs').title,'网枪模块 ≤1300 µs 判低位：平时与击发后都保持此脉宽；范围 1000–1250 µs，默认 1000 µs');
assert.match(byId('trigHoldS').title,/范围 0\\.3–2\\.5 s，默认 1\\.0 s$/);
assert.equal(byId('triggerParamError').textContent,'');
assert.equal(byId('saveTrigger').disabled,true,'defaults need no file');assert.equal(byId('revertTrigger').disabled,true);assert.equal(byId('defaultTrigger').disabled,true);
assert.equal(byId('trigLowUs').disabled,false);
state.interceptor.trigger.options.limits.hold_s=[0.5,2];updateTriggerControl();assert.match(byId('trigHoldS').title,/范围 0\\.5–2 s/,'board limits win over the fallback');
""")

    def test_save_sends_numbers_and_keeps_draft_until_board_confirms(self):
        self.run_browser_logic("""
updateTriggerControl();
byId('trigHoldS').value='1.5';byId('trigHoldS').oninput();byId('trigLowUs').value=' 1100 ';byId('trigLowUs').oninput();
assert.deepEqual(pill(),['未保存修改','warn']);assert.match(byId('triggerParamState').title,/保存后立即生效/);
assert.equal(byId('saveTrigger').disabled,false);assert.equal(byId('revertTrigger').disabled,false);
await byId('saveTrigger').onclick();
assert.equal(calls.length,1);assert.equal(calls[0].name,'interceptor_trigger_save');
assert.deepEqual(calls[0].params,{config:{low_us:1100,high_us:2000,hold_s:1.5,low_s:0.3}});
assert.equal(messages.at(-1)[0],'击发参数已保存并生效：待命 1100 µs → 击发 2000 µs 保持 1.5 s');
updateTriggerControl();assert.deepEqual(pill(),['已保存，等待刷新','']);assert.equal(byId('trigHoldS').value,'1.5','stale poll must not replace the saved draft');
assert.equal(byId('saveTrigger').disabled,true,'no duplicate save while waiting for the board');
state.interceptor.trigger.options=options({low_us:1100,hold_s:1.5,source:'file',saved_at:'2026-10-24T00:40:00+08:00'});
updateTriggerControl();assert.deepEqual(pill(),['已保存','']);assert.match(byId('triggerParamState').title,/保存于 2026-10-24 00:40:00（板上时间）/);
assert.equal(byId('saveTrigger').disabled,true);assert.equal(byId('defaultTrigger').disabled,false);
state.interceptor.trigger.options.hold_s=2;updateTriggerControl();assert.equal(byId('trigHoldS').value,'2.0','clean form follows the board');
document.activeElement=byId('trigHoldS');state.interceptor.trigger.options.hold_s=2.25;updateTriggerControl();
assert.equal(byId('trigHoldS').value,'2.0','focused field is never overwritten');document.activeElement=null;
byId('defaultTrigger').onclick();assert.deepEqual(values(),['1000','2000','1.0','0.3']);assert.deepEqual(pill(),['未保存修改','warn']);
byId('revertTrigger').onclick();assert.deepEqual(values(),['1100','2000','2.25','0.3']);assert.equal(messages.at(-1)[0],'已恢复为板上已保存的击发参数');
""")

    def test_invalid_values_never_reach_board(self):
        self.run_browser_logic("""
updateTriggerControl();
const cases=[['trigLowUs','',/请填写待命低位/],['trigLowUs','abc',/待命低位需为整数/],['trigLowUs','1251',/待命低位需在 1000–1250 µs 之间/],['trigLowUs','999',/1000–1250/],
 ['trigHighUs','1999.5',/击发高位需为整数/],['trigHighUs','1700',/1750–2000/],['trigHoldS','0.29',/高位保持需在 0\\.3–2\\.5 s 之间/],['trigHoldS','1.234',/最多两位小数/],
 ['trigHoldS','1e1',/最多两位小数/],['trigLowS','-0.3',/最多两位小数/],['trigLowS','Infinity',/最多两位小数/],['trigLowS','2.01',/0\\.2–2 s/]];
for(const [id,value,pattern] of cases){
 const before=byId(id).value;byId(id).value=value;byId(id).oninput();assert.match(byId('triggerParamError').textContent,pattern,id+'='+value);
 await byId('saveTrigger').onclick();assert.match(messages.at(-1)[0],/^击发参数有误：/);byId(id).value=before;byId(id).oninput();
}
assert.equal(calls.length,0);assert.equal(byId('triggerParamError').textContent,'');
byId('trigLowS').value='.25';byId('trigLowS').oninput();await byId('saveTrigger').onclick();assert.deepEqual(calls[0].params.config,{low_us:1000,high_us:2000,hold_s:1,low_s:0.25});
""")

    def test_armed_busy_old_backend_and_stopped_service(self):
        self.run_browser_logic("""
state.interceptor.trigger.link.armed=true;updateTriggerControl();
assert.deepEqual(pill(),['已解锁 · 不可修改','']);assert.equal(byId('trigLowUs').disabled,true);
assert.equal(byId('saveTrigger').disabled,true);assert.equal(byId('saveTrigger').title,'飞控已解锁：上锁后再改击发参数');
state.interceptor.trigger.link.armed=false;state.interceptor.trigger.busy=true;updateTriggerControl();
assert.deepEqual(pill(),['击发中 · 不可修改','']);assert.equal(byId('trigHoldS').disabled,true);
state.interceptor.trigger.busy=false;state.features=undefined;updateTriggerControl();
assert.deepEqual(pill(),['面板后台待重启','warn']);assert.match(byId('triggerParamState').title,/重启面板后才能保存/);
byId('trigHoldS').value='1.2';byId('trigHoldS').oninput();assert.equal(byId('saveTrigger').disabled,true);await byId('saveTrigger').onclick();
assert.equal(calls.length,0);assert.match(messages.at(-1)[0],/重启面板/);
state.features=['trigger_params'];byId('revertTrigger').onclick();
state.interceptor.trigger={ok:false,error:'击发服务未运行',options:options({source:'file',hold_s:1.2,saved_at:'2026-10-24T00:10:00+08:00'})};updateTriggerControl();
assert.deepEqual(pill(),['已保存 · 服务未运行','warn']);assert.match(byId('triggerParamState').title,/击发服务未运行（击发服务未运行），启动后按此参数/);
assert.equal(byId('trigHoldS').value,'1.2');assert.equal(byId('trigHoldS').disabled,false,'the file can still be edited while the service is down');
response={ok:true,service:false,applied:false,note:'击发服务未运行：已写入文件，服务启动后生效',options:options({source:'file',hold_s:1.3})};
byId('trigHoldS').value='1.3';byId('trigHoldS').oninput();assert.match(byId('triggerParamState').title,/服务启动后生效/);
await byId('saveTrigger').onclick();assert.equal(calls.length,1);assert.equal(messages.at(-1)[0],'击发参数已写入板上文件；击发服务未运行，启动后生效');
""")

    def test_missing_invalid_or_disconnected_states(self):
        self.run_browser_logic("""
state.interceptor.trigger.options=options({low_us:null,high_us:null,hold_s:null,low_s:null,source:'file',error:'击发参数文件无效（/home/uavlab/netgun_config/netgun_pwm.json）：缺少击发高位（high_us）'});
updateTriggerControl();assert.deepEqual(pill(),['参数文件无效','bad']);assert.match(byId('triggerParamError').textContent,/缺少击发高位.*击发被拒绝，保存后覆盖/);
assert.deepEqual(values(),['1000','2000','1.0','0.3'],'defaults are offered for repair');assert.equal(byId('saveTrigger').disabled,false);
state.interceptor.trigger.options={fire_value:-1,fire_us:1000,safe_value:1,safe_us:2000,hold_s:1};updateTriggerControl();
assert.deepEqual(pill(),['板上未更新','warn']);assert.match(byId('triggerParamState').title,/install_netgun_pwm_link/);
assert.equal(byId('trigLowUs').disabled,true);assert.equal(byId('saveTrigger').disabled,true);
state.interceptor={running:false};updateTriggerControl();assert.deepEqual(pill(),['板上未更新','warn']);assert.match(byId('triggerParamState').title,/同步部署/);
state.interceptor={running:false,trigger:trigger()};busy=true;updateTriggerControl();assert.equal(byId('saveTrigger').disabled,true);busy=false;
state.connected=false;updateTriggerControl();assert.deepEqual(pill(),['连接后读取','']);assert.equal(byId('saveTrigger').disabled,true);
await byId('saveTrigger').onclick();assert.equal(calls.length,0);
""")


if __name__ == '__main__':
    unittest.main()
