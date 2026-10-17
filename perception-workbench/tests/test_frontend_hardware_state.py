import pathlib,shutil,subprocess,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]

class FrontendHardwareStateTests(unittest.TestCase):
 def test_unavailable_stale_and_disconnected_sensors_never_show_healthy_numbers(self):
  node=shutil.which('node')
  if not node:self.skipTest('Node required')
  html=(ROOT/'panel/index.html').read_text(encoding='utf8')
  block='const hardware='+html.split('const hardware=',1)[1].split('const activeSession=',1)[0]
  script=r'''
const assert=require('node:assert/strict'),elements={};const byId=id=>elements[id]||(elements[id]={textContent:'',classList:{toggle(){}}});
const number=(value,d=2)=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(d):'—';
const state={connected:true,error:null,control_roundtrip_ms:150};let s={hardware:{sample_age_s:0,temperatures_c:{max:55,cpu:54,gpu:55},module_input_voltage_v:5,module_power_w:10,mem_available_kib:null,disk_free_bytes:null,network:{},radio_signal_dbm:null}};
function render(){
'''+block+r'''}
render();assert.equal(byId('boardTemperature').textContent,'55.0 °C');assert.equal(byId('memory').textContent,'—');assert.equal(byId('disk').textContent,'—');assert.equal(byId('radioSignal').textContent,'未接入');
assert.equal(byId('cpuUsage').textContent,'—');assert.equal(byId('gpuUsage').textContent,'—');
s.hardware.cpu_utilization_pct=25;s.hardware.gpu_utilization_pct=0;render();assert.equal(byId('cpuUsage').textContent,'25 %');assert.equal(byId('gpuUsage').textContent,'0 %');
s.hardware.gpu_utilization_pct=150;render();assert.equal(byId('gpuUsage').textContent,'—');
s.hardware.sample_age_s=6.1;render();assert.equal(byId('boardVoltage').textContent,'—');assert.equal(byId('controlRoundtrip').textContent,'—');
assert.equal(byId('cpuUsage').textContent,'—');assert.equal(byId('gpuUsage').textContent,'—');
s.hardware.sample_age_s=0;state.error='timeout';render();assert.equal(byId('boardTemperature').textContent,'—');assert.equal(byId('linkHealth').textContent,'连接异常');
state.error=null;state.connected=false;render();assert.equal(byId('boardPower').textContent,'—');assert.equal(byId('linkHealth').textContent,'未连接');
'''
  p=subprocess.run([node,'-e',script],capture_output=True,text=True,encoding='utf8',timeout=10)
  self.assertEqual(p.returncode,0,p.stderr)

if __name__=='__main__':unittest.main()
