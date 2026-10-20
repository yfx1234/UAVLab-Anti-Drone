"""Daily workflow: configure while paused, save size, then enable YOLO."""
import pathlib
import shutil
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
HTML = (ROOT / 'panel/index.html').read_text(encoding='utf-8')


def handler(element):
    marker = "byId('" + element + "').onclick="
    return HTML.split(marker, 1)[1].split('\n', 1)[0].removesuffix(';')


class ParameterWorkflowTests(unittest.TestCase):
    def test_paused_configuration_is_used_when_yolo_is_enabled(self):
        if not shutil.which('node'):
            self.skipTest('Node required')
        model = 'function liveConfidence(' + HTML.split('function liveConfidence(', 1)[1].split('async function loadModels()', 1)[0]
        size = HTML.split('// Size estimate controls', 1)[1].split('// End size estimate controls', 1)[0]
        script = """
const assert=require('node:assert/strict');
const elements={yoloConfidence:{value:'0.35'},rangeSizeCm:{value:'24.7'},captureResolution:{value:'HD1080'},captureFps:{value:'15'}};
const byId=id=>elements[id]||(elements[id]={value:'',disabled:false});
const document={activeElement:null};let busy=false,controlBusy=false,restarted=false;
let state={connected:true,error:null,state:{pid:42,camera_state:'running',model_id:'old',pipeline:{record_state:'idle',detector_enabled:false,target_size_m:.30,yolo_confidence:.55}}};
let chosen='custom1280',views=[];const selectedModel=()=>chosen,setView=v=>views.push(v),update=()=>{},populateModels=()=>{},message=()=>{};
const calls=[];const action=async(name,params)=>{calls.push({name,params});return {ok:true};};
""" + model + size + "\nconst enable=" + handler('enableDetector') + ";\n(async()=>{\n" + """
await applyLiveModel();
assert.deepEqual(calls.at(-1),{name:'set_detector',params:{enabled:false,model_id:'custom1280',confidence:.35}});
editRangeSize();assert.equal(calls.length,1,'Editing size must not submit');
await saveRangeSize();assert.deepEqual(calls.at(-1).params,{target_size_m:.247,size_definition:'manual_projection_extent_unvalidated',samples:[]});
state.state.pipeline.target_size_m=.247;updateRangeSizeControl();
await enable();assert.deepEqual(calls.at(-1),{name:'set_detector',params:{enabled:true,confidence:.35,model_id:'custom1280'}});
assert.deepEqual(views,[true]);
// A second workflow chooses a different model, input engine, confidence and size.
state.state.pipeline.detector_enabled=false;chosen='general640';byId('yoloConfidence').value='0.85';
byId('rangeSizeCm').value='30';editRangeSize();await applyLiveModel();await saveRangeSize();await enable();
assert.deepEqual(calls.slice(-3).map(c=>c.name),['set_detector','save_calibration','set_detector']);
assert.deepEqual(calls.at(-3).params,{enabled:false,model_id:'general640',confidence:.85});
assert.equal(calls.at(-2).params.target_size_m,.30);
assert.deepEqual(calls.at(-1).params,{enabled:true,model_id:'general640',confidence:.85});
assert.equal(calls[0].params.model_id,'custom1280','Historical request must remain unchanged');
""" + "\n})().catch(e=>{console.error(e);process.exitCode=1;});"
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_camera_start_uses_selected_engine_and_rgb_profile(self):
        if not shutil.which('node'):
            self.skipTest('Node required')
        script = """
const assert=require('node:assert/strict');
const byId=id=>({value:id==='captureResolution'?'HD1080':'15'}),selectedModel=()=>'general1280';
let sent;const action=(name,params)=>sent={name,params};
const start=""" + handler('startCapture') + ";start();" + """
assert.deepEqual(sent,{name:'start_capture',params:{profile:'rgb',camera_resolution:'HD1080',capture_fps:15,model_id:'general1280'}});
"""
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
