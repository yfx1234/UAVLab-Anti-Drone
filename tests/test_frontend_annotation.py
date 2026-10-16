import pathlib
import shutil
import subprocess
import unittest

HTML = (pathlib.Path(__file__).resolve().parents[1] / 'panel/index.html').read_text(encoding='utf-8')


class FrontendAnnotationTests(unittest.TestCase):
    def test_annotation_uses_selected_local_video_not_left_scene(self):
        handler=HTML.split("byId('annotateSession').onclick=",1)[1].split("byId('cancelDownload')",1)[0]
        self.run_node("""
const assert=require('node:assert/strict');
const nodes={mediaSelect:{value:'/api/media/selected'},annotateModel:{value:'a'.repeat(64)},annotateConfidence:{value:'0.35'}};
const byId=id=>nodes[id],localMedia=[{url:'/api/media/selected',relative:'selected/exports/segment_2/rgb_video.view.mp4',name:'所选视频'}];
const selectedSession=()=>{throw new Error('Must not use left scene')},messages=[],message=(...args)=>messages.push(args),loadMedia=async()=>{};
let params,response=null;async function action(name,value){params=value;return response;}
const handler="""+handler+"""
handler().then(async()=>{assert.equal(params.relative,'selected/exports/segment_2/rgb_video.view.mp4');assert.equal(params.confidence,.35);assert.equal(params.session_id,undefined);response={cleanup_warning:'cleanup failed'};await handler();assert.match(messages.at(-1)[0],/cleanup failed/);assert.equal(messages.at(-1)[1],true);}).catch(e=>{console.error(e);process.exitCode=1;});
""")

    def run_node(self, script):
        if not shutil.which('node'): self.skipTest('Node required')
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_model_family_and_resolution_stay_bound_to_the_selected_weights(self):
        functions = 'let modelCatalog=[];' + HTML.split('let modelCatalog=[];', 1)[1].split('async function loadModels()', 1)[0]
        self.run_node("""
const assert=require('node:assert/strict');
class Select {constructor(){this.options=[];this.chosen='';}replaceChildren(){this.options=[];this.chosen='';}add(o){this.options.push(o);}get value(){return this.options.find(o=>o.value===this.chosen)?.value||this.options[0]?.value||'';}set value(v){this.chosen=v;}}
const elements={},byId=id=>elements[id]||(elements[id]=new Select()),Option=function(text,value){this.text=text;this.value=value;};
for(const id of ['yoloFamily','annotateFamily']){byId(id).add(new Option('自训模型','custom'));byId(id).add(new Option('通用模型','general'));}
const state={state:{}},update=()=>{};
""" + functions + """
modelCatalog=[
 {model_id:'custom1280',name:'exp.engine',manifest:{source_pt_sha256:'trained',input_nchw:[1,3,736,1280]}},
 {model_id:'custom1920',name:'exp1080.engine',manifest:{source_pt_sha256:'trained',input_nchw:[1,3,1088,1920]}},
 {model_id:'glad640',name:'glad.engine',manifest:{source_pt_sha256:'public',input_nchw:[1,3,384,640]}}
];
populateModels('live','custom1920');assert.equal(byId('yoloFamily').value,'custom');assert.equal(byId('yoloInput').value,'custom1920');assert.deepEqual(byId('yoloInput').options.map(o=>o.text),['1280 × 736','1920 × 1088']);
byId('yoloFamily').value='general';populateModels('live','',true);assert.equal(byId('yoloInput').value,'glad640');assert.equal(byId('yoloInput').options.length,1);
populateModels('annotation','custom1280');assert.equal(byId('annotateModel').value,'custom1280');assert.equal(byId('yoloInput').value,'glad640','Offline preview selection must not change the live model');
""")

    def test_progress_changes_units_and_does_not_leave_a_false_100_percent(self):
        functions = 'let annotationStarted=0;' + HTML.split('let annotationStarted=0;', 1)[1].split("byId('yoloInput').onchange=", 1)[0]
        self.run_node("""
const assert=require('node:assert/strict'),elements={},byId=id=>elements[id]||(elements[id]={removeAttribute(name){delete this[name];}}),elapsedTime=v=>String(v),number=(v,n)=>v.toFixed(n);
""" + functions + """
annotationProgress({phase_code:'inference',stage:'逐帧检测',frames:25,total_frames:100,eta_s:9});assert.equal(byId('annotationPercent').textContent,'25%');assert.match(byId('annotationDetail').textContent,/25 .* 100/);
annotationProgress({phase_code:'transfer',stage:'取回标注视频',downloaded_bytes:1048576,total_bytes:2097152});assert.equal(byId('annotationPercent').textContent,'50%');assert.match(byId('annotationDetail').textContent,/MiB/);
annotationProgress({phase_code:'local_encode',stage:'转换为网页播放格式'});assert.equal(byId('annotationPercent').textContent,'');assert.equal(byId('annotationBar').value,undefined);
annotationProgress({phase_code:'complete'});assert.equal(byId('annotationPercent').textContent,'100%');
""")

    def test_action_polls_progress_and_keeps_error_visible(self):
        function = 'async function action(' + HTML.split('async function action(', 1)[1].split('function selectedSession()', 1)[0]
        self.run_node("""
const assert=require('node:assert/strict'),crypto=require('node:crypto'),TOKEN='t',stopActions=[],elements={},byId=id=>elements[id]||(elements[id]={removeAttribute(name){delete this[name];}});
let busy=false,controlBusy=false,restarted=false,mediaPhase='',state={state:{}},annotationStarted=0;
const update=()=>{},message=()=>{},refresh=async()=>{},scheduleMessageDismiss=()=>{},setTimeout=callback=>callback(),progress=[];
const annotationProgress=value=>progress.push(value);
let n=0;const fetch=async(url,options)=>({ok:true,status:200,json:async()=>options?{operation_id:'job'}:++n===1?{state:'running',progress:{phase_code:'inference',frames:2,total_frames:10}}:{state:'error',error:'磁盘已满'}});
""" + function + """
action('annotate_session').then(result=>{assert.equal(result,null);assert.equal(progress[1].frames,2);assert.equal(byId('annotationStage').textContent,'生成失败');assert.match(byId('annotationDetail').textContent,/磁盘已满/);assert.equal(busy,false);}).catch(e=>{console.error(e);process.exitCode=1;});
""")

    def test_live_confidence_is_sent_to_the_detector_and_rejected_when_invalid(self):
        functions = 'function liveConfidence(' + HTML.split('function liveConfidence(', 1)[1].split('async function loadModels()', 1)[0]
        self.run_node("""
const assert=require('node:assert/strict'),elements={yoloConfidence:{value:'0.35'}},byId=id=>elements[id],state={state:{pid:123,pipeline:{detector_enabled:true}}},selectedModel=()=>'chosen-engine',update=()=>{},message=()=>{};
const calls=[];const action=async(name,params)=>{calls.push({name,params});return {ok:true};};
""" + functions + """
(async()=>{await applyLiveModel();assert.deepEqual(calls[0],{name:'set_detector',params:{enabled:true,model_id:'chosen-engine',confidence:.35}});byId('yoloConfidence').value='0.1';await applyLiveModel();assert.equal(calls.length,1);})().catch(e=>{console.error(e);process.exitCode=1;});
""")


if __name__ == '__main__': unittest.main()
