"""Execute the shipped browser functions for workspace and high-frequency updates."""
import pathlib
import re
import shutil
import subprocess
import unittest
from html.parser import HTMLParser

ROOT=pathlib.Path(__file__).resolve().parents[1]
HTML=(ROOT/'panel/index.html').read_text(encoding='utf-8')

class IdParser(HTMLParser):
 def __init__(self):super().__init__();self.ids=[]
 def handle_starttag(self,tag,attrs):
  for name,value in attrs:
   if name=='id':self.ids.append(value)

class FrontendWorkspaceTests(unittest.TestCase):
 def run_node(self,script):
  node=shutil.which('node')
  if not node:self.skipTest('Node is required for frontend behavior checks')
  result=subprocess.run([node,'-e',script],text=True,capture_output=True,timeout=10)
  self.assertEqual(result.returncode,0,result.stderr)

 def test_html_references_and_removed_extraction_entry(self):
  parser=IdParser();parser.feed(HTML)
  self.assertEqual(len(parser.ids),len(set(parser.ids)))
  self.assertFalse(set(re.findall(r"byId\('([^']+)'\)",HTML))-set(parser.ids))
  self.assertNotIn('extract_local_frames',HTML)
  self.assertNotIn('extractFrames',HTML)
  self.assertNotIn('extractFps',HTML)
  self.assertIn('.target-viewport{height:264px;overflow:auto;scrollbar-gutter:stable;',HTML)
  self.assertIn('position:sticky;top:0;z-index:1;',HTML)

 def test_target_updates_reuse_rows_and_preserve_internal_scroll(self):
  function='function renderTargets('+HTML.split('function renderTargets(',1)[1].split('async function frames()',1)[0]
  self.run_node(r'''
const assert=require('node:assert/strict');
class Node {
 constructor(){this.children=[];this.cells=[];this.dataset={};this.parentNode=null;this.textContent='';this.replacements=0;}
 get firstElementChild(){return this.children[0]||null;}
 insertCell(){const cell={textContent:'',colSpan:1};this.cells.push(cell);return cell;}
 remove(){if(this.parentNode){const i=this.parentNode.children.indexOf(this);this.parentNode.children.splice(i,1);this.parentNode=null;}}
 insertBefore(row,before){row.remove();const index=before?this.children.indexOf(before):this.children.length;this.children.splice(index,0,row);row.parentNode=this;}
 replaceChildren(...rows){this.replacements++;for(const row of [...this.children])row.remove();for(const row of rows)this.insertBefore(row,null);}
}
const tbody=new Node(),viewport={scrollTop:106,scrollLeft:84},count=new Node();
const initialEmpty=new Node();initialEmpty.insertCell();tbody.insertBefore(initialEmpty,null);
const document={createElement:()=>new Node()},byId=id=>({targets:tbody,targetsViewport:viewport,targetCount:count})[id];
const targetRows=new Map();let emptyTargetRow=initialEmpty;
const number=(v,d=2)=>typeof v==='number'?v.toFixed(d):'—';
const modeNames={depth:'有效深度'},reasonNames={valid:'中心深度一致'};
const target=id=>({track_id:id,confidence:.9,mode:'depth',horizontal_angle_rad:0,vertical_angle_rad:0,position_camera:[1,2,3],depth_z_m:3,distance_m:4,depth_valid_ratio:.7,reason:'valid'});
'''+function+r'''
renderTargets([target(1)]);assert.equal(tbody.children.length,1,'initial empty row must leave immediately');
const original=tbody.children[0];renderTargets([target(1)]);assert.equal(tbody.children[0],original,'same target must keep DOM identity');
assert.equal(tbody.replacements,0,'live rows must not be replaced every frame');
for(const size of [8,60,1,0,8]){
 renderTargets(Array.from({length:size},(_,n)=>target(n+1)));
 assert.equal(count.textContent,String(size));assert.equal(tbody.children.length,size||1);
 assert.equal(viewport.scrollTop,106);assert.equal(viewport.scrollLeft,84);
}
renderTargets([target(3),target(3)]);assert.equal(tbody.children.length,2,'duplicate track IDs must not overwrite rows');
''')

 def test_workspace_switch_has_no_capture_or_recording_action(self):
  function='function setWorkspace('+HTML.split('function setWorkspace(',1)[1].split('for(const tab of',1)[0]
  self.run_node(r'''
const assert=require('node:assert/strict');let workspace='live',busy=false,sessions=[{session_id:'saved'}],state={connected:true};let requests=0,paused=0;
const make=()=>({hidden:false,tabIndex:0,attrs:{},setAttribute(k,v){this.attrs[k]=v;},focus(){this.focused=true;}});
const elements={tabLive:make(),tabLibrary:make(),workspaceLive:make(),workspaceLibrary:make(),localVideo:{pause(){paused++;}}};
const byId=id=>elements[id],loadSessions=()=>{requests++;};
const action=()=>{throw Error('workspace switch must not send hardware actions');};
'''+function+r'''
setWorkspace('library');assert.equal(elements.workspaceLive.hidden,true);assert.equal(elements.workspaceLibrary.hidden,false);
assert.equal(elements.tabLibrary.attrs['aria-selected'],'true');assert.equal(elements.tabLive.tabIndex,-1);
setWorkspace('live',true);assert.equal(elements.workspaceLive.hidden,false);assert.equal(elements.tabLive.focused,true);assert.equal(paused,1);
assert.equal(requests,0,'already loaded scenes should not trigger RPCs');
''')

 def test_completed_action_dismisses_success_but_keeps_errors_and_progress(self):
  feedback=HTML.split("const reasonNames=",1)[1].split("byId('dismissMessage').onclick",1)[0]
  feedback=feedback[feedback.index('function '):]
  action='async function action('+HTML.split('async function action(',1)[1].split('function selectedSession()',1)[0]
  self.run_node(r'''
const assert=require('node:assert/strict'),crypto=require('node:crypto');
let busy=false,controlBusy=false,restarted=false,messageTimer=null,messageError=false,mediaPhase='',downloadRequest=null;
const TOKEN='test',stopActions=[],state={state:{}},elements={},timers=new Map();let nextTimer=0;
const byId=id=>elements[id]||(elements[id]={textContent:'',hidden:true,value:'',classList:{toggle(){}}});
const setTimeout=(fn,ms)=>{if(ms===400){fn();return 0;}const id=++nextTimer;timers.set(id,{fn,ms});return id;},clearTimeout=id=>timers.delete(id);
const update=()=>{},refresh=async()=>{},restartNotice=()=>{},transferProgress=()=>{};
let requests=0;const fetch=async()=>({ok:true,status:200,json:async()=>++requests===1?{operation_id:'one'}:{state:'complete',result:{ok:true}}});
'''+feedback+action+r'''
(async()=>{
 busy=true;message('正在执行');assert.equal(timers.size,0,'progress stays while work runs');busy=false;
 message('发生错误',true);assert.equal(timers.size,0,'errors remain until dismissed');
 await action('set_detector',{}, {label:'启用 YOLO'});
 assert.equal(busy,false);assert.equal(byId('message').textContent,'已完成：启用 YOLO');assert.equal(byId('feedback').hidden,false);
 assert.equal(timers.size,1,'success must schedule dismissal after action clears busy');
 const timer=[...timers.values()][0];assert.equal(timer.ms,6000);timer.fn();assert.equal(byId('feedback').hidden,true);
 message('连接故障',true);assert.equal(timers.size,0);
 restarted=true;message('刷新恢复');assert.equal(timers.size,0,'restart recovery remains visible');
})().catch(error=>{console.error(error);process.exitCode=1;});
''')

 def test_download_stages_report_limits_and_retained_conversion_compatibility(self):
  function='function transferProgress('+HTML.split('function transferProgress(',1)[1].split('const beijingFormatter=',1)[0]
  self.run_node(r'''
const assert=require('node:assert/strict');let mediaPhase='';
const elements={};const byId=id=>elements[id]||(elements[id]={value:0,textContent:'',removeAttribute(){this.value=0;}});
const number=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';
'''+function+r'''
transferProgress({phase_code:'transfer',stage:'传输',downloaded_bytes:1048576,total_bytes:2097152,bytes_s:262144,limit_bytes_s:262144,eta_s:4});
assert.equal(mediaPhase,'transfer');assert.equal(byId('progressPercent').textContent,'50%');assert.equal(byId('progressStage').textContent,'传输视频');
assert.match(byId('progressRate').textContent,/0.25 MiB\/s/);assert.match(byId('progressRate').textContent,/限速 256 KiB\/s/);assert.match(byId('progressRate').textContent,/约剩 4 秒/);
transferProgress({phase_code:'remote_hash',downloaded_bytes:2097152,total_bytes:2097152});assert.equal(byId('progressStage').textContent,'板上校验');assert.equal(byId('progressRate').textContent,'');
transferProgress({phase:'正在板上生成RGB视频'});assert.equal(mediaPhase,'conversion');assert.equal(byId('progressStage').textContent,'板上生成视频');
transferProgress({phase:'下载完成，正在电脑准备播放版本'});assert.equal(mediaPhase,'local');assert.equal(byId('progressStage').textContent,'本机准备播放');
transferProgress({phase_code:'ready_check',stage:'检查视频'});assert.equal(mediaPhase,'checking');assert.equal(byId('progressStage').textContent,'检查视频');
transferProgress({phase_code:'board_convert',stage:'板上转换',cancel_after_stage:true});assert.equal(mediaPhase,'conversion');assert.equal(byId('progressStage').textContent,'板上生成视频');
transferProgress({phase_code:'local_prepare',stage:'本机准备'});assert.equal(mediaPhase,'local');assert.equal(byId('progressStage').textContent,'本机准备播放');
transferProgress({phase_code:'transfer',downloaded_bytes:1048576,total_bytes:2097152,bytes_s:786432,limit_bytes_s:786432,eta_s:1});assert.match(byId('progressRate').textContent,/限速 768 KiB\/s/);
''')

 def test_board_conversion_disables_camera_start_and_defers_cancellation(self):
  transfer='function transferProgress('+HTML.split('function transferProgress(',1)[1].split('const beijingFormatter=',1)[0]
  update='function update()'+HTML.split('function update()',1)[1].split('async function action(',1)[0]
  cancel="byId('cancelDownload').onclick="+HTML.split("byId('cancelDownload').onclick=",1)[1].split('function mediaIdentity(',1)[0]
  self.run_node(r'''
const assert=require('node:assert/strict'),crypto=require('node:crypto');
let busy=true,controlBusy=false,restarted=false,mediaPhase='',sessions=[],sessionsLoading=false,renameTarget='',downloadRequest='active-download',knownRecordedSession='';
const TOKEN='test',stopButtons=['stopCapture','stopRaw','stopLocal'],state={connected:true,state:{camera_state:'stopped',pipeline:{}}};
const cameraNames={},recordNames={},elements={};
const byId=id=>elements[id]||(elements[id]={id,dataset:{},value:'',textContent:'',classList:{toggle(){},add(){},remove(){}},removeAttribute(){}});
const document={querySelectorAll:()=>['startCapture','stopCapture','stopRaw','stopLocal'].map(byId)};
const number=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—',beijingTime=()=>'',selectedSession=()=>'',renderTargets=()=>{};
let notice='';const message=text=>{notice=text;},fetch=async()=>({ok:true,status:200}),restartNotice=()=>{};
'''+transfer+update+cancel+r'''
(async()=>{
 transferProgress({phase_code:'ready_check'});update();assert.equal(byId('startCapture').disabled,true,'camera cannot start while video readiness is being checked');
 transferProgress({phase_code:'board_convert',cancel_after_stage:true});update();assert.equal(byId('startCapture').disabled,true,'camera cannot start during offline board conversion');
 await byId('cancelDownload').onclick();assert.match(notice,/转换结束后生效/,'board conversion cancellation must describe deferred application');
 transferProgress({phase_code:'local_prepare'});update();assert.equal(byId('startCapture').disabled,false,'local preparation does not own the camera');
 transferProgress({phase_code:'transfer'});update();assert.equal(byId('startCapture').disabled,false,'file transfer permits camera start');
})().catch(error=>{console.error(error);process.exitCode=1;});
''')

if __name__=='__main__':unittest.main()
