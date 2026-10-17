"""Exercise shipped autosave with edits arriving while a save is in flight."""
import pathlib,shutil,subprocess,unittest
HTML=(pathlib.Path(__file__).resolve().parents[1]/'panel/index.html').read_text(encoding='utf-8')
class FrontendOperatorNotesTests(unittest.TestCase):
 def node(self,script):
  if not shutil.which('node'):self.skipTest('Node required')
  p=subprocess.run(['node','-e',script],capture_output=True,text=True,timeout=10)
  self.assertEqual(p.returncode,0,p.stderr)
 def test_model_options_are_bound_to_current_nx_and_original_shapes(self):
  label='function modelInputLabel('+HTML.split('function modelInputLabel(',1)[1].split('async function loadModels()',1)[0]
  self.node("const assert=require('node:assert/strict');"+label+"""
const model=shape=>({manifest:{input_nchw:shape,build_environment:{module_sku:'P3767-0000'}}});
assert.match(modelInputLabel(model([1,3,736,1280])),/原输入/);
assert.match(modelInputLabel(model([1,3,1088,1920])),/1080p/);
assert.equal(modelInputLabel({manifest:{input_nchw:[1,3,736,1280],build_environment:{module_sku:'P3767-0003'}}}),'');
assert.equal(modelInputLabel(model([1,3,640,640])),'');
""")
 def test_latest_edit_is_saved_after_inflight_update_without_changing_scene_id(self):
  functions='async function writeNotes('+HTML.split('async function writeNotes(',1)[1].split("for(const id of ['sessionLabel'",1)[0]
  self.node("""
const assert=require('node:assert/strict'),crypto=require('node:crypto');
let notePending={sid:'session_a',values:{label:'first'}},noteTimer=null,noteSaving=null,restarted=false;
let state={connected:true,state:{boot_id:'b',run_id:'r',revision:1}},sessions=[{session_id:'session_a'}];
const TOKEN='t',elements={},byId=id=>elements[id]||(elements[id]={}),renderSessionOptions=()=>{},showSession=()=>{},selectedSession=()=>'';
const refresh=async()=>{};let submitted=[],jobs={},first=true;
const fetch=async(url,options)=>{
 if(options){const body=JSON.parse(options.body);submitted.push(body.params);jobs[body.request_id]=body.params;
  if(first){first=false;notePending={sid:'session_a',values:{label:'latest'}};state.state.run_id='another_run';}
  return {ok:true,status:200,json:async()=>({operation_id:body.request_id})};}
 const request=jobs[url.split('/').pop()];return {ok:true,json:async()=>({state:'complete',result:{result:{metadata:{label:request.label}}}})};
};
"""+functions+"""
flushNotes().then(()=>{
 assert.deepEqual(submitted.map(x=>x.label),['first','latest']);
 assert(submitted.every(x=>x.session_id==='session_a'),'edits must remain bound to their original session');
 assert.equal(sessions[0].metadata.label,'latest');assert.equal(notePending,null);assert.equal(byId('noteStatus').textContent,'已保存到场次');
}).catch(e=>{console.error(e);process.exitCode=1;});
""")
if __name__=='__main__':unittest.main()
