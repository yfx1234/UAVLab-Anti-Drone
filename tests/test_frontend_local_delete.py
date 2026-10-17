import pathlib
import shutil
import subprocess
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]


class FrontendLocalDeleteTests(unittest.TestCase):
    def test_confirmation_keeps_original_target_releases_player_and_refreshes(self):
        node=shutil.which('node')
        if not node:self.skipTest('Node required')
        html=(ROOT/'panel/index.html').read_text(encoding='utf-8')
        self.assertTrue('let localDeleteTarget=' in html,'Local video delete flow is missing')
        flow='let localDeleteTarget='+html.split('let localDeleteTarget=',1)[1].split('const healthDetails=',1)[0]
        script=r'''
const assert=require('node:assert/strict');
let busy=false,controlBusy=false,restarted=false,reloads=0,outcome={},calls=[];
let localMedia=[{url:'/api/media/a',relative:'scene/exports/segment_1/a.view.mp4',name:'Video A',path:'C:/Video A.mp4'},
 {url:'/api/media/b',relative:'scene/exports/segment_1/b.view.mp4',name:'Video B',path:'C:/Video B.mp4'}];
const nodes={};const byId=id=>nodes[id]??=( {value:'',textContent:'',focus(){},addEventListener(event,fn){this[event]=fn;},showModal(){this.open=true;},close(){this.open=false;this.closeEvent?.();}} );
nodes.localVideo={src:'a',pause(){this.paused=true;},removeAttribute(){this.src='';},load(){this.released=true;}};
nodes.deleteMediaDialog={showModal(){this.open=true;},close(){this.open=false;this.onclose?.();},addEventListener(e,fn){this['on'+e]=fn;}};
const message=()=>{};
async function action(name,params){assert.equal(name,'delete_local_media');assert.equal(nodes.localVideo.src,'');assert.equal(nodes.localVideo.paused,true);assert.equal(nodes.localVideo.released,true);calls.push(params);return outcome;}
async function loadMedia(){reloads++;}
''' + flow + r'''
(async()=>{
 byId('mediaSelect').value='/api/media/a';byId('deleteLocalMedia').onclick();
 assert.match(byId('deleteMediaName').textContent,/Video A/);
 byId('cancelMediaDelete').onclick();assert.equal(calls.length,0);
 byId('deleteLocalMedia').onclick();byId('mediaSelect').value='/api/media/b';
 await byId('confirmMediaDelete').onclick();
 assert.deepEqual(calls[0],{relative:'scene/exports/segment_1/a.view.mp4',confirmation:'scene/exports/segment_1/a.view.mp4'});
 assert.equal(reloads,1);assert.equal(byId('deleteMediaDialog').open,false);
 outcome=null;byId('deleteLocalMedia').onclick();await byId('confirmMediaDelete').onclick();assert.equal(reloads,2);
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
        result=subprocess.run([node,'-e',script],text=True,capture_output=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)


if __name__=='__main__':unittest.main()
