"""Run the real session refresh function across a directory-migration boundary."""
import pathlib
import shutil
import subprocess
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]

class FrontendMediaRefreshTests(unittest.TestCase):
 def test_session_refresh_reloads_migrated_media_and_finishes_loading(self):
  node=shutil.which('node')
  if not node:self.skipTest('Node is required for frontend behavior check')
  html=(ROOT/'panel/index.html').read_text(encoding='utf-8')
  function=html.split('function sessionNotes(',1)[1].split("byId('refreshSessions')",1)[0]
  script='''
const assert=require('node:assert/strict');
let sessionsLoading=false,busy=false,state={connected:true},sessions=[],mediaReloads=0;
let serverPath='old-folder/depth.mp4',playerPath=serverPath;
const select={value:'session_old',replaceChildren(){},add(){}};
const byId=()=>select,selectedSession=()=>select.value,update=()=>{},showSession=()=>{},hydrateNotes=()=>{},number=()=>'',sessionStates={};
function Option(name,value){this.name=name;this.value=value;}
async function action(){serverPath='renamed-folder/depth.mp4';return [{session_id:'session_old',display_name:'renamed',summary:{}}];}
async function loadMedia(){mediaReloads++;await Promise.resolve();playerPath=serverPath;}
'''+'function sessionNotes('+function+'''
loadSessions().then(()=>{
 assert.equal(playerPath,'renamed-folder/depth.mp4','player must reload after session metadata migrates its folder');
 assert.equal(mediaReloads,1);assert.equal(select.value,'session_old');assert.equal(sessionsLoading,false);
}).catch(error=>{console.error(error);process.exitCode=1;});
'''
  result=subprocess.run([node,'-e',script],text=True,capture_output=True,timeout=10)
  self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
