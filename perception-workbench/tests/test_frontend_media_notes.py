import pathlib,shutil,subprocess,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]

class FrontendMediaNotesTests(unittest.TestCase):
 def test_video_details_show_notes_as_text_and_clear_them_when_selection_is_empty(self):
  node=shutil.which('node')
  if not node:self.skipTest('Node required')
  html=(ROOT/'panel/index.html').read_text(encoding='utf8')
  function='function renderMediaDetails('+html.split('function renderMediaDetails(',1)[1].split('function showMedia(',1)[0]
  script=r'''
const assert=require('node:assert/strict'),elements={};
const byId=id=>elements[id]||(elements[id]={textContent:'',value:'video',hidden:false});
const video=byId('localVideo');video.readyState=1;video.videoWidth=1920;video.videoHeight=1080;video.duration=90;video.getAttribute=()=> 'video';
const localMedia=[{url:'video',display_name:'场次',metadata:{label:'数据集 <img onerror=bad>',scene:'天空'}}];
const beijingTime=()=>'',elapsedTime=()=>'01:30';
const sessionNotes=item=>[item?.metadata?.label,item?.metadata?.scene,item?.metadata?.lighting].filter(Boolean).join(' · ');
'''+function+r'''
renderMediaDetails();assert.equal(byId('mediaNotes').hidden,false);assert.equal(byId('mediaNotes').textContent,'备注：数据集 <img onerror=bad> · 天空');
byId('mediaSelect').value='';renderMediaDetails();assert.equal(byId('mediaNotes').hidden,true);assert.equal(byId('mediaNotes').textContent,'');
'''
  result=subprocess.run([node,'-e',script],capture_output=True,text=True,encoding='utf8',timeout=10)
  self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
