import hashlib,json,pathlib,sys,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from media_library import MediaLibrary

class MediaNotesTests(unittest.TestCase):
 def scene(self,root):
  folder=root/'session_notes';video=folder/'exports/session/rgb_video.view.mp4'
  video.parent.mkdir(parents=True);video.write_bytes(b'video-must-stay-identical')
  (folder/'manifest.json').write_text(json.dumps({'created_at_unix_s':1800000000,'created_time_source':'pc_reference_monotonic','metadata':{'scene':'天空','label':'原始备注'}}),encoding='utf8')
  (folder/'notes.json').write_text(json.dumps({'label':'数据集','lighting':'阴天'}),encoding='utf8')
  return folder,video

 def test_existing_download_exposes_notes_offline_without_rewriting_video(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);folder,video=self.scene(root);before=hashlib.sha256(video.read_bytes()).hexdigest()
   row=MediaLibrary(root).list()[0]
   self.assertEqual(row['metadata'],{'label':'数据集','scene':'天空','lighting':'阴天'})
   self.assertIn('数据集',row['name']);self.assertEqual(hashlib.sha256(pathlib.Path(row['path']).read_bytes()).hexdigest(),before)

 def test_refreshed_board_notes_persist_and_explicit_clear_beats_old_download(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);self.scene(root);library=MediaLibrary(root)
   source={'session_id':'session_notes','display_name':'场次','created_at_unix_s':1800000000,'metadata':{'label':'新备注','scene':'天空'}}
   library.scene_names([source]);self.assertEqual(library.list()[0]['metadata']['label'],'新备注')
   source['metadata']['label']='外部修改';self.assertEqual(library.list()[0]['metadata']['label'],'新备注')
   from scene_registry import SceneRegistry
   loaded=MediaLibrary(root,registry=SceneRegistry(root));self.assertEqual(loaded.list()[0]['metadata']['label'],'新备注')
   source['metadata']={'label':'','scene':''};loaded.scene_names([source])
   row=loaded.list()[0];self.assertEqual(row['metadata']['label'],'');self.assertNotIn('数据集',row['name'])

 def test_malformed_notes_do_not_hide_media_or_inject_nontext_values(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);folder,_=self.scene(root)
   (folder/'notes.json').write_text(json.dumps({'label':{'bad':'object'},'scene':'控制\n字符','lighting':'x'*301}),encoding='utf8')
   row=MediaLibrary(root).list()[0]
   self.assertEqual(row['metadata'],{'label':'原始备注','scene':'天空'})

if __name__=='__main__':unittest.main()
