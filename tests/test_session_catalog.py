import sys,pathlib,tempfile,json,unittest,os
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'board/zed_tracking/tools'),str(ROOT/'panel')]
class CatalogTests(unittest.TestCase):
 def fixture(self,root,name,stamp,state='complete'):
  p=root/name;p.mkdir();(p/'manifest.json').write_text(json.dumps({'created_at_unix_s':stamp,'metadata':{'label':'old_label'}}));(p/'session_summary.json').write_text(json.dumps({'state':state,'tail_metadata_may_arrive':False}));(p/'raw.bin').write_bytes(b'important_data');return p
 def test_latest_first_and_rename_does_not_change_order_or_uuid(self):
  import session_catalog as c
  with tempfile.TemporaryDirectory() as t:
   root=pathlib.Path(t);older=self.fixture(root,'session_z',1800000000);newer=self.fixture(root,'session_a',1800000010)
   self.assertEqual([x['session_id'] for x in c.list_sessions(root)],['session_a','session_z']);self.assertNotIn('old_label',c.list_sessions(root)[0]['display_name'])
   c.rename(root,'session_z','  自定义名称  ');rows=c.list_sessions(root)
   self.assertEqual(rows[1]['display_name'],'自定义名称');self.assertEqual(rows[1]['created_at_unix_s'],1800000000);self.assertTrue(older.is_dir())
 def test_delete_requires_confirmation_and_finalized_nonactive_scene(self):
  import session_catalog as c
  with tempfile.TemporaryDirectory() as t:
   root=pathlib.Path(t);p=self.fixture(root,'session_test',1800000000)
   for confirmation,active in [('wrong',''),('session_test','session_test')]:
    with self.assertRaises(ValueError):c.delete(root,'session_test',confirmation,active)
    self.assertTrue(p.exists())
   c.delete(root,'session_test','session_test','');self.assertFalse(p.exists())
   p=self.fixture(root,'session_recording',1800000000,'recording')
   with self.assertRaises(ValueError):c.delete(root,'session_recording','session_recording','')
   self.assertTrue(p.exists())
 def test_paths_symlinks_and_names_rejected(self):
  import session_catalog as c
  with tempfile.TemporaryDirectory() as t:
   root=pathlib.Path(t);p=self.fixture(root,'session_test',1800000000)
   for name in ['', 'x'*101,'line\nname']:
    with self.assertRaises(ValueError):c.rename(root,'session_test',name)
   for name in ['../outside','.', '', '/absolute']:
    with self.assertRaises(ValueError):c.delete(root,name,name,'')
 def test_legacy_uses_creation_manifest_not_directory_uuid_and_is_uncertain(self):
  import session_catalog as c
  with tempfile.TemporaryDirectory() as t:
   root=pathlib.Path(t);p=self.fixture(root,'session_test',1800000000);(p/'manifest.json').write_text('{}');os.utime(p/'manifest.json',(1800000100,1800000100))
   row=c.list_sessions(root)[0];self.assertEqual(row['created_at_unix_s'],1800000100);self.assertTrue(row['created_time_uncertain'])
if __name__=='__main__':unittest.main()
