import contextlib,json,pathlib,sys,tempfile,unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'board/zed_tracking/tools'),str(ROOT/'panel')]
import session_catalog as catalog
import perception_supervisor as supervisor

class OperatorNotesModelsTests(unittest.TestCase):
 def fixture(self,root):
  p=root/'session_data';p.mkdir()
  (p/'manifest.json').write_text(json.dumps({'created_at_unix_s':1791875387,'created_time_source':'pc_reference_monotonic','metadata':{'label':'数据集','scene':'天空','lighting':'阴天'}}))
  return p
 def test_original_recording_notes_are_exposed_to_operator(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=self.fixture(pathlib.Path(tmp));row=catalog.scene_info(p)
   self.assertEqual(row.get('metadata'),{'label':'数据集','scene':'天空','lighting':'阴天'})
 def test_notes_saved_during_recording_preserve_original_manifest_and_time(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);p=self.fixture(root);original=(p/'manifest.json').read_bytes()
   self.assertTrue(hasattr(catalog,'update_metadata'),'recording-time note update must exist')
   catalog.update_metadata(root,'session_data',{'label':'数据集-远距离'})
   row=catalog.scene_info(p)
   self.assertEqual(row['metadata']['label'],'数据集-远距离');self.assertEqual(row['metadata']['scene'],'天空')
   self.assertEqual(row['created_at_unix_s'],1791875387);self.assertEqual((p/'manifest.json').read_bytes(),original)
   for invalid in [{'label':'x'*301},{'label':'line\nname'},{'model_id':'bad'},{'label':False}]:
    with self.assertRaises(ValueError):catalog.update_metadata(root,'session_data',invalid)
 def test_requested_model_is_bound_without_loading_when_detector_paused(self):
  current={'boot_id':'b','run_id':'r','revision':2,'pid':20,'model_id':'a'*64,'pipeline':{'record_state':'idle'}}
  model={'model_id':'b'*64}
  with tempfile.TemporaryDirectory() as tmp,patch.object(supervisor,'ROOT',pathlib.Path(tmp)),patch.object(supervisor,'state',return_value=dict(current)),patch.object(supervisor,'state_lock',contextlib.nullcontext),patch.object(supervisor,'model_by_id',return_value=model) as lookup,patch.object(supervisor,'stage_model',return_value='fixed-engine'),patch.object(supervisor,'node_command',return_value={'ok':True}) as command:
   req={'action':'set_detector','params':{'enabled':False,'model_id':'b'*64},'request_id':'bind','expected_boot_id':'b','expected_run_id':'r','expected_revision':2}
   self.assertTrue(supervisor.dispatch(req)['ok'])
   lookup.assert_called_once_with('b'*64)
   self.assertEqual(command.call_args.args[2]['model_id'],'b'*64)
   self.assertEqual(json.loads((pathlib.Path(tmp)/'supervisor.json').read_text())['model_id'],'b'*64)
 def test_different_model_is_rejected_in_active_raw_recording_even_if_yolo_paused(self):
  current={'boot_id':'b','run_id':'r','revision':2,'pid':20,'model_id':'a'*64,'pipeline':{'record_state':'recording'}}
  with tempfile.TemporaryDirectory() as tmp,patch.object(supervisor,'ROOT',pathlib.Path(tmp)),patch.object(supervisor,'state',return_value=current),patch.object(supervisor,'state_lock',contextlib.nullcontext),patch.object(supervisor,'model_by_id',return_value={'model_id':'b'*64}),patch.object(supervisor,'stage_model',side_effect=AssertionError('must reject before staging')):
   req={'action':'set_detector','params':{'enabled':False,'model_id':'b'*64},'request_id':'blocked','expected_boot_id':'b','expected_run_id':'r','expected_revision':2}
   with self.assertRaises(supervisor.Conflict):supervisor.dispatch(req)
 def test_model_identity_cannot_be_an_engine_path_or_unregistered_value(self):
  for mid in ('../../weights/arbitrary.engine','',None,False):
   with self.subTest(mid=mid),self.assertRaises(ValueError):supervisor.validate_action('set_detector',{'enabled':True,'model_id':mid})
  with patch.object(supervisor,'models',return_value=[]),self.assertRaises(ValueError):supervisor.model_by_id('a'*64)

if __name__=='__main__':unittest.main()
