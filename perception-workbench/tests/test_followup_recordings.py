import json,pathlib,sys,tempfile,subprocess,unittest
import contextlib
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'board/zed_tracking/tools'),str(ROOT/'panel')]
from session_catalog import scene_info
from media_library import MediaLibrary
from preview_recorder import PreviewRecorder

class FollowupRecordingsTests(unittest.TestCase):
 def test_record_start_refreshes_boot_scoped_reference_without_reconnect(self):
  import perception_supervisor as s
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);current={'boot_id':'new_boot','run_id':'capture','revision':1,'pid':123}
   (root/'clock_reference.json').write_text(json.dumps({'boot_id':'old_boot','pc_unix_s':1791875387.094,'board_monotonic_s':1}))
   def command(*args):
    anchor=json.loads((root/'clock_reference.json').read_text())
    self.assertEqual(anchor['boot_id'],'new_boot')
    self.assertEqual(anchor['pc_unix_s'],1791956458.011862)
    return {'ok':True}
   with patch.object(s,'ROOT',root),patch.object(s,'state',return_value=current),patch.object(s,'state_lock',contextlib.nullcontext),patch.object(s,'node_command',side_effect=command),patch('device_capabilities.collect',return_value={}),patch('recording_profiles.resolve_codec',return_value='LOSSLESS'):
    s.dispatch({'action':'start_raw_record','params':{'client_started_at_unix_s':1791956458.011862},'request_id':'field_start','expected_boot_id':'new_boot','expected_run_id':'capture','expected_revision':1})

 def test_legacy_wrong_board_date_uses_saved_pc_request_time(self):
  with tempfile.TemporaryDirectory() as tmp:
   folder=pathlib.Path(tmp)/'session_field';folder.mkdir()
   (folder/'manifest.json').write_text(json.dumps({'created_at_unix_s':1791363102.3316603,'created_time_source':'board_clock','metadata':{'client_started_at_unix_s':1791875387.094}}))
   row=scene_info(folder)
   self.assertEqual(row['created_at_unix_s'],1791875387.094)
   self.assertEqual(row['display_name'],'2026-10-13 15:09:47 北京时间')
   self.assertFalse(row['created_time_uncertain'])
   self.assertEqual(json.loads((folder/'manifest.json').read_text())['created_at_unix_s'],1791363102.3316603)

 def test_confirmed_monotonic_time_and_custom_name_are_preserved(self):
  with tempfile.TemporaryDirectory() as tmp:
   folder=pathlib.Path(tmp)/'session_field';folder.mkdir()
   (folder/'manifest.json').write_text(json.dumps({'created_at_unix_s':1791875400,'created_time_source':'pc_reference_monotonic','metadata':{'client_started_at_unix_s':1791875387.094}}))
   (folder/'display.json').write_text(json.dumps({'name':'外场无人机'}))
   row=scene_info(folder)
   self.assertEqual(row['created_at_unix_s'],1791875400);self.assertEqual(row['display_name'],'外场无人机')

 def test_offline_import_recovers_date_from_original_pc_request(self):
  from scene_registry import SceneRegistry
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);folder=root/'session_field';folder.mkdir()
   (folder/'manifest.json').write_text(json.dumps({'created_at_unix_s':1791363102.3316603,'created_time_source':'board_clock','metadata':{'client_started_at_unix_s':1791875387.094}}))
   registry=SceneRegistry(root);registry.discover();entry=registry.scenes['session_field']
   self.assertEqual(entry['created_at_unix_s'],1791875387.094)
   self.assertFalse(entry['created_time_uncertain'])
   self.assertTrue(registry.folder('session_field').name.startswith('2026-10-13'))

 def fixture(self,root,missing=False):
  folder=root/'session_test';files=[]
  for number,count in [(2,5),(1,8)]:
   source=folder/f'exports/segment_{number}/rgb_video.mp4';source.parent.mkdir(parents=True)
   subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-f','lavfi','-i',f'color=c={"red" if number==1 else "blue"}:size=192x108:rate=15','-frames:v',str(count),'-c:v','libx264','-pix_fmt','yuv420p',str(source)],check=True)
   if not missing or number==1:files.append({'file':source.relative_to(folder).as_posix()})
  (folder/'session_summary.json').write_text(json.dumps({'state':'complete','segments':[{'segment_id':1},{'segment_id':2}]}))
  return {'session_id':'session_test','folder':str(folder),'kind':'rgb','files':files}

 def test_full_scene_contains_every_segment_in_numeric_order(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);result=self.fixture(root);library=MediaLibrary(root)
   prepared=library.prepare(result)
   self.assertEqual(prepared['media'][0]['segment'],'session')
   output=pathlib.Path(prepared['media'][0]['path'])
   raw=subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-i',str(output),'-vsync','0','-f','rawvideo','-pix_fmt','rgb24','pipe:1'],capture_output=True,check=True).stdout
   self.assertEqual(len(raw),192*108*3*13,'entire scene must keep 8+5 frames')
   self.assertGreater(raw[0],raw[2]);self.assertGreater(raw[-1],raw[-3])
   self.assertEqual(library.list()[0]['segment'],'session')
   again=library.prepare(result);self.assertEqual(again['media'][0]['preparation'],'cache')

 def test_partial_download_never_appears_as_full_scene(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);result=self.fixture(root,missing=True)
   prepared=MediaLibrary(root).prepare(result)
   self.assertFalse(any(row['segment']=='session' for row in prepared['media']))

 def test_selective_video_download_uses_rpc_summary_without_metadata_files(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);result=self.fixture(root)
   summary=root/'session_test/session_summary.json'
   result['session_summary']=json.loads(summary.read_text());summary.unlink()
   prepared=MediaLibrary(root).prepare(result)
   self.assertEqual(prepared['media'][0]['segment'],'session')

if __name__=='__main__':unittest.main()
