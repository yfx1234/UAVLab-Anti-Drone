import unittest,sys,pathlib,tempfile
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'board/zed_tracking/tools'),str(R/'panel')]
class ClockFolderTests(unittest.TestCase):
 def test_clock_reference_survives_wall_clock_jump_but_not_boot_change(self):
  import clock_reference as c
  with patch.object(c.time,'monotonic',return_value=10):anchor=c.reference(1800000000,'boot1')
  with patch.object(c.time,'monotonic',return_value=15):
   self.assertEqual(c.estimate(anchor,'boot1'),1800000005);self.assertIsNone(c.estimate(anchor,'boot2'))
 def test_folder_reveal_only_registered_existing_mp4(self):
  from media_library import MediaLibrary
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);p=root/'fixture/exports/segment_1/rgb_video.view.mp4';p.parent.mkdir(parents=True);p.touch();library=MediaLibrary(root)
   with patch('media_library.reveal_file',create=True) as reveal,patch('media_library.subprocess.Popen'):
    result=library.open_folder('fixture/exports/segment_1/rgb_video.view.mp4');self.assertTrue(pathlib.Path(result['file']).is_file());self.assertEqual(pathlib.Path(result['folder']),pathlib.Path(result['file']).parent);reveal.assert_called_once_with(pathlib.Path(result['file']));self.assertTrue(result['selected'])
   with self.assertRaises(ValueError):library.open_folder('../outside.mp4')
 def test_reveal_failure_is_reported_instead_of_claiming_the_folder_opened(self):
  from media_library import MediaLibrary
  with tempfile.TemporaryDirectory() as tmp:
   root=pathlib.Path(tmp);p=root/'fixture/exports/北京时间 场次/rgb_video.view.mp4';p.parent.mkdir(parents=True);p.touch();library=MediaLibrary(root)
   with patch('media_library.reveal_file',create=True,side_effect=OSError('Shell selection failed')),patch('media_library.subprocess.Popen'):
    with self.assertRaisesRegex(OSError,'Shell selection failed'):library.open_folder(p.relative_to(root).as_posix())
if __name__=='__main__':unittest.main()
