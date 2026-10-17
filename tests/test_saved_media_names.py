import hashlib
import json
import pathlib
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'panel'))
from media_library import MediaLibrary


class SavedMediaNamesTests(unittest.TestCase):
    def fixture(self, root):
        library = MediaLibrary(root)
        library.scene_names([{'session_id': 'scene', 'display_name': '场次',
                              'created_at_unix_s': 1791875745.879}])
        return library, library.registry.folder('scene', create=True)

    def test_preparation_uses_recording_time_and_kind_in_actual_name(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            library, folder = self.fixture(tmp)
            source = folder / 'exports/segment_1/depth_video.mp4'
            source.parent.mkdir(parents=True);source.write_bytes(b'source')
            def encode(args, *rest):pathlib.Path(args[-1]).write_bytes(b'playable')
            with patch.object(library, '_can_stream_copy', return_value=True), patch.object(library, '_run', side_effect=encode):
                row = library.prepare({'session_id': 'scene', 'folder': str(folder), 'kind': 'depth',
                                       'files': [{'file': 'exports/segment_1/depth_video.mp4'}]})['media'][0]
            self.assertTrue(pathlib.Path(row['path']).name.startswith('20261013-151545_深度视频_第1段'))
            self.assertEqual(library.selected(row['relative'])['kind'], 'depth')

    def test_migration_preserves_bytes_cache_references_and_legacy_identity(self):
        from media_filename_migration import MediaFilenameMigration
        with tempfile.TemporaryDirectory() as tmp:
            library, folder = self.fixture(tmp)
            raw = folder / 'exports/segment_1/rgb_video.mp4'
            raw.parent.mkdir(parents=True);raw.write_bytes(b'raw')
            video = raw.with_name('rgb_video.view.mp4');video.write_bytes(b'playable')
            binding = {'source_bytes': 3, 'source_mtime_ns': raw.stat().st_mtime_ns, 'source_sha256': 'abc'}
            video.with_suffix('.json').write_text(json.dumps(binding))
            annotation = folder / 'exports/annotated/annotated_glad-736x1280_20261016_175039.view.mp4'
            annotation.parent.mkdir();annotation.write_bytes(b'annotation')
            annotation.with_suffix('.json').write_text(json.dumps({'source_relative': video.relative_to(library.root).as_posix()}))
            (folder/'download_integrity_rgb.json').write_text(json.dumps([{'file': 'exports/segment_1/rgb_video.mp4', 'sha256': 'abc'}]))
            before = sorted(hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.rglob('*.mp4'))
            migration = MediaFilenameMigration(library)
            plan = migration.plan()
            self.assertEqual(len(plan), 3)
            self.assertTrue(raw.is_file(), 'planning must not rename files')
            migration.apply(plan)
            self.assertEqual(migration.plan(), [])
            self.assertEqual(before, sorted(hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.rglob('*.mp4')))
            rows = library.list()
            detected = next(r for r in rows if r['kind'] == 'annotated')
            source = next(r for r in rows if r['kind'] == 'rgb')
            self.assertIn('通用检测', pathlib.Path(detected['path']).name)
            self.assertIn('1280x736', pathlib.Path(detected['path']).name)
            self.assertIn('17:50:39', detected['name'])
            data = json.loads(pathlib.Path(detected['path']).with_suffix('.json').read_text(encoding='utf-8'))
            self.assertEqual(data['source_relative'], source['relative'])
            self.assertTrue(library._cache_matches(pathlib.Path(source['path']), pathlib.Path(source['path']).with_suffix('.json'), binding))
            integrity=json.loads((folder/'download_integrity_rgb.json').read_text(encoding='utf-8'))[0]
            self.assertEqual(integrity['remote_file'], 'exports/segment_1/rgb_video.mp4')
            self.assertTrue((folder/integrity['file']).is_file())

    def test_collision_refuses_to_overwrite_any_video(self):
        from media_filename_migration import MediaFilenameMigration
        with tempfile.TemporaryDirectory() as tmp:
            library, folder = self.fixture(tmp)
            video = folder/'exports/session/rgb_video.view.mp4'
            video.parent.mkdir(parents=True);video.write_bytes(b'original')
            migration=MediaFilenameMigration(library);plan=migration.plan()
            target=library.root/plan[0]['new'];target.write_bytes(b'conflict')
            with self.assertRaisesRegex(ValueError, '存在'):migration.apply(plan)
            self.assertEqual(video.read_bytes(), b'original')
            self.assertEqual(target.read_bytes(), b'conflict')

    def test_failed_second_move_rolls_back_first_move(self):
        from media_filename_migration import MediaFilenameMigration
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            library, folder = self.fixture(tmp)
            video=folder/'exports/session/rgb_video.view.mp4'
            video.parent.mkdir(parents=True);video.write_bytes(b'video')
            sidecar=video.with_suffix('.json');sidecar.write_bytes(b'{"cache": true}')
            migration=MediaFilenameMigration(library);plan=migration.plan()
            rename=pathlib.Path.rename
            def fail_metadata(path,target):
                if path == sidecar:raise PermissionError('occupied')
                return rename(path,target)
            with patch.object(pathlib.Path,'rename',fail_metadata), self.assertRaises(PermissionError):migration.apply(plan)
            self.assertEqual(video.read_bytes(),b'video')
            self.assertEqual(sidecar.read_bytes(),b'{"cache": true}')
            self.assertFalse((library.root/plan[0]['new']).exists())

    def test_whole_video_concatenates_named_segments_and_reuses_cache(self):
        import subprocess
        from preview_recorder import PreviewRecorder
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            library, folder=self.fixture(tmp);files=[]
            for number in (1,2):
                path=folder/f'exports/segment_{number}/depth_video.mp4'
                path.parent.mkdir(parents=True)
                subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-f','lavfi','-i','testsrc2=size=192x108:rate=15','-frames:v','8','-c:v','libx264','-pix_fmt','yuv420p',str(path)],check=True)
                files.append({'file':path.relative_to(folder).as_posix(),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
            result={'session_id':'scene','folder':str(folder),'kind':'depth','files':files,
                    'session_summary':{'state':'complete','segments':[{'segment_id':1},{'segment_id':2}]}}
            prepared=library.prepare(result)
            output=pathlib.Path(prepared['media'][0]['path'])
            self.assertEqual(output.name,'20261013-151545_深度视频_整场.view.mp4')
            decoded=subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-i',str(output),'-f','framemd5','-'],capture_output=True,check=True).stdout
            self.assertEqual(sum(not line.startswith(b'#') for line in decoded.splitlines()),16)
            with patch.object(library,'_run',side_effect=AssertionError('cache must not encode')):
                self.assertEqual(library.prepare(result)['media'][0]['preparation'],'cache')


if __name__ == '__main__':unittest.main()
