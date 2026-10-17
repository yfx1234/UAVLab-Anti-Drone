import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'panel'))
from media_library import MediaLibrary
from preview_recorder import PreviewRecorder


class MediaPreparationTests(unittest.TestCase):
    def fixture(self, root, codec='libx264', pixel='yuv420p'):
        source = root / 'session_test/exports/segment_1/rgb_video.mp4'
        source.parent.mkdir(parents=True)
        subprocess.run([PreviewRecorder.ffmpeg(), '-v', 'error', '-f', 'lavfi', '-i',
                        'testsrc2=size=192x108:rate=15', '-frames:v', '8', '-c:v', codec,
                        '-pix_fmt', pixel, str(source)], check=True)
        return source, {'session_id': 'session_test', 'folder': str(source.parents[2]), 'kind': 'rgb',
                        'files': [{'file': 'exports/segment_1/rgb_video.mp4',
                                   'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}],
                        'timings_s': {'transfer': 2.5}}

    def frame_hash(self, path):
        return subprocess.run([PreviewRecorder.ffmpeg(), '-v', 'error', '-i', str(path),
                               '-map', '0:v:0', '-pix_fmt', 'yuv420p', '-f', 'hash', '-hash',
                               'sha256', '-'], check=True, capture_output=True).stdout

    def test_compatible_h264_is_not_reencoded(self):
        with tempfile.TemporaryDirectory() as temp:
            source, result = self.fixture(pathlib.Path(temp))
            original_hash = self.frame_hash(source)
            library = MediaLibrary(pathlib.Path(temp))
            output = pathlib.Path(library.prepare(result)['media'][0]['path'])
            self.assertEqual(self.frame_hash(output), original_hash,
                             'H264 playback remux must preserve every decoded source frame')

    def test_broken_cache_sidecar_can_be_regenerated(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            _, result = self.fixture(root, 'mpeg4')
            library = MediaLibrary(root)
            prepared = library.prepare(result)
            output = pathlib.Path(prepared['media'][0]['path'])
            output.with_suffix('.json').write_text('{broken', encoding='utf-8')
            regenerated = library.prepare(result)
            self.assertEqual(len(regenerated['media']), 1)
            self.assertTrue(output.is_file())
            self.assertTrue(json.loads(output.with_suffix('.json').read_text(encoding='utf-8')))

    def test_cached_prepare_does_not_probe_or_encode_and_keeps_transfer_timing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            _, result = self.fixture(root, 'mpeg4')
            library = MediaLibrary(root)
            first = library.prepare(result)
            with (patch.object(library, '_run', side_effect=AssertionError('valid cache must not encode')),
                  patch.object(library, '_can_stream_copy', side_effect=AssertionError('valid cache must not probe'))):
                cached = library.prepare(result)
            self.assertEqual(cached['media'][0]['path'], first['media'][0]['path'])
            self.assertEqual(cached['media'][0]['preparation'], 'cache')
            self.assertEqual(cached['timings_s']['transfer'], 2.5)
            self.assertGreaterEqual(cached['timings_s']['local_prepare_s'], 0)

    def test_changed_source_is_prepared_again(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            _, result = self.fixture(root, 'mpeg4')
            library = MediaLibrary(root)
            first = library.prepare(result)
            source = library.registry.folder('session_test') / result['files'][0]['file']
            with source.open('ab') as stream: stream.write(b'ignored_trailing_data')
            result['files'][0]['sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
            with patch.object(library, '_run', wraps=library._run) as convert:
                prepared = library.prepare(result)
            self.assertEqual(convert.call_count, 1)
            self.assertEqual(prepared['media'][0]['path'], first['media'][0]['path'])
            self.assertEqual(prepared['media'][0]['preparation'], 'transcode')
            binding = json.loads(pathlib.Path(prepared['media'][0]['path']).with_suffix('.json').read_text(encoding='utf-8'))
            self.assertEqual(binding['source_sha256'], result['files'][0]['sha256'])

    def test_h264_with_incompatible_pixel_format_still_transcodes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            _, result = self.fixture(root, pixel='yuv444p')
            prepared = MediaLibrary(root).prepare(result)
            self.assertEqual(prepared['media'][0]['preparation'], 'transcode')
            output = pathlib.Path(prepared['media'][0]['path'])
            probe = subprocess.run([PreviewRecorder.ffmpeg(), '-hide_banner', '-i', str(output),
                                    '-map', '0:v:0', '-c:v', 'copy', '-frames:v', '0', '-f', 'null', '-'],
                                   capture_output=True, text=True, encoding='utf-8', check=True)
            self.assertRegex(probe.stderr, r'Video: h264 .*yuv420p')


if __name__ == '__main__': unittest.main()
