import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'panel'))
from annotation_preview import AnnotationPreview
from media_library import MediaLibrary
from preview_recorder import PreviewRecorder


class AnnotationPreviewTests(unittest.TestCase):
    def test_new_scene_needs_only_annotation_transfer_and_becomes_h264(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            fixture = root / 'fixture.mp4'
            subprocess.run([PreviewRecorder.ffmpeg(), '-v', 'error', '-f', 'lavfi', '-i',
                            'testsrc2=size=192x108:rate=15', '-frames:v', '8', '-c:v', 'mpeg4',
                            str(fixture)], check=True)
            calls = []
            class Board:
                def call(self, action, params=None, *args, **kwargs):
                    calls.append(action)
                    if action == 'list_models': return [{'model_id': 'a' * 64}]
                    if action == 'list_sessions': return [{'session_id': 'scene_new', 'display_name': '新场次', 'created_at_unix_s': 1792100000}]
                    if action == 'ready_media': return {'ready': True}
                    if action == 'status': return {'camera_state': 'stopped', 'pid': None}
                    raise AssertionError(action)
                def stream_json(self, command, progress, timeout):
                    progress({'phase_code': 'inference', 'frames': 8, 'total_frames': 8})
                    return {'ok': True, 'output': '/home/uavlab/Jetson_ROS2_V0/perception_records/scene_new/annotated_test.mp4', 'frames': 8}
                def fetch_file(self, remote, local, progress=None, **kwargs):
                    calls.append('annotation_transfer')
                    pathlib.Path(local).write_bytes(fixture.read_bytes())
                    if progress: progress(fixture.stat().st_size, fixture.stat().st_size)
            library = MediaLibrary(root / 'library')
            updates = []
            result = AnnotationPreview(Board(), library).run({'session_id': 'scene_new', 'model_id': 'a' * 64, 'confidence': .3}, updates.append)
            output = library.path(result['relative'])
            self.assertTrue(library._can_stream_copy(output), 'Published previews must be browser-compatible H264')
            self.assertEqual(len(library.list()), 1)
            self.assertNotIn('download_session', calls)
            self.assertIn('annotation_transfer', calls)
            self.assertTrue(any(p.get('frames') == 8 for p in updates))
            self.assertTrue(any(p.get('phase_code') == 'local_encode' for p in updates))

    def test_invalid_confidence_rejected_before_board_work(self):
        for value in [0, 1.1, float('nan'), True, 'bad']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                AnnotationPreview(None, None).run({'session_id': 'scene', 'model_id': 'a' * 64, 'confidence': value})

    def test_repair_old_preview_preserves_path_and_frame_count(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            source = root / 'scene/exports/annotated/old.view.mp4'
            source.parent.mkdir(parents=True)
            subprocess.run([PreviewRecorder.ffmpeg(), '-v', 'error', '-f', 'lavfi', '-i',
                            'testsrc2=size=192x108:rate=15', '-frames:v', '8', '-c:v', 'mpeg4', str(source)], check=True)
            library = MediaLibrary(root)
            library.prepare_annotation(source, source)
            self.assertTrue(library._can_stream_copy(source))
            decoded = subprocess.run([PreviewRecorder.ffmpeg(), '-v', 'error', '-i', str(source),
                                      '-f', 'framemd5', '-'], capture_output=True, check=True).stdout
            self.assertEqual(sum(not line.startswith(b'#') for line in decoded.splitlines()), 8)


if __name__ == '__main__': unittest.main()
