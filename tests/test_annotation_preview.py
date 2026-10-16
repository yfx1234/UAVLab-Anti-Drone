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
    def test_selected_file_is_transferred_and_result_keeps_source_identity(self):
        import hashlib
        with tempfile.TemporaryDirectory() as temp:
            root=pathlib.Path(temp);library=MediaLibrary(root/'library')
            library.scene_names([{'session_id':'scene_old','display_name':'旧场次','created_at_unix_s':1791875000}])
            source=library.registry.folder('scene_old',create=True)/'exports/segment_2/rgb_video.view.mp4'
            source.parent.mkdir(parents=True)
            subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-f','lavfi','-i','testsrc2=size=192x108:rate=15','-frames:v','8','-c:v','mpeg4',str(source)],check=True)
            relative=source.relative_to(library.root).as_posix();digest=hashlib.sha256(source.read_bytes()).hexdigest()
            class Transport:
                cleaned=False
                def upload(self,job_id,stream,size,progress):
                    self.job_id=job_id;self.data=stream.read()
                    if len(self.data)!=size:raise AssertionError('wrong upload size')
                    progress(size,size)
                def cleanup(self,job_id):
                    if job_id!=self.job_id:raise AssertionError('wrong cleanup job')
                    self.cleaned=True
            transport=Transport()
            class Board:
                def call(self,action,*args,**kwargs):
                    if action=='list_models':return [{'model_id':'a'*64,'name':'glad-384x640.engine','manifest':{'input_nchw':[1,3,384,640]}}]
                    if action=='status':return {'pid':None}
                    raise AssertionError('Unexpected scene-wide action: '+action)
                def stream_json(self,command,progress,timeout):
                    if '--input-job '+transport.job_id not in command or '--input-sha256 '+digest not in command:raise AssertionError(command)
                    progress({'phase_code':'inference','frames':8,'total_frames':8})
                    return {'ok':True,'output':'/home/uavlab/Jetson_ROS2_V0/perception_annotation_jobs/'+transport.job_id+'/output.mp4','frames':8,'input_nchw':[1,3,384,640],'source_sha256':digest,'model_tag':'glad-384x640'}
                def fetch_file(self,remote,local,progress=None,**kwargs):
                    pathlib.Path(local).write_bytes(transport.data)
                    if progress:progress(len(transport.data),len(transport.data))
            updates=[]
            result=AnnotationPreview(Board(),library,transport=transport).run({'relative':relative,'session_id':'wrong_scene','model_id':'a'*64,'confidence':.3},updates.append)
            output=library.path(result['relative'])
            self.assertTrue(library._can_stream_copy(output))
            metadata=json.loads(output.with_suffix('.json').read_text(encoding='utf-8'))
            self.assertEqual(metadata['source_relative'],relative)
            self.assertEqual(metadata['source_sha256'],digest)
            self.assertEqual(metadata['scene_created_at_unix_s'],1791875000)
            self.assertEqual(metadata['model_family'],'general')
            self.assertTrue(transport.cleaned)
            self.assertEqual(source.read_bytes(),transport.data)
            self.assertTrue(any(p.get('phase_code')=='upload' for p in updates))
            self.assertTrue(any(p.get('phase_code')=='local_encode' for p in updates))
            decoded=subprocess.run([PreviewRecorder.ffmpeg(),'-v','error','-i',str(output),'-f','framemd5','-'],capture_output=True,check=True).stdout
            self.assertEqual(sum(not line.startswith(b'#') for line in decoded.splitlines()),8)
            transport.cleanup=lambda job:(_ for _ in ()).throw(OSError('cleanup unavailable'))
            result=AnnotationPreview(Board(),library,transport=transport).run({'relative':relative,'model_id':'a'*64},updates.append)
            self.assertIn('cleanup unavailable',result.get('cleanup_warning',''))
            self.assertIn('清理失败',updates[-1]['stage'])

    def test_invalid_confidence_rejected_before_board_work(self):
        for value in [0, 1.1, float('nan'), True, 'bad']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                AnnotationPreview(None, None).run({'session_id': 'scene', 'model_id': 'a' * 64, 'confidence': value})

    def test_failed_upload_cleans_job_without_publishing_or_deleting_source(self):
        with tempfile.TemporaryDirectory() as temp:
            library=MediaLibrary(temp);library.scene_names([{'session_id':'scene','display_name':'场次','created_at_unix_s':1800000000}])
            source=library.registry.folder('scene',create=True)/'exports/session/rgb_video.view.mp4'
            source.parent.mkdir(parents=True);source.write_bytes(b'selected source')
            relative=source.relative_to(library.root).as_posix()
            class Board:
                def call(self,action):return [{'model_id':'a'*64,'name':'custom.engine'}] if action=='list_models' else {'pid':None}
            class Transport:
                cleaned=False
                def upload(self,job,stream,size,progress):self.job=job;raise OSError('upload failed')
                def cleanup(self,job):
                    if job!=self.job:raise AssertionError('wrong cleanup')
                    self.cleaned=True
            transport=Transport()
            with self.assertRaisesRegex(OSError,'upload failed'):
                AnnotationPreview(Board(),library,transport).run({'relative':relative,'model_id':'a'*64})
            self.assertTrue(transport.cleaned)
            self.assertEqual(source.read_bytes(),b'selected source')
            self.assertEqual(len(library.list()),1)
            self.assertFalse(list(source.parents[2].glob('annotated/*')))

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
