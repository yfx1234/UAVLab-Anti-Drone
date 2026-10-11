import pathlib,sys,unittest
import tempfile,json,hashlib
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from session_manager import SessionManager

class SelectiveDownloadTests(unittest.TestCase):
    def test_ready_media_is_readonly_and_rejects_changed_source(self):
        sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
        import export_session,perception_supervisor
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);scene=root/'session_cached';(scene/'segments').mkdir(parents=True)
            source=scene/'segments/segment_1.svo2';source.write_bytes(b'raw')
            video=scene/'exports/segment_1/rgb_video.mp4';video.parent.mkdir(parents=True);video.write_bytes(b'video')
            (scene/'session_summary.json').write_text(json.dumps({'state':'complete','verified_frames':1,'metadata_complete':True,'tail_metadata_may_arrive':False}))
            asset={'contract':'native_stream_v1','sources':[{'name':source.name,'bytes':3,'mtime_ns':source.stat().st_mtime_ns}],
                   'reports':[{'video':str(video),'video_bytes':5}]}
            (scene/'media_assets.json').write_text(json.dumps({'rgb_video':asset}))
            with patch.object(perception_supervisor,'RECORDS',root):
                self.assertTrue(export_session.ready_media('session_cached','rgb_video')['ready'])
                source.write_bytes(b'changed');self.assertFalse(export_session.ready_media('session_cached','rgb_video')['ready'])
    def test_rgb_video_uses_native_stream_export_without_png_stage(self):
        sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
        import export_session,perception_supervisor
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);scene=root/'session_test';(scene/'segments').mkdir(parents=True)
            (scene/'segments/segment_1.svo2').touch()
            summary={'segments':[{'segment_id':1}],'submitted_frames':1,'metadata_complete':True,'tail_metadata_may_arrive':False,'state':'closed_pending_validation'}
            (scene/'session_summary.json').write_text(json.dumps(summary))
            (scene/'frames.jsonl').write_text(json.dumps({'stamp_ns':10,'segment_id':1,'submitted':True})+'\n')
            report={'frames':1,'timestamp_sha256':hashlib.sha256(b'10\n').hexdigest(),'video':str(scene/'exports/segment_1/rgb_video.mp4'),'video_width':1920,'video_height':1080,'video_fps':15}
            def tool(cmd,**kwargs):
                self.assertEqual(cmd[-1],'rgb_video','must not generate PNG intermediates')
                self.assertEqual(cmd[-2],'0','video keeps native FPS')
                video=pathlib.Path(report['video']);video.parent.mkdir(parents=True,exist_ok=True);video.write_bytes(b'fixture')
                class Result:returncode=0;stdout=json.dumps(report);stderr=''
                return Result()
            with patch.object(perception_supervisor,'RECORDS',root),patch.object(export_session.subprocess,'run',tool):
                result=export_session.export('session_test',mode='rgb_video')
            self.assertEqual(result['state'],'complete')
    def test_rgb_does_not_include_svo_depth_images_or_logs(self):
        files=[{'name':n,'bytes':3} for n in ('segments/segment_1.svo2','frames.jsonl','calibration.json',
            'exports/segment_1/rgb_video.mp4','exports/segment_1/depth_video.mp4','exports/segment_1/frame_1.png')]
        self.assertEqual([f['name'] for f in SessionManager.select_files(files,'rgb')],['exports/segment_1/rgb_video.mp4'])
        self.assertEqual([f['name'] for f in SessionManager.select_files(files,'depth')],['exports/segment_1/depth_video.mp4'])
        self.assertEqual([f['name'] for f in SessionManager.select_files(files,'stereo')],['segments/segment_1.svo2'])
        self.assertEqual([f['name'] for f in SessionManager.select_files(files,'images')],['exports/segment_1/frame_1.png'])

    def test_auto_transfer_uncapped_only_after_capture_stopped(self):
        self.assertEqual(SessionManager.transfer_limit({'pid':None,'camera_state':'stopped'}),0)
        self.assertEqual(SessionManager.transfer_limit({'pid':12,'camera_state':'running'}),256*1024)
        self.assertEqual(SessionManager.transfer_limit({'pid':12,'camera_state':'unresponsive'}),256*1024)
        self.assertEqual(SessionManager.transfer_limit({}),256*1024)

    def test_download_rejects_unknown_kind_before_ssh(self):
        with self.assertRaises(ValueError):SessionManager.select_files([],'everything_else')

if __name__=='__main__':unittest.main()
