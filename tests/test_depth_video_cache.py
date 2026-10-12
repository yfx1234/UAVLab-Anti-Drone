"""Depth rendering fixes must invalidate old depth assets without redoing RGB."""
import json,pathlib,sys,tempfile,unittest
from unittest.mock import patch

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
import export_session,perception_supervisor

class DepthVideoCacheTests(unittest.TestCase):
    def test_millimeter_bug_depth_cache_is_not_ready_but_rgb_is(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);scene=root/'session_depth';(scene/'segments').mkdir(parents=True)
            source=scene/'segments/segment_1.svo2';source.write_bytes(b'stereo')
            (scene/'session_summary.json').write_text(json.dumps({'state':'complete','verified_frames':1,'metadata_complete':True,'tail_metadata_may_arrive':False}))
            assets={}
            for mode in ('rgb_video','depth_video'):
                video=scene/'exports/segment_1'/(mode+'.mp4');video.parent.mkdir(parents=True,exist_ok=True);video.write_bytes(b'old video')
                assets[mode]={'contract':'native_stream_v1','sources':[{'name':source.name,'bytes':source.stat().st_size,'mtime_ns':source.stat().st_mtime_ns}],
                              'reports':[{'video':str(video),'video_bytes':video.stat().st_size}]}
            (scene/'media_assets.json').write_text(json.dumps(assets))
            with patch.object(perception_supervisor,'RECORDS',root):
                self.assertTrue(export_session.ready_media('session_depth','rgb_video')['ready'])
                self.assertFalse(export_session.ready_media('session_depth','depth_video')['ready'],'v1 depth was clipped at10.5mm; regenerate it')

    def test_new_depth_contract_requires_real_depth_quality_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);scene=root/'session_depth';(scene/'segments').mkdir(parents=True)
            source=scene/'segments/segment_1.svo2';source.write_bytes(b'stereo')
            video=scene/'exports/segment_1/depth_video.mp4';video.parent.mkdir(parents=True);video.write_bytes(b'new video')
            (scene/'session_summary.json').write_text(json.dumps({'state':'complete','verified_frames':1,'metadata_complete':True,'tail_metadata_may_arrive':False}))
            report={'video':str(video),'video_bytes':video.stat().st_size,'depth_unit':'METER','depth_view':'SDK_VIEW_DEPTH',
                    'depth_minimum_m':.3,'depth_maximum_m':10.5,'depth_valid_sample_fraction':.8}
            asset={'contract':'native_depth_meter_view_v2','sources':[{'name':source.name,'bytes':source.stat().st_size,'mtime_ns':source.stat().st_mtime_ns}],
                   'reports':[report]}
            path=scene/'media_assets.json';path.write_text(json.dumps({'depth_video':asset}))
            with patch.object(perception_supervisor,'RECORDS',root):
                self.assertTrue(export_session.ready_media('session_depth','depth_video')['ready'])
                report['depth_valid_sample_fraction']=0;path.write_text(json.dumps({'depth_video':asset}))
                self.assertFalse(export_session.ready_media('session_depth','depth_video')['ready'],'all-invalid video must not be offered as valid depth')

if __name__=='__main__':unittest.main()
