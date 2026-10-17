import pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
from recording_profiles import resolve_codec
class RecordingProfileTests(unittest.TestCase):
    def facts(self):return {'module_sku':'P3767-0000','platform_configuration_consistent':True,'sdk_version':'5.5.0','kernel_version':'5.15.148-tegra'}
    def proof(self):return {'module_sku':'P3767-0000','sdk_version':'5.5.0','kernel_version':'5.15.148-tegra','codec':'H264_LOSSLESS','timestamp_complete':True,'rgb_pixel_exact':False}
    def test_default_keeps_pixel_exact_training_recording(self):
        self.assertEqual(resolve_codec(None,{},{}),'LOSSLESS')
        self.assertEqual(resolve_codec('LOSSLESS',self.facts(),self.proof()),'LOSSLESS')
    def test_fast_recording_needs_explicit_request_and_matching_replay_proof(self):
        self.assertEqual(resolve_codec('H264_LOSSLESS',self.facts(),self.proof()),'H264_LOSSLESS')
        for altered in ({},dict(self.proof(),sdk_version='5.4.0'),dict(self.proof(),timestamp_complete=False)):
            with self.assertRaises(ValueError):resolve_codec('H264_LOSSLESS',self.facts(),altered)
    def test_unknown_or_mismatched_platform_cannot_use_fast_mode(self):
        for altered in ({},dict(self.facts(),platform_configuration_consistent=False),dict(self.facts(),module_sku='P3767-0003')):
            with self.assertRaises(ValueError):resolve_codec('H264_LOSSLESS',altered,self.proof())
        with self.assertRaises(ValueError):resolve_codec('H264',self.facts(),self.proof())
    def test_malformed_proof_disables_fast_mode_without_breaking_default(self):
        for proof in (None,[],True):
            self.assertEqual(resolve_codec(None,self.facts(),proof),'LOSSLESS')
            with self.assertRaises(ValueError):resolve_codec('H264_LOSSLESS',self.facts(),proof)
