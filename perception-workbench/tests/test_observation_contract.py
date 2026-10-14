import pathlib,sys,unittest
from types import SimpleNamespace as N
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
from inspect_observations import check_frame

class ObservationContractTests(unittest.TestCase):
 def frame(self,**changes):
  return N(**dict({'schema_version':2,'boot_id':'boot','run_id':'run','header':N(frame_id='zed_left_camera_optical_frame'),
      'processing_latency_s':.1,'observations_fresh':True,'detector_enabled':True,'targets':[]},**changes))
 def target(self,mode='size_estimate',**changes):
  return N(**dict({'mode':mode,'depth_measured_valid':mode=='depth','position_valid':mode!='bearing_only',
                  'position_camera':N(x=1,y=2,z=10),'distance_m':10.25},**changes))
 def test_fresh_depth_estimate_and_bearing_are_distinct_valid_contracts(self):
  for mode in ('depth','size_estimate','bearing_only'):
   self.assertEqual(check_frame(self.frame(targets=[self.target(mode)])),[])
 def test_disabled_detector_cannot_publish_targets(self):
  self.assertIn('inactive_frame_contains_targets',check_frame(self.frame(detector_enabled=False,observations_fresh=False,targets=[self.target()])))
  self.assertEqual(check_frame(self.frame(detector_enabled=False,observations_fresh=False)),[])
 def test_old_data_must_not_be_marked_fresh(self):
  self.assertIn('invalid_freshness_flag',check_frame(self.frame(processing_latency_s=1.1)))
 def test_wrong_coordinates_and_invalid_numbers_are_reported(self):
  self.assertIn('unexpected_coordinate_frame',check_frame(self.frame(header=N(frame_id='body'))))
  self.assertIn('invalid_position_values',check_frame(self.frame(targets=[self.target(distance_m=float('nan'))])))
 def test_estimate_must_not_claim_measured_depth(self):
  self.assertIn('depth_mode_flag_mismatch',check_frame(self.frame(targets=[self.target(depth_measured_valid=True)])))

if __name__=='__main__':unittest.main()
