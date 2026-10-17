import pathlib
import sys
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from evaluate_depth_reference import evaluate


def sample(sid,index,z=4.,extent=.25,**values):
    return dict(session_id=sid,stamp_ns=1800000000000000000+index*1000000000,
                bbox_xyxy=[100,100,100+extent*1000/z,140],fx=1000.,fy=1000.,
                confidence=.9,quality_accepted=True,clipped=False,background_separated=True,
                depth_z_m=z,depth_valid_ratio=.8,depth_valid_pixels=200,depth_iqr_m=.01,
                prior_size_gate_used=False,model_id='old-model',camera_serial=42,
                image_width=1920,image_height=1080,image_id=str(index),**values)


class DepthReferenceEvaluationTests(unittest.TestCase):
    def test_fit_uses_confirmed_independent_references_and_holds_out_sessions(self):
        rows=[sample(sid,i,z=4+i/10) for sid in ('a','b') for i in range(20)]
        reviews={r['session_id']+'/'+r['image_id']:'target' for r in rows}
        result=evaluate(rows,reviews)
        self.assertAlmostEqual(result['candidate_effective_size_m'],.25)
        self.assertEqual(len(result['leave_one_session_out']),2)
        self.assertEqual(result['selected_seconds'],40)
        self.assertFalse(result['calibrated'],'offline stereo checks do not establish externally validated calibration')
        self.assertEqual(result['model_ids'],['old-model'])

    def test_old_prior_gate_estimates_and_unreviewed_rows_are_never_fit(self):
        rows=[sample('a',i) for i in range(20)]
        reviews={r['session_id']+'/'+r['image_id']:'target' for r in rows}
        for row in rows[:10]:row['prior_size_gate_used']=True
        rows[10]['depth_z_m']=float('nan')
        rows[11]['quality_accepted']=False
        reviews['a/12']='not_target'
        del reviews['a/13']
        result=evaluate(rows,reviews)
        self.assertEqual(result['selected_seconds'],6)
        self.assertIsNone(result['candidate_effective_size_m'])
        self.assertTrue(result['limitations'])

    def test_correlated_frames_do_not_count_as_independent_seconds(self):
        rows=[sample('a',0) for _ in range(100)]
        result=evaluate(rows,{'a/0':'target'})
        self.assertEqual(result['selected_seconds'],1)
        self.assertIsNone(result['candidate_effective_size_m'])

    def test_mixed_models_are_reported_without_one_combined_fit(self):
        rows=[sample('a',i) for i in range(20)]
        rows[-1]['model_id']='another-model'
        result=evaluate(rows,{'a/'+str(i):'target' for i in range(20)})
        self.assertIsNone(result['candidate_effective_size_m'])
        self.assertIn('mixed_model_or_camera_binding',result['limitations'])

    def test_user_exclusions_override_target_reviews_for_whole_scene_and_interval(self):
        rows=[sample(sid,i) for sid in ('a','b','person') for i in range(20)]
        for row in rows:row.update(segment_id=4,svo_position=int(row['image_id'])*5)
        reviews={r['session_id']+'/'+r['image_id']:'target' for r in rows}
        exclusions=[{'session_id':'person'},{'session_id':'b','segment_id':4,'start_svo_position':50}]
        result=evaluate(rows,reviews,exclusions)
        self.assertEqual(result['selected_seconds'],30)
        self.assertEqual(result['rejected_counts']['user_exclusion'],30)
        self.assertNotIn('person',result['by_session'])


if __name__=='__main__':unittest.main()
