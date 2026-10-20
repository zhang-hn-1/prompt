import unittest
import numpy as np
from reference_bearing_ambiguity_v1 import segment_distance, assess_reference

class TestReferenceBearingAmbiguity(unittest.TestCase):
    def test_crossing_segments_zero(self):
        self.assertAlmostEqual(segment_distance([0,0,0],[2,0,0],[1,-1,0],[1,1,0]),0.)
    def test_skew_and_parallel_segments(self):
        self.assertAlmostEqual(segment_distance([0,0,0],[2,0,0],[1,-1,3],[1,1,3]),3.)
        self.assertAlmostEqual(segment_distance([0,0,0],[2,0,0],[0,4,0],[2,4,0]),4.)
    def test_endpoints_and_degenerate_segment(self):
        self.assertAlmostEqual(segment_distance([0,0,0],[1,0,0],[2,1,0],[3,1,0]),np.sqrt(2))
        self.assertAlmostEqual(segment_distance([0,0,0],[0,0,0],[2,-1,0],[2,1,0]),2.)
    def record(self,t=1.):return dict(capture_time_s=t,world_ray_available=True,world_ray_origin=[0,0,0],world_ray_directions=[[1,0,0]])
    def test_interval_crossing_is_ambiguity_not_obstacle_or_free(self):
        r=assess_reference([self.record()],[[2,-1,0],[2,1,0]],1.)
        self.assertEqual(r['ambiguous_bearings'],1);self.assertFalse(r['measured_obstacle']);self.assertFalse(r['certified_free']);self.assertFalse(r['control_candidate_admitted'])
    def test_outside_interval_does_not_prove_safe(self):
        r=assess_reference([self.record()],[[4,-1,0],[4,1,0]],1.)
        self.assertEqual(r['ambiguous_bearings'],0);self.assertFalse(r['certified_free'])
    def test_future_history_and_invalid_reference_rejected(self):
        with self.assertRaises(ValueError):assess_reference([self.record(2.)],[[1,0,0]],1.)
        with self.assertRaises(ValueError):assess_reference([],[[np.nan,0,0]],1.)
    def test_uncertain_pose_is_unusable_without_world_geometry(self):
        r=self.record();r['world_ray_available']=False;r['world_ray_directions']=None
        out=assess_reference([r],[[1,0,0]],1.);self.assertEqual(out['pose_unusable_epochs'],1);self.assertFalse(out['certified_free'])

if __name__=='__main__':unittest.main()
