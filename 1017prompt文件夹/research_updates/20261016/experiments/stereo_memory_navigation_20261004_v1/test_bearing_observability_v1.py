import unittest
import numpy as np
from bearing_observability_v1 import build_context, assess_view

K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
def pose():
    a=np.eye(4);a[:3,:3]=np.diag([1.,-1.,-1.]);return a

class TestBearingObservability(unittest.TestCase):
    def context(self):return build_context([[319.5,239.5]],pose(),K,10.,10)
    def test_transverse_motion_resolves_center_ray(self):
        b=pose();b[0,3]=.3
        r=assess_view(self.context(),b,[1.,2.,3.])
        self.assertTrue(all(x['sigma_pass'] for x in r['rows']))
        self.assertTrue(all(x['sigma_m']<.06 for x in r['rows']))
    def test_near_ray_can_leave_field_of_view_despite_large_parallax(self):
        b=pose();b[0,3]=.3;r=assess_view(self.context(),b,[.5])
        self.assertFalse(r['rows'][0]['in_image']);self.assertFalse(r['rows'][0]['sigma_pass'])
    def test_optical_axis_motion_does_not_resolve_center_ray(self):
        b=pose();b[2,3]=.3
        self.assertFalse(any(x['sigma_pass'] for x in assess_view(self.context(),b,[1.,2.,3.])['rows']))
    def test_rotation_only_cannot_triangulate_depth(self):
        b=pose();r=np.array([[np.cos(.1),0,np.sin(.1)],[0,1,0],[-np.sin(.1),0,np.cos(.1)]])
        b[:3,:3]=r@b[:3,:3]
        self.assertFalse(any(x['sigma_pass'] for x in assess_view(self.context(),b,[1.,2.])['rows']))
    def test_prior_never_becomes_observed_geometry(self):
        b=pose();b[0,3]=.3;r=assess_view(self.context(),b,[1.])
        self.assertFalse(r['observed_geometry']);self.assertFalse(r['certified_free'])
        self.assertEqual(r['role'],'conditional_geometric_prediction')
    def test_bad_calibration_or_hypothesis_rejected(self):
        with self.assertRaises(ValueError):build_context([[np.nan,2]],pose(),K,10.,10)
        with self.assertRaises(ValueError):build_context([[1,2]],pose(),K,np.inf,10)
        with self.assertRaises(ValueError):assess_view(self.context(),pose(),[-1.])

if __name__=='__main__':unittest.main()
