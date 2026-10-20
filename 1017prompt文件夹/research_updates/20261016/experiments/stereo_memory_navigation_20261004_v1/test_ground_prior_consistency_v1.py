import unittest,numpy as np
from ground_prior_consistency_v1 import annotate_ground_prior
class Tests(unittest.TestCase):
 def test_sign_tags_without_dropping_or_admitting(self):
  p=dict(time_s=0.,world=np.array([[0,0,-.2],[0,0,0],[0,0,.2]]));s=annotate_ground_prior(p,0.,[0,0,1,0]);np.testing.assert_array_equal(s['prior_inconsistent'],[True,False,False]);np.testing.assert_array_equal(s['world'],p['world']);self.assertFalse(s['certified_free']);self.assertFalse(s['source_admitted']);self.assertTrue(s['ground_is_prior'])
 def test_plane_coordinates_not_assumed_world_z(self):
  s=annotate_ground_prior(dict(time_s=1.,world=np.array([[2.,0,0],[4.,0,0]])),1.,[1,0,0,-3]);np.testing.assert_array_equal(s['signed_plane_distance_m'],[-1,1])
 def test_future_clock_before_geometry_payload(self):
  class Bomb(dict):
   def __getitem__(self,k):
    if k=='world':raise AssertionError('future payload read')
    return super().__getitem__(k)
  with self.assertRaises(ValueError):annotate_ground_prior(Bomb(time_s=1.),0.,[0,0,1,0])
 def test_copy_and_invalid_plane_no_certificate(self):
  p=dict(time_s=0.,world=np.array([[0.,0,.1]]));s=annotate_ground_prior(p,0.,[0,0,1,0]);s['world'][0,2]=3;self.assertEqual(p['world'][0,2],.1)
  with self.assertRaises(ValueError):annotate_ground_prior(p,0.,[0,0,0,0])
if __name__=='__main__':unittest.main()
