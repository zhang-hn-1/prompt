import unittest
import numpy as np
import cv2
from lk_surface_v1 import track_pairs
class LKTests(unittest.TestCase):
 def fixture(self):
  rng=np.random.default_rng(8);a=rng.integers(0,256,(128,128,3),dtype=np.uint8);b=cv2.warpAffine(a,np.array([[1.,0,3],[0,1.,-2]]),(128,128));return a,b,np.array([[45.,45.],[70.,70.]],np.float32)
 def test_signed_correspondence_translation(self):
  a,b,xy=self.fixture();r=track_pairs(a,b,xy,np.zeros((128,128),bool));self.assertEqual(len(r),2)
  for p in r:np.testing.assert_allclose(np.array(p['source_xy'])-xy[p['query_index']],[3,-2],atol=.15)
 def test_CAD_source_patch_excludes(self):
  a,b,xy=self.fixture();r=track_pairs(a,b,xy,np.ones((128,128),bool));self.assertEqual(r,[])
 def test_untextured_rejected(self):
  a=np.zeros((128,128,3),np.uint8);self.assertEqual(track_pairs(a,a,[[64,64]],np.zeros((128,128),bool)),[])
 def test_invalid_mask_rejected(self):
  a,b,xy=self.fixture()
  with self.assertRaises(ValueError):track_pairs(a,b,xy,np.zeros((4,4),bool))
if __name__=='__main__':unittest.main()
