import unittest
import numpy as np
from census import CensusStereo, census_codes

class CensusChecks(unittest.TestCase):
 def pair(self):
  rng=np.random.default_rng(42);l=rng.integers(10,110,(32,52),dtype=np.uint8);r=rng.integers(10,110,l.shape,dtype=np.uint8);r[:,:-7]=l[:,7:]
  return l,r
 def test_known_disparity(self):
  l,r=self.pair();v=CensusStereo('cpu').match_rectified(l,r,np.zeros_like(l,bool),np.zeros_like(r,bool));m=v['valid'];self.assertGreater(m[6:-6,13:-6].mean(),.95);self.assertLess(np.max(np.abs(v['disparity'][m]-7)),.15)
 def test_brightness_order(self):
  l,r=self.pair();model=CensusStereo('cpu');a=model.match_rectified(l,r,l<0,r<0);b=model.match_rectified(l,2*r+10,l<0,r<0);np.testing.assert_array_equal(a['valid'],b['valid']);np.testing.assert_allclose(a['disparity'],b['disparity'],equal_nan=True)
 def test_constant_unknown(self):
  l=np.ones((24,36),np.uint8)*80;v=CensusStereo('cpu').match_rectified(l,l,l<0,l<0);self.assertFalse(v['valid'].any())
 def test_cad_complete_support(self):
  l,r=self.pair();m=l<0;m[15,26]=True;v=CensusStereo('cpu').match_rectified(l,r,m,r<0);self.assertFalse(v['valid'][11:20,22:31].any());self.assertFalse(v['valid'][:4].any());self.assertFalse(v['valid'][:,:4].any())
 def test_right_mask(self):
  l,r=self.pair();m=r<0;m[15,19]=True;v=CensusStereo('cpu').match_rectified(l,r,l<0,m);self.assertFalse(v['valid'][15,26])
 def test_native_rotation(self):
  l,r=self.pair();l=np.rot90(l,-1);r=np.rot90(r,-1);l=np.repeat(l[:,:,None],3,2);r=np.repeat(r[:,:,None],3,2);v=CensusStereo('cpu',focal=14,baseline=1).predict(l,r,np.zeros(l.shape[:2],bool),np.zeros(r.shape[:2],bool));self.assertEqual(v['depth_m'].shape,l.shape[:2]);m=v['confidence'];self.assertGreater(m.sum(),100);self.assertLess(np.max(np.abs(v['depth_m'][m]-2)),.05)
 def test_descriptor_exact(self):
  l,r=self.pair();a=census_codes(l);y,x=12,19;bits=0;k=0
  for dy in range(-3,4):
   for dx in range(-3,4):
    if dy==dx==0:continue
    bits |= int(l[y+dy,x+dx]<l[y,x])<<k;k+=1
  self.assertEqual(int(a[y,x]),bits)
 def test_invalid_shape(self):
  l,r=self.pair()
  with self.assertRaises(ValueError): CensusStereo('cpu').match_rectified(l,r[:,:-1],l<0,r<0)
if __name__=='__main__':unittest.main()
