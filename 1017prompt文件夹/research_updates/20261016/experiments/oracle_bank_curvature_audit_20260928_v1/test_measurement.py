import unittest,numpy as np
from scipy.spatial.transform import Rotation
from measurement import bank,geometry
class Tests(unittest.TestCase):
 def test_shared_axis_independent_shell_spin(self):
  r=Rotation.from_euler('ZYX',[[.7,.3,.2]]);s=r*Rotation.from_euler('Y',[2.]);q=r.as_quat()[:,[3,0,1,2]];z=s.as_quat()[:,[3,0,1,2]];np.testing.assert_allclose(bank(q),bank(z),atol=1e-12);self.assertAlmostEqual(bank(q)[0],np.arcsin(np.cos(.3)*np.sin(.2)))
 def test_circle_and_reverse(self):
  t=np.arange(1001)*.02
  for sign in [-1,1]:
   xy=2*np.column_stack([np.cos(sign*.2*t),np.sin(sign*.2*t)]);v,k=geometry(xy);np.testing.assert_allclose(v[30:-30],.4,atol=1e-5);np.testing.assert_allclose(k[30:-30],sign*.5,atol=.001)
 def test_stop_invalid(self):
  _,k=geometry(np.zeros((100,2)));self.assertTrue(np.isnan(k).all())
if __name__=='__main__':unittest.main()
