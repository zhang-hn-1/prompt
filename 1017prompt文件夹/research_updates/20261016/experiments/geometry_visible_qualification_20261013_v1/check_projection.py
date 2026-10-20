import unittest,numpy as np
from projection import project_world,backproject,root_from_camera
class Checks(unittest.TestCase):
 def test_roundtrip(self):
  T=np.eye(4);T[:3,3]=[.4,.3,.7];p=backproject(np.array([319.5,400.]),np.array([239.5,270.]),np.array([1.,2.]),T);u,v,z=project_world(p,T);np.testing.assert_allclose(u,[319.5,400]);np.testing.assert_allclose(v,[239.5,270]);np.testing.assert_allclose(z,[1,2])
 def test_behind(self):
  _,_,z=project_world(np.array([[0,0,1.]]),np.eye(4));self.assertLess(z[0],0)
 def test_stereo_sign(self):
  R=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
  T=np.eye(4);T[:3,:3]=R@np.diag([1,-1,-1]);T[:3,3]=[.4,.3,.05];Q=T.copy();Q[1,3]=-.3
  p=backproject(np.array([319.5]),np.array([239.5]),np.array([2.]),T);u,v,z=project_world(p,Q);self.assertAlmostEqual(u[0],319.5);self.assertAlmostEqual(v[0],239.5-584.8888888888889*.6/2)
 def test_root(self):
  R=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
  T=np.eye(4);T[:3,:3]=R@np.diag([1,-1,-1]);T[:3,3]=[1.4,2.3,.75];r,p=root_from_camera(T);np.testing.assert_allclose(r,np.eye(3));np.testing.assert_allclose(p,[1,2,.7])
if __name__=='__main__':unittest.main()
