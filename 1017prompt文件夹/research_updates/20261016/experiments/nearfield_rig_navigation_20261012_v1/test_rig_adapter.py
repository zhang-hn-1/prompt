import unittest
from unittest.mock import patch
import numpy as np
from rig_adapter import Rig, RigStereo, native_pose, paired, validate, ROTATION, FOCAL
from dense_source_points import point_packet
class RigChecks(unittest.TestCase):
 def setUp(self):
  self.rig=Rig('compact',.4,.1)
  self.record=dict(time_s=1.,available=True,live_before_next_action=True,position_world=[1,2,.4],rotation_world=np.eye(3),velocity_world_m_s=[0,0,0],flat_height_is_prior=True)
 def test_pose_projection_roundtrip(self):
  pose=native_pose(self.record,1.,self.rig);X=np.array([2.,0.,.1])
  for eye in ['left','right']:
   opt=(X-self.rig.translation(eye))@ROTATION
   T=pose[eye+'_camera_usd_world'];world=T[:3,3]+T[:3,:3]@np.diag([1,-1,-1])@opt
   np.testing.assert_allclose(world,X+[1,2,.4])
 def test_depth_scale(self):
  obj=RigStereo.__new__(RigStereo);obj.rig=self.rig
  with patch('rig_adapter.OriginalStereo.predict',return_value=dict(depth_m=np.array([3.,np.nan]),confidence=np.array([True,False]),disparity_px=np.array([117.,-1.]))):
   out=obj.predict(None,None,None,None)
  self.assertAlmostEqual(out['depth_m'][0],1.)
  self.assertTrue(np.isnan(out['depth_m'][1]));self.assertTrue(out['confidence'][0]);self.assertEqual(out['disparity_px'][0],117.)
 def test_correspondence_and_sigma(self):
  shape=(480,640);z=np.full(shape,np.nan);d=np.zeros(shape);confidence=np.zeros(shape,bool);z[350,320]=2.;d[350,320]=FOCAL*self.rig.baseline/2.;confidence[350,320]=True
  pred=dict(depth_m=z,confidence=confidence,disparity_native_px=d,frame=0,time_s=1.)
  p=native_pose(self.record,1.,self.rig);pose={k:p[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']}
  result=point_packet(pred,pose,np.zeros(shape,bool),np.zeros(shape,bool),baseline=self.rig.baseline)
  self.assertEqual(len(result['world']),1);self.assertGreater(result['sigma_m'][0],.03);self.assertLess(result['sigma_m'][0],.05)
  self.assertAlmostEqual(result['world'][0,0],3.4)
 def test_capture_rejects_old_calibration(self):
  r=dict(raw_time_s=1.)
  for eye in ['left','right']:r.update({eye+'_optical_to_base':self.rig.optical(eye),eye+'_rgb':np.zeros((480,640,3),np.uint8),eye+'_render_time_s':1.})
  self.assertTrue(validate(r,self.rig));r['left_optical_to_base'][0,3]=.2
  with self.assertRaises(ValueError):validate(r,self.rig)
 def test_stale_pose_rejected(self):
  with self.assertRaises(ValueError):native_pose(self.record,2.,self.rig)
if __name__=='__main__':unittest.main()
