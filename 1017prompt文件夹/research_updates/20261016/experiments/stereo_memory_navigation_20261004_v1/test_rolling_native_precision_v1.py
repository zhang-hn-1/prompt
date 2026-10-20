import unittest
import numpy as np
from native_pose_consumer_v1 import current_native_pose
class NativePrecision(unittest.TestCase):
 def test_preserve_native_float64_not_truth_quantized(self):
  from rolling_native_input_audit_v1 import current_native_inputs
  v=[.3141592653589793,.1287182937461111,0.]
  r=dict(time_s=4.,available=True,live_before_next_action=True,position_world=[9,8,.4],rotation_world=np.eye(3).tolist(),velocity_world_m_s=v,flat_height_is_prior=True)
  packet=dict(time_s=4.,gyro_body_rad_s=[.1,.2,.3])
  p,g,s=current_native_inputs(r,packet,4.)
  self.assertEqual(s,float(np.linalg.norm(np.array(v)[:2])))
  self.assertNotEqual(s,float(np.linalg.norm(np.array(v,np.float32).astype(float)[:2])))
  np.testing.assert_array_equal(p['velocity_world_m_s'],v);np.testing.assert_array_equal(g,packet['gyro_body_rad_s'])
 def test_stale_missing_future_rejected(self):
  from rolling_native_input_audit_v1 import current_native_inputs
  r=dict(time_s=4.,available=True,live_before_next_action=True,position_world=[9,8,.4],rotation_world=np.eye(3).tolist(),velocity_world_m_s=[.3,0,0],flat_height_is_prior=True)
  packet=dict(time_s=4.,gyro_body_rad_s=[0,0,0])
  for bad in (dict(r,time_s=4.1),dict(r,time_s=3.9),dict(r,available=False)):
   with self.assertRaises(ValueError):current_native_inputs(bad,packet,4.)
  for bad in (None,dict(packet,time_s=4.1),dict(packet,time_s=3.9)):
   with self.assertRaises(ValueError):current_native_inputs(r,bad,4.)
if __name__=='__main__':unittest.main()

class NativeObservationPrecision(unittest.TestCase):
 def test_actual_policy_observation_quantization_reconstructed(self):
  from benchmark.policies import PolicyObservation
  from estimated_navigation_observation import EstimatedNavigationObservation
  from assess_rolling_endpoint_native_e0_v1 import reconstruct_current_speed
  v=np.array([.3141592653589793,.1287182937461111,0.])
  o=PolicyObservation(stereo_left_rgb=np.zeros((480,640,3),np.uint8),stereo_right_rgb=np.zeros((480,640,3),np.uint8),proprio_history=np.zeros((5,17)),action_history=np.zeros((5,10)),position_world_xy=np.zeros(2),yaw_world_rad=0.,quaternion_world_wxyz_history=np.tile([1,0,0,0],(5,1)))
  pose=dict(time_s=4.,root_position_world_m=np.array([9,8,.4]),root_rotation_world=np.eye(3),velocity_world_m_s=v)
  actual,_=EstimatedNavigationObservation().build(o,pose,np.zeros(3),[10,8,.4])
  self.assertEqual(actual.proprio_history.dtype,np.dtype(np.float32))
  self.assertEqual(reconstruct_current_speed(v),float(np.linalg.norm(np.asarray(actual.proprio_history[-1,:2],float))))
