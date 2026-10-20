import numpy as np
from benchmark.policies import PolicyObservation
from assess_local_endpoint_live_closed_v2 import navigation_velocity

def test_reconstruction_matches_real_observation_dtype_conversion():
 raw=np.array([.014077583479123,-.002502000198321,0.],dtype=np.float64)
 row=np.r_[raw,np.zeros(14)][None]
 o=PolicyObservation(stereo_left_rgb=np.zeros((480,640,3),np.uint8),stereo_right_rgb=np.zeros((480,640,3),np.uint8),proprio_history=row,action_history=np.zeros((1,10)),position_world_xy=np.array([0.,0.]),yaw_world_rad=0.)
 expected=np.asarray(o.proprio_history[-1],float)[:2]
 np.testing.assert_array_equal(navigation_velocity(raw),expected)
 assert np.any(expected!=raw[:2])
