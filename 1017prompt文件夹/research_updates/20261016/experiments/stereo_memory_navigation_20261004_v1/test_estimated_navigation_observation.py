from types import SimpleNamespace
import numpy as np
import pytest
from estimated_navigation_observation import EstimatedNavigationObservation

def obs():
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('GT read')
 o=SimpleNamespace(stereo_left_rgb=np.zeros((2,2,3),np.uint8),stereo_right_rgb=np.ones((2,2,3),np.uint8),action_history=np.zeros((5,10)),proprio_history=np.ones((5,17)),position_world_xy=Poison(),yaw_world_rad=Poison(),quaternion_world_wxyz_history=Poison())
 o.proprio_history[:,:9]=np.nan;return o

def pose(t=0):return dict(time_s=t,root_position_world_m=np.array([1.,2.,.45]),root_rotation_world=np.array([[0,-1,0],[1,0,0],[0,0,1.]]),velocity_world_m_s=np.array([0,.2,0]),yaw_world_rad=np.pi/2)

def test_does_not_read_truth_kinematics_and_literal_estimated_goal():
 a=EstimatedNavigationObservation();o,g=a.build(obs(),pose(),[0,0,.1],[1,4,.45])
 np.testing.assert_allclose(o.position_world_xy,[1,2]);np.testing.assert_allclose(g,[2,0],atol=1e-14);assert o.yaw_world_rad==np.pi/2
 np.testing.assert_allclose(o.proprio_history[0,:9],[0,.2,0,0,0,.1,0,0,-1]);np.testing.assert_array_equal(o.proprio_history[:,9:],1)
 np.testing.assert_allclose(o.quaternion_world_wxyz_history,np.tile([np.sqrt(.5),0,0,np.sqrt(.5)],(5,1)))

def test_causal_history_no_reset_or_parent_mutation():
 a=EstimatedNavigationObservation();original=obs();before=original.proprio_history.copy();a.build(original,pose(),[0,0,0],[1,4,.45]);p=pose(.1);p['velocity_world_m_s']=np.array([0,.4,0]);o,_=a.build(original,p,[0,0,.1],[1,4,.45]);np.testing.assert_allclose(o.proprio_history[:4,1],.2);assert o.proprio_history[-1,1]==pytest.approx(.4);np.testing.assert_array_equal(original.proprio_history,before)
 with pytest.raises(ValueError):a.build(original,pose(),[0,0,0],[1,4,.45])
