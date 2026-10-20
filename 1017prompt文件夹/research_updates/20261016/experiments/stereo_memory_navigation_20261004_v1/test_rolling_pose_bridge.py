import numpy as np
import pytest
from rolling_pose_bridge import RollingPoseBridge

def record(t,p=(0,0,0),R=None):return dict(time_s=t,position=np.array(p,float),rotation=np.eye(3) if R is None else R,velocity=np.array([.2,0,0]),shell_omega_gauge=np.array([0,.5,0]))

def test_literal_frozen_rig_and_one_initial_gauge():
 b=RollingPoseBridge([1,2,.45],np.eye(3));b.consume(record(0));r=b.at(0)
 np.testing.assert_allclose(r['root_position_world_m'],[1,2,.45]);np.testing.assert_allclose(r['left_camera_usd_world'][:3,3],[1.2,2.3,.5]);np.testing.assert_allclose(r['right_camera_usd_world'][:3,3],[1.2,1.7,.5]);np.testing.assert_allclose(r['left_camera_usd_world'][:3,:3],[[0,0,-1],[0,1,0],[1,0,0]])

def test_rotated_gauge_is_applied_to_position_velocity_and_rig():
 R=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
 b=RollingPoseBridge([1,2,0],R);b.consume(record(0,p=(.4,0,0)));r=b.at(0)
 np.testing.assert_allclose(r['root_position_world_m'],[1,2.4,0]);np.testing.assert_allclose(r['velocity_world_m_s'],[0,.2,0]);np.testing.assert_allclose(r['left_camera_usd_world'][:3,3],[.7,2.6,.05]);assert r['yaw_world_rad']==pytest.approx(np.pi/2)

def test_future_unavailable_missing_timestamp_and_copies():
 b=RollingPoseBridge([0,0,0],np.eye(3));r=record(0);b.consume(r);r['position'][:]=9
 with pytest.raises(ValueError):b.at(.1)
 with pytest.raises(ValueError):b.at(-1)
 p=b.at(0);p['root_position_world_m'][:]=4;np.testing.assert_array_equal(b.at(0)['root_position_world_m'],[0,0,0]);b.consume(record(.2));assert b.at(0)['time_s']==0
 with pytest.raises(ValueError):b.at(.1)

def test_invalid_fields_and_rotation_do_not_poison_history():
 b=RollingPoseBridge([0,0,0],np.eye(3))
 with pytest.raises(ValueError):b.consume({**record(0),'future_gt_position':[0,0,0]})
 with pytest.raises(ValueError):b.consume(record(0,R=np.ones((3,3))))
 b.consume(record(0))
 with pytest.raises(ValueError):b.consume(record(0))
 with pytest.raises(ValueError):RollingPoseBridge([0,0,np.nan],np.eye(3))
