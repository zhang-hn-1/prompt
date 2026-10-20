import numpy as np
import pytest
from rolling_pose_bridge import RollingPoseBridge
from flat_ground_pose_bridge import FlatGroundPoseBridge

def record(t=0):return dict(time_s=t,position=[.2,-.1,.03],rotation=np.eye(3),velocity=[.1,.2,.03],shell_omega_gauge=[0,0,0])
def test_one_floor_prior_keeps_xy_rotation_and_camera_extrinsics():
 p=np.array([1.,2.,.446]);old=RollingPoseBridge(p,np.eye(3));new=FlatGroundPoseBridge(p,np.eye(3));old.consume(record());new.consume(record());a=old.at(0);b=new.at(0);np.testing.assert_equal(b['root_position_world_m'][:2],a['root_position_world_m'][:2]);np.testing.assert_equal(b['root_rotation_world'],a['root_rotation_world']);assert b['root_position_world_m'][2]==.4000000059604645
 for eye in ['left','right']:
  np.testing.assert_equal(b[eye+'_camera_usd_world'][:3,:3],a[eye+'_camera_usd_world'][:3,:3]);np.testing.assert_equal(b[eye+'_camera_usd_world'][:2,3],a[eye+'_camera_usd_world'][:2,3]);assert abs(b[eye+'_camera_usd_world'][2,3]-(.4000000059604645+.05))<1e-12
 np.testing.assert_equal(p,[1,2,.446])
def test_prior_is_not_measurement_or_full_safety():
 b=FlatGroundPoseBridge([0,0,.45],np.eye(3));b.consume(record());r=b.at(0);assert r['flat_ground_height_prior'] and not r['height_prior_is_observation'] and not r['full_3d_certified'];np.testing.assert_equal(r['velocity_world_m_s'],[.1,.2,0]);assert r['unconstrained_root_world_z_m']==.48

def test_future_lookup_rejected():
 b=FlatGroundPoseBridge([0,0,.45],np.eye(3));b.consume(record())
 with pytest.raises(ValueError):b.at(1)
def test_invalid_prior_rejected():
 for args in [dict(floor_z_m=np.nan),dict(radius_m=0),dict(radius_m=np.inf)]:
  with pytest.raises(ValueError):FlatGroundPoseBridge([0,0,.45],np.eye(3),**args)
