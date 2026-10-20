import importlib.util
import numpy as np
import pytest

def implementation():
 assert importlib.util.find_spec('native_pose_consumer_v1') is not None, 'current-only native pose consumer missing'
 from native_pose_consumer_v1 import current_native_pose
 return current_native_pose

def record(t=1.):
 return dict(time_s=t,available=True,position_world=[2,3,.4],rotation_world=np.eye(3).tolist(),velocity_world_m_s=[.1,0,0],live_before_next_action=True,flat_height_is_prior=True)

def test_registration_and_navigation_share_current_estimate():
 pose=implementation()(record(),1.)
 np.testing.assert_allclose(pose['root_position_world_m'],[2,3,.4])
 np.testing.assert_allclose(pose['left_camera_usd_world'][:3,3],[2.2,3.3,.45])
 np.testing.assert_allclose(pose['right_camera_usd_world'][:3,3],[2.2,2.7,.45])
 assert pose['initial_gauge_privileged'] and pose['pose_estimated']

def test_future_or_stale_pose_cannot_be_consumed():
 consumer=implementation()
 for t in (.9,1.1):
  with pytest.raises(ValueError): consumer(record(t),1.)

def test_unavailable_or_nonlive_pose_stops_instead_of_truth_fallback():
 consumer=implementation()
 for field in ('available','live_before_next_action'):
  r=record();r[field]=False
  with pytest.raises(ValueError):consumer(r,1.)

def test_unrelated_truth_fields_do_not_change_output():
 consumer=implementation();r=record();a=consumer(r,1.)
 r.update(future_truth=[999]*3,root_truth=[float('nan')]*3)
 b=consumer(r,1.)
 for key in ('root_position_world_m','root_rotation_world','velocity_world_m_s','left_camera_usd_world','right_camera_usd_world'):
  np.testing.assert_array_equal(a[key],b[key])

def test_nonfinite_record_clock_cannot_be_relabelled_current():
 consumer=implementation()
 for t in (float('nan'),float('inf'),-float('inf')):
  with pytest.raises(ValueError):consumer(record(t),1.)
