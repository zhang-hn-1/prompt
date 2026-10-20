import numpy as np
import pytest
from rolling_odometry import RollingOdometry

def test_actual_shell_encoder_forward_speed_without_reading_requested_command():
    model=RollingOdometry(radius=.4);model.update(0,[0,0,0],.5)
    out=model.update(1,[0,0,0],.5)
    np.testing.assert_allclose(out['velocity'],[.2,0,0]);np.testing.assert_allclose(out['position'],[.2,0,0])

def test_world_normal_spin_has_zero_translation_but_changes_attitude():
    model=RollingOdometry(radius=.4);model.update(0,[0,0,1],0);out=model.update(.1,[0,0,1],0)
    np.testing.assert_allclose(out['position'],0)
    np.testing.assert_allclose(out['rotation'],[[np.cos(.1),-np.sin(.1),0],[np.sin(.1),np.cos(.1),0],[0,0,1]],atol=1e-12)

def test_initial_gravity_normal_changes_rolling_projection_without_pose_truth():
    model=RollingOdometry(radius=.4,normal=(0,1,0));out=model.update(0,[0,0,0],.5)
    np.testing.assert_allclose(out['velocity'],0)

def test_invalid_time_or_sensor_cannot_mutate_live_estimator():
    model=RollingOdometry();first=model.update(1,[0,0,0],.5)
    for t,g,m in [(1,[0,0,0],.5),(.9,[0,0,0],.5),(2,[np.nan,0,0],.5),(2,[0,0,0],np.inf)]:
        with pytest.raises(ValueError):model.update(t,g,m)
        assert model.time==1;np.testing.assert_equal(model.position,first['position'])

def test_output_and_sensor_arrays_do_not_alias_state_and_future_is_unread():
    model=RollingOdometry(radius=.4);g=np.zeros(3);o=model.update(0,g,.5);o['rotation'][:]=17;o['position'][:]=42
    np.testing.assert_equal(g,0);out=model.update(.1,g,.5)
    np.testing.assert_allclose(out['rotation'],np.eye(3));np.testing.assert_allclose(out['position'],[.02,0,0])
