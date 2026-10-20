import copy
import numpy as np
import pytest
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row

def test_literal_current_sensor_emulation_does_not_read_gt_position_or_velocity():
    class Guard(dict):
        def __getitem__(self,k):
            if k not in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']:raise AssertionError(k)
            return super().__getitem__(k)
    d=Guard(simulation_time_s=[0.],root_quaternion_world_wxyz=[[1,0,0,0]],angular_velocity_world_rad_s=[[0,0,0]],joint_velocity_rad_s=[[.5,0,0]])
    sensor,n=emulate_sensor_row(d);assert set(sensor)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'}
    model=OnlineRollingShadow(.4);assert model.consume(sensor,n) is None
    np.testing.assert_allclose(model.records[0]['velocity'],[.2,0,0])

def test_shadow_on_off_parent_diagnostic_and_requests_are_identical_and_immutable():
    rows=[dict(time_s=i*.02,gyro_body_rad_s=np.zeros(3),main_encoder_rad_s=.5) for i in range(10)]
    requests=np.array([[.5,-.5235987756]]*10);before=requests.copy();snapshot=copy.deepcopy(rows)
    shadow=OnlineRollingShadow(.4)
    for row in rows:assert shadow.consume(row,[0,0,1]) is None
    np.testing.assert_array_equal(requests,before)
    for a,b in zip(rows,snapshot):
        for k in a:np.testing.assert_equal(a[k],b[k])
    np.testing.assert_allclose(shadow.records[-1]['position'],[.036,0,0])

def test_future_sensor_stream_is_not_delivered_and_prefix_is_exact():
    prefix=[dict(time_s=i*.02,gyro_body_rad_s=np.array([0,0,.1]),main_encoder_rad_s=.5) for i in range(10)]
    a=OnlineRollingShadow(.4);b=OnlineRollingShadow(.4)
    for p in prefix:a.consume(p,[0,0,1]);b.consume(p,[0,0,1])
    assert a.records==b.records
    bad={**prefix[-1],'time_s':.3,'gt_position':[1,2,3]}
    with pytest.raises(ValueError):a.consume(bad)
    assert a.records==b.records

def test_invalid_first_sensor_does_not_latch_bad_initial_calibration():
    shadow=OnlineRollingShadow(.4)
    with pytest.raises(ValueError):shadow.consume(dict(time_s=np.nan,gyro_body_rad_s=np.zeros(3),main_encoder_rad_s=.5),[1,0,0])
    assert shadow.model is None and shadow.initial_normal is None and not shadow.records
    shadow.consume(dict(time_s=0.,gyro_body_rad_s=np.zeros(3),main_encoder_rad_s=.5),[0,0,1])
    np.testing.assert_allclose(shadow.records[0]['velocity'],[.2,0,0])
