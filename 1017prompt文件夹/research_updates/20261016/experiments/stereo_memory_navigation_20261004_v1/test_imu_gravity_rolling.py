import numpy as np
import pytest
from imu_gravity_rolling import GravityRolling

def packet(t,gyro=(0,0,0),acc=(0,0,9.81),encoder=0):return dict(time_s=t,gyro_body_rad_s=np.array(gyro),linear_acceleration_body_m_s2=np.array(acc),main_encoder_rad_s=encoder)

def test_stationary_gravity_reduces_physical_roll_drift_without_truth_input():
 m=GravityRolling(normal=(0,0,1));out=None
 for i in range(501):out=m.update(packet(i*.02,gyro=(.01,0,0)))
 angle=np.arccos(np.clip((np.trace(out['rotation'])-1)/2,-1,1));assert np.degrees(angle)<1

def test_zero_or_high_acceleration_rejects_gravity_correction():
 a=GravityRolling();b=GravityRolling()
 for i in range(101):x=a.update(packet(i*.02,gyro=(.02,0,0),acc=(0,0,0)));y=b.update(packet(i*.02,gyro=(.02,0,0),acc=(0,0,20)))
 np.testing.assert_allclose(x['rotation'],y['rotation']);assert not x['gravity_used'] and not y['gravity_used']

def test_pure_yaw_gyro_preserved_gravity_does_not_impose_heading():
 m=GravityRolling()
 for i in range(101):o=m.update(packet(i*.02,gyro=(0,0,.1)))
 assert np.arctan2(o['rotation'][1,0],o['rotation'][0,0])==pytest.approx(.2,abs=1e-10)

def test_noncollinear_integration_remains_so3_and_outputs_copies():
 m=GravityRolling()
 for i in range(101):o=m.update(packet(i*.02,gyro=(.1,-.2,.3),acc=(0,0,0)))
 np.testing.assert_allclose(o['rotation'].T@o['rotation'],np.eye(3),atol=1e-12);assert np.linalg.det(o['rotation'])==pytest.approx(1);o['rotation'][:]=99;assert np.max(m.R)<2

def test_unknown_or_nonfinite_packet_rejected_without_state_change():
 m=GravityRolling();o=m.update(packet(0));p=packet(.02);p['orientation']=np.eye(3)
 with pytest.raises(ValueError):m.update(p)
 bad=packet(.02);bad['linear_acceleration_body_m_s2'][1]=np.nan
 with pytest.raises(ValueError):m.update(bad)
 assert m.time==0;np.testing.assert_array_equal(m.R,o['rotation'])

def test_roll_tilt_feedback_sign_with_stationary_known_gravity():
 m=GravityRolling()
 for i in range(101):o=m.update(packet(i*.02,acc=(0,9.81*np.sin(.1),9.81*np.cos(.1))))
 roll=np.arctan2(o['rotation'][2,1],o['rotation'][2,2]);assert .075<roll<.1
