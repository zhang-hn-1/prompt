from types import SimpleNamespace
import pytest
from imu_interval_packet import interval_packet
class Reading(SimpleNamespace):
 @property
 def orientation(self):raise AssertionError('orientation read')
def reading(**kw):
 d=dict(is_valid=True,time=1.,ang_vel_x=.1,ang_vel_y=.2,ang_vel_z=.3,lin_acc_x=0.,lin_acc_y=0.,lin_acc_z=9.81);d.update(kw);return Reading(**d)
def test_actual_begin_timestamp_current_delivery():
 p,d=interval_packet(reading(),1.,1.02,.8,.5);assert p['time_s']==.8 and d['qualified'];assert set(p)=={'time_s','gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s'}
@pytest.mark.parametrize('r,begin,end,reason',[(reading(time=.98),1.,1.02,'stamp_outside_begin'),(reading(),1.,1.04,'wrong_interval'),(reading(is_valid=False),1.,1.02,'invalid'),(reading(lin_acc_x=float('nan')),1.,1.02,'nonfinite')])
def test_rejects_wrong_interval_or_sample(r,begin,end,reason):
 p,d=interval_packet(r,begin,end,.8,.5);assert p is None and d['reason']==reason
def test_no_orientation_or_gt_access_and_copies():
 r=reading();p,d=interval_packet(r,1.,1.02,.8,.5);p['gyro_body_rad_s'][0]=999;assert r.ang_vel_x==.1 and d['gyro_body_rad_s'][0]==.1
