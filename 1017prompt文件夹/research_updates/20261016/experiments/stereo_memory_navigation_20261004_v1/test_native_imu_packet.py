from types import SimpleNamespace
import numpy as np
import pytest
from native_imu_packet import native_packet
class Reading(SimpleNamespace):
 @property
 def orientation(self):raise AssertionError('orientation truth read')
def make(**kw):
 d=dict(is_valid=True,time=1.,ang_vel_x=.1,ang_vel_y=.2,ang_vel_z=.3,lin_acc_x=0.,lin_acc_y=0.,lin_acc_z=9.81);d.update(kw);return Reading(**d)
def test_whitelist_and_does_not_read_orientation():
 p,d=native_packet(make(),.8,1.,.5,[.1,.2,.3]);assert d['qualified'];assert set(p)=={'time_s','gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s'};assert p['time_s']==.8;assert p['main_encoder_rad_s']==.5
@pytest.mark.parametrize('reading,raw,expected,reason',[(make(time=.98),1.,[.1,.2,.3],'time_mismatch'),(make(ang_vel_x=.11),1.,[.1,.2,.3],'gyro_axes_mismatch'),(make(lin_acc_z=float('nan')),1.,[.1,.2,.3],'nonfinite'),(make(is_valid=False),1.,[.1,.2,.3],'invalid')])
def test_unqualified_has_no_packet(reading,raw,expected,reason):
 p,d=native_packet(reading,.8,raw,.5,expected);assert p is None;assert d['reason']==reason
def test_copies_and_no_orientation_future_field():
 p,d=native_packet(make(),0.,1.,0.,[.1,.2,.3]);p['gyro_body_rad_s'][0]=999;assert d['gyro_body_rad_s'][0]==.1
