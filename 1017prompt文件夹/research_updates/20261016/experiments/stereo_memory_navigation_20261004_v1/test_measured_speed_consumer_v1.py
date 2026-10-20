import numpy as np
from types import SimpleNamespace
from measured_speed_consumer_v1 import with_measured_speed
class Base:
 def __init__(self):
  self.calls=0;self.records=[];self.tracker=SimpleNamespace(_reference_xy=np.array([[0.,0.],[3.,0.]]),_tracking_audit=dict(valid=True,source='tracking'),_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking')
 def act(self,o,g):
  self.calls+=1;a=np.array([.5,.04]*5,np.float32);self.records.append(dict(request=a.tolist()));return a

def obs(speed=.2,x=0.):return SimpleNamespace(position_world_xy=np.array([x,0.]),proprio_history=np.tile(np.array([speed,0.,0.]),(5,1)))
def test_feedback_changes_main_once_aux_exact_and_responds_current_speed():
 p=with_measured_speed(Base)();a=p.act(obs(.2),np.array([4.,0.]));b=p.act(obs(.34),np.array([4.,0.]));assert p.calls==2
 assert .5<b[0]<a[0]<=1 and np.array_equal(a[1::2],np.full(5,np.float32(.04)))
 assert p.records[-1]['measured_speed_control_applied']

def test_local_terminal_and_braking_restore_original():
 p=with_measured_speed(Base)();a=p.act(obs(x=2.5),np.array([4.,0.]));assert np.all(a[::2]==.5)
 p.tracker._braking_state='braking';b=p.act(obs(),np.array([4.,0.]));assert np.all(b[::2]==.5)
 assert p.calls==2 and not p.records[-1]['measured_speed_control_applied']

def test_same_output_at_speed_saturation_is_not_behavioral_intervention():
 p=with_measured_speed(Base)();a=p.act(obs(1.),np.array([4.,0.]))
 assert np.all(a[::2]==.5) and not p.records[-1]['measured_speed_control_applied']
