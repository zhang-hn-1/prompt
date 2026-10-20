from types import SimpleNamespace as S
import numpy as np

def make(pending=None,braking='tracking',valid=True):
 from straight_pacing_consumer_v2 import with_straight_pacing
 class Base:
  def act(self,o,g):
   self.calls+=1;self.records.append(dict(request=[.5,.001]*5));return np.array([.5,.001]*5,np.float32)
 x=with_straight_pacing(Base)();x.calls=0;x.records=[];x.tracker=S(_active_drive_direction='forward',_pending_drive_direction=pending,_braking_state=braking,tracking_diagnostics=dict(valid=valid,source='tracking'))
 return x

def test_only_current_main_request_changes_and_single_act():
 x=make();o=S(quaternion_world_wxyz_history=np.array([[1.,0,0,0]]));a=x.act(o,None)
 assert x.calls==1 and np.all(a[::2]==1.) and np.allclose(a[1::2],.001)
 assert x.records[-1]['straight_pacing_applied'] and x.tracker._active_drive_direction=='forward'

def test_pending_stop_or_invalid_geometry_no_boost():
 for kw in [dict(pending='reverse'),dict(braking='goal_braking'),dict(valid=False)]:
  x=make(**kw);a=x.act(S(quaternion_world_wxyz_history=np.array([[1.,0,0,0]])),None)
  assert np.all(a[::2]==.5) and not x.records[-1]['straight_pacing_applied'] and x.calls==1

def test_bank_history_uses_current_sample_and_bounds_steering():
 x=make();q=np.array([[1.,0,0,0],[np.cos(.1),np.sin(.1),0,0]])
 a=x.act(S(quaternion_world_wxyz_history=q),None)
 assert np.all(a[::2]==.5) and 'measured_bank_outside_straight_screen' in x.records[-1]['straight_pacing_reasons']
 x=make();old=x.__class__.__mro__[1].act
 def steering(self,o,g):
  a=old(self,o,g);a[1::2]=.2;return a
 x.__class__.__mro__[1].act=steering
 a=x.act(S(quaternion_world_wxyz_history=np.array([[1.,0,0,0]])),None)
 assert np.all(a[::2]==.5) and np.allclose(a[1::2],.2)
 assert 'requested_bank_outside_straight_screen' in x.records[-1]['straight_pacing_reasons']

def test_current_invalid_audit_cannot_reuse_previous_valid_call():
 x=make();o=S(quaternion_world_wxyz_history=np.array([[1.,0,0,0]]));assert np.all(x.act(o,None)[::2]==1.)
 x.tracker.tracking_diagnostics={'valid':False,'source':'goal_braking'}
 assert np.all(x.act(o,None)[::2]==.5) and not x.records[-1]['straight_pacing_applied'] and x.calls==2
