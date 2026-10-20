from types import SimpleNamespace as N
import numpy as np
from measured_speed_consumer_v1 import with_measured_speed

def fixture():
 class Base:
  def __init__(self):
   self.time=10.;self.calls=0;self.records=[];self.plan=N(status='success',direction_changes=0,waypoints=[N(x_m=0.,y_m=0.,drive_direction='forward'),N(x_m=2.,y_m=0.,drive_direction='forward')]);self.plan_meta=dict(status='success',route_xy=[[0.,0.],[2.,0.],[8.,0.]]);self.reference_decision=dict(decision='retain',measured_time_s=10.,certified_safe=False);self.tracker=N(_tracking_audit=dict(valid=True,source='tracking'),_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking',_reference_xy=np.array([[0.,0.],[2.,0.]]))
  def act(self,o,g):
   self.calls+=1;a=np.tile([.5,.1],5).astype(np.float32);self.records.append(dict(request=a.tolist()));return a
 o=N(position_world_xy=np.array([1.4,0.]),yaw_world_rad=0.,proprio_history=np.tile([.2,0.],(5,1)))
 return Base,o,np.array([6.6,0.])

def test_current_intermediate_horizon_bypasses_only_local_reserve_with_one_act():
 from rolling_endpoint_qualification_v1 import with_rolling_endpoint
 Base,o,g=fixture();old=with_measured_speed(Base)();new=with_rolling_endpoint(Base)();a=old.act(o,g);b=new.act(o,g)
 assert np.all(a[::2]==.5) and np.all(b[::2]>.5)
 assert old.calls==new.calls==1 and np.array_equal(a[1::2],b[1::2])
 assert new.records[-1]['rolling_local_reserve_bypassed']
 assert not new.records[-1]['rolling_endpoint_context']['certified_safe']

def test_final_fixed_reverse_missing_future_or_globalnear_preserves_old_action():
 from rolling_endpoint_qualification_v1 import with_rolling_endpoint
 for case in ('final','fixed','reverse','missing','future','globalnear','plan_mismatch'):
  Base,o,g=fixture();p=with_rolling_endpoint(Base)()
  if case=='final':p.plan_meta['route_xy']=[[0.,0.],[2.,0.]];g=np.array([.6,0.])
  if case=='fixed':p.reference_decision['shared_full_frozen_reference']=True
  if case=='reverse':p.plan.waypoints[0].drive_direction='reverse'
  if case=='missing':p.plan_meta={}
  if case=='future':p.reference_decision['measured_time_s']=10.1
  if case=='globalnear':g=np.array([.5,0.])
  if case=='plan_mismatch':p.plan.waypoints[-1].x_m=3.
  action=p.act(o,g);assert np.all(action[::2]==.5),case
  assert not p.records[-1]['rolling_local_reserve_bypassed'] and p.calls==1

def test_qualification_without_actual_request_difference_is_not_intervention():
 from rolling_endpoint_qualification_v1 import with_rolling_endpoint
 Base,o,g=fixture();o.proprio_history[:]=[1.,0.];p=with_rolling_endpoint(Base)();a=p.act(o,g)
 assert p.records[-1]['rolling_local_reserve_bypassed']
 assert np.all(a[::2]==.5) and not p.records[-1]['rolling_control_applied']

def test_future_or_stale_boundary_does_not_read_geometry_payload():
 from rolling_endpoint_qualification_v1 import with_rolling_endpoint
 class Poison:
  def __array__(self,*args,**kwargs):raise AssertionError('future/stale geometry read')
 for boundary_time in (10.1,9.):
  Base,o,g=fixture();p=with_rolling_endpoint(Base)();p.reference_decision['measured_time_s']=boundary_time;p.plan_meta['route_xy']=Poison()
  a=p.act(o,g)
  assert np.all(a[::2]==.5) and not p.records[-1]['rolling_control_applied']
  assert p.records[-1]['rolling_endpoint_context']['reasons']==['not_current_past_boundary']
