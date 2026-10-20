import numpy as np

def test_live_observer_on_off_cannot_call_or_mutate_policy():
 from types import SimpleNamespace
 from local_endpoint_event_live_v1 import observe_policy_endpoint
 class Policy:
  def __init__(self):
   self.calls=0;self.local_reference_version=1;self.tracker=SimpleNamespace(_context=object(),_reference_xy=np.array([[0.,0.],[1.,0.]]),_path_index=1,_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking',drive_model=SimpleNamespace(stopping_distance=lambda speed,direction:.25))
  def act(self,o,g):self.calls+=1;return np.array([.5,.1])
 o=SimpleNamespace(proprio_history=np.array([[.2,0]+[0]*15]),position_world_xy=np.array([.72,0]))
 off=Policy();on=Policy();before=on.tracker._reference_xy.copy();r=observe_policy_endpoint(on,o,2.,[5.,0]);assert on.calls==0 and on.tracker._path_index==1
 np.testing.assert_array_equal(on.tracker._reference_xy,before)
 np.testing.assert_array_equal(on.act(o,[5.,0]),off.act(o,[5.,0]));assert on.calls==off.calls==1
 assert r['created_live_before_policy_act'] and not r['actor_command_issued']
