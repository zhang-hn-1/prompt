from types import SimpleNamespace
import numpy as np
from forward_course_geometry_adapter_v1 import with_forward_course_geometry
class Parent:
 def __init__(self):
  self.records=[];self._packet=None;self.calls=0;self.lookahead_calls=0;self.compose_calls=0
  self.tracker=SimpleNamespace(_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking')
  self.tracker._lookahead_target=self.target;self.tracker._compose_tracking_curvature=self.compose
 def target(self,x):self.lookahead_calls+=1;return np.array([1.,0.])
 def compose(self,o,**kw):self.compose_calls+=1;return kw['geometric_curvature_1_m'],{}
 def act(self,o,g):
  self.calls+=1;self.tracker._lookahead_target(.5);k,m=self.tracker._compose_tracking_curvature(o,waypoint=SimpleNamespace(drive_direction='forward'),geometric_curvature_1_m=0.);self.records.append({});return np.array([.5,k]*5,np.float32)
def test_single_parent_call_and_current_target_only():
 for enabled in [False,True]:
  p=with_forward_course_geometry(Parent,enabled=enabled)();o=SimpleNamespace(proprio_history=np.array([[.2,.2]+[0.]*10+[.5]+[0.]*4]),position_world_xy=np.zeros(2))
  for i in range(5):p._packet={'time_s':i*.1};out=p.act(o,None)
  assert p.calls==p.lookahead_calls==p.compose_calls==5
  assert (out[1]!=0)==enabled
  np.testing.assert_array_equal(out[::2],np.ones(5,np.float32)*.5)
  assert p.records[-1]['forward_course_geometry']['used']==enabled
  assert p.tracker._compose_tracking_curvature==p.compose
