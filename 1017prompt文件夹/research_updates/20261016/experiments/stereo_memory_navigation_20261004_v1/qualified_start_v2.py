"""One initial qualification consumed at both original timing gates; no clock advance."""
import copy,math
import numpy as np
from guarded_reference import GuardedReferencePolicy
from depth_local_curve import finite
from provenance_grid import ProvenanceGrid
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from current_forward_intent_qualification_v1 import with_current_forward_intent
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from command_domain_cap import with_command_domain_cap
from rolling_endpoint_qualification_v1 import with_rolling_endpoint

class QualifiedStartGuarded(GuardedReferencePolicy):
 def reset(self,context):
  self.initial_start_qualified_time_s=None
  super().reset(context)
 def qualify_initial_start(self,time_s):
  t=finite(time_s)
  if t<0:raise ValueError('invalid initial qualification')
  if self.initial_start_qualified_time_s is not None and t!=self.initial_start_qualified_time_s:raise ValueError('initial qualification is latched')
  self.initial_start_qualified_time_s=t
 def initial_start_eligible(self,time_s):
  q=self.initial_start_qualified_time_s
  if q is not None and (not math.isfinite(q) or q<0 or q>time_s+1e-9):raise ValueError('future or invalid initial qualification')
  return time_s>=6.-1e-6 or (self.initial_start_candidate_enabled and q is not None)
 def act(self,observation,goal):
  # Same frozen base transaction, with only the planning-boundary consumer changed.
  if self._packet is None:raise RuntimeError('current packet required')
  packet=self._packet;t=packet['time_s'];settled=self.initial_start_eligible(t);xy=np.asarray(observation.position_world_xy,float);yaw=finite(observation.yaw_world_rad);goal=np.asarray(goal,float)
  if np.linalg.norm(packet['root_xyz'][:2]-xy)>1e-5:raise ValueError('current state/packet mismatch')
  if goal.shape!=(2,) or not np.isfinite(goal).all():raise ValueError('invalid goal')
  if self.grid is None:self.grid=ProvenanceGrid(xy-20.,memory_mode='history')
  self.grid.update_points(packet['points_world'],packet['root_xyz'],t)
  c,s=math.cos(yaw),math.sin(yaw);worldgoal=xy+np.array([[c,-s],[s,c]])@goal
  created=False;checked=False
  if settled and t-self.last_plan>=.5-1e-6:
   created,_=self.plan_boundary(xy,yaw,worldgoal,t);checked=True
  if not settled or self.plan is None:
   self.tracker._begin_tracking_audit(observation)
   action=self.tracker._zero_action('observation_settling' if not settled else 'local_no_route')
  else:action=self.tracker.act(observation,goal)
  self._packet=None;self.last_frame=packet['frame'];self.time=t
  self.records.append(dict(frame=self.last_frame,time_s=t,local_plan_created=created,local_reference_version=self.local_reference_version,plan=copy.deepcopy(self.plan_meta),unknown_fraction=self.plan_meta.get('unknown_fraction',1.),certified_safe=False,request=action.tolist(),depth_pose_privileged=True,global_reference_consumed=False,guard_measured_this_call=checked,reference_decision=copy.deepcopy(self.reference_decision),reference_installed_time_s=self.reference_installed_time,reference_age_s=None if self.reference_installed_time is None else t-self.reference_installed_time,**self.grid.diagnostics()))
  return action

class QualifiedStartPriorRGB(PriorRGBPolicy,QualifiedStartGuarded):
 pass

def policy_class(enabled):
 class InitialQualification(QualifiedStartPriorRGB):
  initial_start_candidate_enabled=bool(enabled)
  def act(self,observation,goal):
   # Validate before PriorRGB creates a grid or touches current source payload.
   if self._packet is not None:self.initial_start_eligible(self._packet['time_s'])
   return super().act(observation,goal)
 result=with_rolling_endpoint(with_command_domain_cap(with_cusp_coast_acquisition(with_current_forward_intent(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(InitialQualification)),candidate=False))))
 assert sum(c.__name__=='CurrentForwardIntent' for c in result.__mro__)==1
 return result
