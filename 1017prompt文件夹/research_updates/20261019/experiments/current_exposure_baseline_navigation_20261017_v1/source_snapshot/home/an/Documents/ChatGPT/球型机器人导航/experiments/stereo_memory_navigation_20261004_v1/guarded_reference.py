"""One reference-replacement consumer; frozen planner and tracker remain intact."""
import copy,math
import numpy as np
from depth_local_curve import DepthLocalCurvePolicy,finite,local_curve,install_local_reference
from provenance_grid import ProvenanceGrid
from reference_continuity_audit import reference_geometry_status

def boundary_state(tracker):
 return dict(path_index=int(tracker._path_index),active_direction=tracker._active_drive_direction,pending_direction=tracker._pending_drive_direction,stopped_cycles=int(tracker._stopped_cycles),off_reference_cycles=int(tracker._off_reference_cycles),failure=tracker._failure_subreason,braking_state=tracker._braking_state,mapping_count=len(tracker.mapping_records))

class GuardedReferencePolicy(DepthLocalCurvePolicy):
 policy_name='privileged_depth_guarded_reference_diagnostic'
 def reset(self,context):
  super().reset(context)
  self.reference_installed_time=None;self.reference_decision=None
 def plan_boundary(self,xy,yaw,worldgoal,t):
  before=boundary_state(self.tracker);guard=None
  if self.plan is not None:
   guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
  keep=guard is not None and guard['retention_necessary_screen_pass']
  self.last_plan=t
  if not keep:
   self.plan,self.plan_meta=local_curve(self.grid,xy,yaw,worldgoal,self.tracker.drive_model)
   self.local_reference_version+=1
   # No reset/forced direction/low-speed wait. Original geometry replacement only.
   if self.plan is not None:install_local_reference(self.tracker,self.plan)
   self.reference_installed_time=t if self.plan is not None else None
  decision=dict(decision='retain' if keep else 'rebuild',measured_time_s=t,guard=guard,initial_or_no_route=guard is None,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False)
  self.reference_decision=decision
  return not keep,decision
 def act(self,observation,goal):
  # Same frozen base transaction, with only the planning-boundary consumer changed.
  if self._packet is None:raise RuntimeError('current packet required')
  packet=self._packet;t=packet['time_s'];xy=np.asarray(observation.position_world_xy,float);yaw=finite(observation.yaw_world_rad);goal=np.asarray(goal,float)
  if np.linalg.norm(packet['root_xyz'][:2]-xy)>1e-5:raise ValueError('current state/packet mismatch')
  if goal.shape!=(2,) or not np.isfinite(goal).all():raise ValueError('invalid goal')
  if self.grid is None:self.grid=ProvenanceGrid(xy-20.,memory_mode='history')
  self.grid.update_points(packet['points_world'],packet['root_xyz'],t)
  c,s=math.cos(yaw),math.sin(yaw);worldgoal=xy+np.array([[c,-s],[s,c]])@goal
  created=False;checked=False
  if t>=6.-1e-6 and t-self.last_plan>=.5-1e-6:
   created,_=self.plan_boundary(xy,yaw,worldgoal,t);checked=True
  if t<6.-1e-6 or self.plan is None:
   self.tracker._begin_tracking_audit(observation)
   action=self.tracker._zero_action('observation_settling' if t<6.-1e-6 else 'local_no_route')
  else:action=self.tracker.act(observation,goal)
  self._packet=None;self.last_frame=packet['frame'];self.time=t
  self.records.append(dict(frame=self.last_frame,time_s=t,local_plan_created=created,local_reference_version=self.local_reference_version,plan=copy.deepcopy(self.plan_meta),unknown_fraction=self.plan_meta.get('unknown_fraction',1.),certified_safe=False,request=action.tolist(),depth_pose_privileged=True,global_reference_consumed=False,guard_measured_this_call=checked,reference_decision=copy.deepcopy(self.reference_decision),reference_installed_time_s=self.reference_installed_time,reference_age_s=None if self.reference_installed_time is None else t-self.reference_installed_time,**self.grid.diagnostics()))
  return action
