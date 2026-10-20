"""One retention condition after a measured reversal; no new margin or control law."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from execution_domain_four_point_consumer_v2 import planning_clearance,local_curve_for_direction_tube
from guarded_reference import boundary_state
from reference_continuity_audit import reference_geometry_status
from depth_local_curve import install_local_reference

def tube_retention_screen(grid,guard):
 margin=planning_clearance(grid);segments=[]
 for key in ['bridge_xy','suffix_xy']:
  q=np.asarray(guard[key],float)
  if q.ndim!=2 or q.shape[1]!=2 or not np.isfinite(q).all() or len(q)<1:raise ValueError('current finite retained reference geometry')
  if len(q)==1:segments.append(q)
  for a,b in zip(q[:-1],q[1:]):segments.append(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)))
 points=np.vstack(segments);cells=grid.cells(points);inside=(cells>=0).all(axis=1)&(cells<np.asarray(grid.labels.shape)).all(axis=1);minimum=None
 if inside.all():
  clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution;minimum=float(clearance[tuple(cells.T)].min())
 return dict(**{'pass':minimum is not None and minimum>=margin-1e-8},minimum_raster_clearance_m=minimum,existing_planning_clearance_m=margin,samples=len(points),outside=int((~inside).sum()),sample_step_m=.05,necessary_nominal_partial_screen_only=True,no_free_or_completed_volume=True)

def with_tube_viability_after_measured_reversal(base):
 class TubeViability(base):
  def reset(self,context):
   super().reset(context);self.tube_viability_armed=False;self.measured_reverse_seen=False;self.last_tube_viability_screen=None
  def act(self,observation,goal):
   before=self.tracker._active_drive_direction;measured=np.asarray(observation.proprio_history,float)
   if measured.ndim!=2 or measured.shape[1]!=17 or not np.isfinite(measured).all():raise ValueError('finite current measured proprio schema17')
   if before=='reverse' and measured[-1,12]<-.1:self.measured_reverse_seen=True
   action=super().act(observation,goal);after=self.tracker._active_drive_direction
   event=not self.tube_viability_armed and self.measured_reverse_seen and before=='reverse' and after=='forward'
   if event:self.tube_viability_armed=True
   self.records[-1].update(tube_viability_armed=self.tube_viability_armed,tube_viability_event_armed=event,measured_reverse_seen=self.measured_reverse_seen,tube_viability_screen=self.last_tube_viability_screen)
   return action
  def plan_boundary(self,xy,yaw,worldgoal,t):
   before=boundary_state(self.tracker);guard=None
   if self.plan is not None:guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
   keep=guard is not None and guard['retention_necessary_screen_pass'];self.last_tube_viability_screen=None
   if keep and self.tube_viability_armed:
    self.last_tube_viability_screen=tube_retention_screen(self.grid,guard);keep=keep and self.last_tube_viability_screen['pass']
   initial=None
   if not keep:
    initial=None if self.tracker._context is None else before['active_direction']
    if initial not in (None,'forward','reverse'):raise ValueError('valid accepted direction required')
    self.plan,self.plan_meta=local_curve_for_direction_tube(self.grid,xy,yaw,worldgoal,self.tracker.drive_model,initial_direction=initial)
    self.local_reference_version+=1
    if self.plan is not None:install_local_reference(self.tracker,self.plan)
    self.reference_installed_time=t if self.plan is not None else None
   self.last_plan=t
   decision=dict(decision='retain' if keep else 'rebuild',measured_time_s=t,guard=guard,initial_or_no_route=guard is None,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,direction_cost_input=initial,tube_viability_armed_at_boundary=self.tube_viability_armed,tube_viability_screen=self.last_tube_viability_screen)
   self.reference_decision=decision
   return not keep,decision
 return TubeViability
