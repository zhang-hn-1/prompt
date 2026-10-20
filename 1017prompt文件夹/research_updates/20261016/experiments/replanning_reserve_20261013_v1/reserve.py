"""Preserve mandated safety margin; additional .2m future-replanning reserve.
Empirical tracking allowance is an assumption, not a certified stopping bound.
No risk map edits, controller changes, GT or unknown-as-free claims.
"""
import types,numpy as np
import execution_domain_four_point_consumer_v2 as old
import minimum_prefix_transition_planning_v1 as search
from guarded_reference import boundary_state
from reference_continuity_audit import reference_geometry_status
from continuous_cell_tube_v1 import continuous_cell_tube_screen
from depth_local_curve import install_local_reference
RESERVE=.2
def reserve_clearance(grid):return old.planning_clearance(grid)+RESERVE
class ReserveGrid:
 def __init__(self,base):self.base=base
 def __getattr__(self,k):return getattr(self.base,k)
 def plan(self,xy,goal):
  # Preserve original finite/fixed-goal validation and nonbinding audit state.
  # Its route is not selected; reserve route remains the sole query output.
  self.base.plan(xy,goal)
  fn=types.FunctionType(old.TrackingTubeGrid.plan.__code__,dict(old.TrackingTubeGrid.plan.__globals__,planning_clearance=reserve_clearance))
  return fn(self,xy,goal)
def reserve_curve(grid,xy,yaw,goal,drive_model,*,initial_direction,first_primitive_direction=None):
 def planner(req,drive,**kw):return search.plan_kinodynamic_path(search.KinodynamicPlanRequest(**vars(req)),drive,first_primitive_direction=first_primitive_direction,**kw)
 fn=types.FunctionType(old.local_curve_for_direction_tube.__code__,dict(old.local_curve_for_direction_tube.__globals__,planning_clearance=reserve_clearance,plan_kinodynamic_path=planner))
 p,m=fn(ReserveGrid(grid),xy,yaw,goal,drive_model,initial_direction=initial_direction)
 m.update(mandatory_safety_clearance_m=old.planning_clearance(grid),future_replanning_reserve_m=RESERVE,tracking_assumption_certified=False)
 if p is not None:
  points=np.array([[w.x_m,w.y_m] for w in p.waypoints]);guard=reference_geometry_status(np.asarray(xy),points,grid.labels,grid.origin,grid.resolution)
  screen=reserve_screen(grid,guard);m['generated_reference_reserve_screen']=screen
  if not screen['pass']:p=None;m['status']='generated_reference_fails_same_reserve'
 return p,m
def reserve_screen(grid,guard):
 fn=types.FunctionType(continuous_cell_tube_screen.__code__,dict(continuous_cell_tube_screen.__globals__,planning_clearance=reserve_clearance))
 r=fn(grid,guard);r['reserve_clearance_m']=r.pop('existing_planning_clearance_m');r['mandatory_safety_clearance_m']=old.planning_clearance(grid);r['future_replanning_reserve_m']=RESERVE
 return r
def with_replanning_reserve(base):
 class ReplanningReserve(base):
  def reset(self,context):super().reset(context);self.reserve_diagnostic=None
  def act(self,observation,goal):
   action=super().act(observation,goal);self.records[-1]['replanning_reserve']=self.reserve_diagnostic;return action
  def plan_boundary(self,xy,yaw,worldgoal,t):
   before=boundary_state(self.tracker);guard=None
   if self.plan is not None:guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
   keep=guard is not None and guard['retention_necessary_screen_pass'];self.last_tube_viability_screen=None
   if keep and self.tube_viability_armed:
    self.last_tube_viability_screen=continuous_cell_tube_screen(self.grid,guard);keep=bool(self.last_tube_viability_screen['pass'])
   self.reserve_diagnostic=reserve_screen(self.grid,guard) if keep else None
   if keep:keep=bool(self.reserve_diagnostic['pass'])
   initial=None
   if not keep:
    initial=None if self.tracker._context is None else before['active_direction']
    self.first_direction_query=dict(direction=self.current_first_direction,measured_state=before,first_expansion_only=False,minimum_initial_run_primitives=2 if self.current_first_direction else 0,minimum_nominal_initial_arc_m=.6 if self.current_first_direction else 0.,physical_stopping_bound_certified=False,later_reverse_allowed=True,current_and_past_only=True)
    self.plan,self.plan_meta=reserve_curve(self.grid,xy,yaw,worldgoal,self.tracker.drive_model,initial_direction=initial,first_primitive_direction=self.current_first_direction)
    self.first_direction_query.update(status=self.plan_meta['status'],expanded_nodes=self.plan_meta.get('expanded_nodes'),first_queried_direction=None if self.plan is None else self.plan.waypoints[0].drive_direction)
    self.local_reference_version+=1
    if self.plan is not None:install_local_reference(self.tracker,self.plan)
    self.reference_installed_time=t if self.plan is not None else None
   self.last_plan=t
   self.reference_decision=dict(decision='retain' if keep else 'rebuild',measured_time_s=t,guard=guard,initial_or_no_route=guard is None,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,direction_cost_input=initial,tube_viability_armed_at_boundary=self.tube_viability_armed,tube_viability_screen=self.last_tube_viability_screen,replanning_reserve=self.reserve_diagnostic)
   return not keep,self.reference_decision
 return ReplanningReserve
