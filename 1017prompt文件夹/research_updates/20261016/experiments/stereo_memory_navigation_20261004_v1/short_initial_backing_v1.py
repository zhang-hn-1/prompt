"""ONE local target/reference consumer; no forced side/direction or controller update."""
import copy,types,dataclasses,numpy as np
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid

def initial_reverse_edges(plan):
 if plan is None or plan.status!='success' or len(plan.waypoints)<2:return None
 count=0
 for w in plan.waypoints[:-1]:
  if w.drive_direction!='reverse':break
  count+=1
 return count

def prefer_shorter_initial_backing(original,alternative):
 b=initial_reverse_edges(original);a=initial_reverse_edges(alternative)
 return alternative if b is not None and b>0 and a is not None and a<b else original

def current_side_grid(grid,goal):
 c=copy.deepcopy(grid.commitment)
 if c is None or c['decision_time_s']>grid.time:return None
 clone=RouteSideCommitmentGrid(grid.origin.copy(),10,static_scene_assumption=True);clone.labels=grid.labels.copy();clone.resolution=grid.resolution;clone.time=grid.time;clone.hazard_acquisition_time=grid.hazard_acquisition_time.copy();clone.commitment=c;clone._axis_start=np.asarray(c['axis_start']).copy();clone._axis=np.asarray(c['axis']).copy();clone._normal=np.array([-clone._axis[1],clone._axis[0]]);clone._goal=np.asarray(goal).copy()
 return clone

def with_short_initial_backing(base):
 method=base.plan_boundary;original=method.__globals__.get('local_curve_first_direction')
 if original is None:raise ValueError('frozen FirstPrimitiveDirection boundary identity required')
 class ShortInitialBacking(base):
  def reset(self,context):super().reset(context);self.short_backing_this_call=None;self.short_backing_alternative_queries=0
  def act(self,observation,goal):
   self.short_backing_this_call=None;action=super().act(observation,goal);self.records[-1]['short_initial_backing_this_call']=copy.deepcopy(self.short_backing_this_call);return action
  def plan_boundary(self,*args,**kwargs):
   def choose(grid,xy,yaw,goal,drive,**kw):
    plan,meta=original(grid,xy,yaw,goal,drive,**kw);tracker=self.tracker
    eligible=(self.local_reference_version==1 and tracker._active_drive_direction=='forward' and tracker._pending_drive_direction is None and tracker._braking_state=='tracking' and self.short_backing_alternative_queries==0 and initial_reverse_edges(plan) not in (None,0))
    if not eligible:return plan,meta
    alternative_grid=current_side_grid(grid,goal)
    if alternative_grid is None:return plan,meta
    self.short_backing_alternative_queries+=1;before=grid.labels.copy();alt,altmeta=original(alternative_grid,xy,yaw,goal,drive,**kw);assert np.array_equal(before,grid.labels)
    selected=prefer_shorter_initial_backing(plan,alt);picked=selected is alt and alt is not None
    event=dict(time_s=float(grid.time),chosen_optional=picked,original_initial_reverse_edges=initial_reverse_edges(plan),alternative_initial_reverse_edges=initial_reverse_edges(alt),original_plan=None if plan is None else dataclasses.asdict(plan),alternative_plan=None if alt is None else dataclasses.asdict(alt),original_meta=copy.deepcopy(meta),alternative_meta=copy.deepcopy(altmeta),current_xy=np.asarray(xy).tolist(),past_side_hint=copy.deepcopy(grid.commitment),side_hint_not_observation=True,no_map_change=True,no_actual_direction_or_cursor_assigned=True,nominal_geometry_not_transient_or_safety_certificate=True,max_one_alternative_query=True)
    self.short_backing_this_call=event
    chosen_meta=copy.deepcopy(altmeta if picked else meta);chosen_meta['short_initial_backing_choice']=copy.deepcopy(event)
    return selected,chosen_meta
   scope=dict(method.__globals__,local_curve_first_direction=choose);fn=types.FunctionType(method.__code__,scope,closure=method.__closure__)
   return fn(self,*args,**kwargs)
 return ShortInitialBacking
