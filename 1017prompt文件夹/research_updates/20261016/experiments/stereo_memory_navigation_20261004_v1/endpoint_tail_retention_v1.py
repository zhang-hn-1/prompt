"""ONE first-forward-reference tail refresh consumer, no planner/PID/map change."""
import copy,inspect,textwrap,numpy as np
from reference_continuity_audit import reference_geometry_status

def endpoint_tail_screen(guard,xy,path,eligible,goal_threshold_m):
 xy=np.asarray(xy,float);path=np.asarray(path,float)
 if xy.shape!=(2,) or path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or not np.isfinite(xy).all() or not np.isfinite(path).all() or not np.isfinite(goal_threshold_m) or goal_threshold_m<=0:raise ValueError('current finite reference/position/completion radius')
 g=copy.deepcopy(guard);distance=float(np.linalg.norm(xy-path[-1]));change=bool(eligible and g['reject_reasons']==['remaining_arc_under_lookahead'] and distance>goal_threshold_m)
 if change:g['reject_reasons']=[];g['retention_necessary_screen_pass']=True
 g.update(endpoint_tail_intervened=change,endpoint_distance_m=distance,endpoint_completion_threshold_m=float(goal_threshold_m),endpoint_tail_eligible=bool(eligible),old_reject_reasons=guard['reject_reasons'][:],current_geometry_only_not_safety=True)
 return g

def _tail_guard(self,xy,path,labels,origin,resolution):
 original=reference_geometry_status(xy,path,labels,origin,resolution)
 tracker=self.tracker;reference=tracker._reference
 eligible=(self.local_reference_version==1 and tracker._active_drive_direction=='forward' and tracker._pending_drive_direction is None and tracker._braking_state=='tracking' and reference is not None and all(w.drive_direction=='forward' for w in reference))
 result=endpoint_tail_screen(original,xy,path,eligible,.2)
 self.endpoint_tail_last=copy.deepcopy(result);return result

def with_first_endpoint_tail(base):
 method=base.plan_boundary;source=textwrap.dedent(inspect.getsource(method));old='guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)';new='guard=_tail_guard(self,xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)';assert source.count(old)==1,'frozen original boundary identity required'
 scope=dict(method.__globals__,_tail_guard=_tail_guard);exec(compile(source.replace(old,new),__file__,'exec'),scope);cloned=scope['plan_boundary'];assert 'local_curve_for_direction_tube' in cloned.__globals__
 class EndpointTail(base):
  def reset(self,context):super().reset(context);self.endpoint_tail_last=None
  def act(self,observation,goal):
   self.endpoint_tail_last=None;action=super().act(observation,goal);self.records[-1]['endpoint_tail_this_call']=copy.deepcopy(self.endpoint_tail_last);return action
 EndpointTail.plan_boundary=cloned
 return EndpointTail
