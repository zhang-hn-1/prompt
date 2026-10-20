"""One before-install partial-reference selection; inherited act remains single-call."""
import types
import numpy as np
from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
from guarded_reference import boundary_state
from bounded_forward_prefix_selection_v1 import choose_prefix
from execution_domain_four_point_source_v2 import supported_curvature

def policy_class(enabled,*,goal_axis_world):
 from current_tracking_chain_telemetry_v1 import policy_class as original
 axis=np.asarray(goal_axis_world,float).copy()
 if axis.shape!=(2,) or not np.isfinite(axis).all() or np.linalg.norm(axis)==0:raise ValueError('finite frozen goal axis required')
 axis/=np.linalg.norm(axis);axis.setflags(write=False)
 base=original(True)
 class BoundedForwardPrefix(base):
  def __init__(self,*a,**kw):
   super().__init__(*a,**kw);self.bounded_prefix_used=False;self._bounded_decision=None
  def reset(self,context):
   super().reset(context);self.bounded_prefix_used=False;self._bounded_decision=None
  def act(self,o,goal):
   self._bounded_decision=dict(intervention=False,valid=False,reason='no_current_rebuild',goal_axis_world=axis.tolist())
   action=super().act(o,goal)
   self.records[-1]['bounded_prefix_decision']=self._bounded_decision
   self.records[-1]['bounded_prefix_used']=self.bounded_prefix_used
   return action
  def plan_boundary(self,*a,**kw):
   method=super().plan_boundary
   scope=dict(method.__func__.__globals__)
   # The real B0 owner is minimum-prefix FirstPrimitiveDirection, which itself
   # clones TubeViability's namespace. Intercept its planner callback, not Tube's
   # shadowed module global. Installation/retention remain inside inherited code.
   if 'local_curve_first_direction' not in scope:raise RuntimeError('unsupported real boundary owner')
   parent=scope['local_curve_first_direction']
   def select(*args,**kwargs):
    full,meta=parent(*args,**kwargs)
    grid,xy,yaw=args[:3];state=boundary_state(self.tracker)
    decision=dict(valid=True,intervention=False,reason='ineligible',goal_axis_world=axis.tolist(),parent_full_plan=None if full is None else [vars(w).copy() for w in full.waypoints],parent_plan_meta=dict(meta),measured_state_before_install=state,current_root_xy=np.asarray(xy).tolist(),current_yaw=float(yaw),selector_trace=[])
    self._bounded_decision=decision
    eligible=(full is not None and len(full.waypoints)>1 and full.waypoints[0].drive_direction=='reverse' and self.tracker._context is not None and state['active_direction']=='forward' and state['pending_direction'] is None and state['braking_state']=='tracking' and not self.bounded_prefix_used)
    if not enabled or not eligible:return full,meta
    selected,trace=choose_prefix(xy,yaw,axis,grid,supported_curvature);decision['selector_trace']=trace
    if selected is None:decision['reason']='no_compatible_prefix';return full,meta
    poses=selected['poses'];requests=selected['requests']
    points=tuple(KinodynamicWaypoint(float(p[0]),float(p[1]),float(p[2]),'forward',*requests[min(i,len(requests)-1)]) for i,p in enumerate(poses))
    plan=KinodynamicPlanResult('success',points,0,.3*len(requests),0)
    new_meta=dict(meta,partial_reference=True,old_local_target_reached=False,certified_safe=False,nominal_only=True,partial_terminal_xy=poses[-1][:2],parent_full_plan=decision['parent_full_plan'],local_path_length_m=plan.path_length_m,direction_changes=0)
    self.bounded_prefix_used=True;decision.update(intervention=True,reason='bounded_partial_exploration',selected=selected)
    return plan,new_meta
   scope['local_curve_first_direction']=select
   fn=types.FunctionType(method.__func__.__code__,scope,closure=method.__func__.__closure__)
   return fn(self,*a,**kw)
 return BoundedForwardPrefix
