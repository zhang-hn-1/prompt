"""Temporary instance hooks capture one target and one composition call."""
import numpy as np
from forward_course_geometry_v1 import select_forward_geometry

def with_forward_course_geometry(base,*,enabled):
 class ForwardCourseGeometry(base):
  def reset(self,*args,**kwargs):
   self._course_history=[]
   return super().reset(*args,**kwargs)
  def act(self,o,goal):
   if not hasattr(self,'_course_history'):self._course_history=[]
   if self._packet is None:raise RuntimeError('current packet required')
   now=float(self._packet['time_s'])
   clocks=np.asarray([x[0] for x in self._course_history]+[now],float)
   if not np.isfinite(clocks).all() or np.any(np.diff(clocks)<=0) or (len(clocks)>1 and not np.allclose(np.diff(clocks),.1,atol=1e-6,rtol=0)):raise ValueError('causal consecutive current history clocks required')
   if hasattr(self,'time') and np.isfinite(self.time) and not np.isclose(now-self.time,.1,atol=1e-6,rtol=0):raise ValueError('current packet10Hz clock required')
   row=np.asarray(o.proprio_history,float)[-1]
   self._course_history.append((now,row[:2].copy(),float(row[12])))
   self._course_history=self._course_history[-5:]
   tr=self.tracker;old_target=tr._lookahead_target;old_compose=tr._compose_tracking_curvature
   context={'target':None,'decision':dict(used=False,source='composition_not_called',valid=False)}
   def target(*args,**kwargs):
    value=old_target(*args,**kwargs);context['target']=np.asarray(value,float).copy();return value
   def compose(observation,*,waypoint,geometric_curvature_1_m):
    selected=float(geometric_curvature_1_m)
    if enabled and abs(float(waypoint.main_command_rad_s))>1e-12:
     if context['target'] is None:raise RuntimeError('same-call lookahead required')
     h=self._course_history
     selected,decision=select_forward_geometry(selected,context['target']-np.asarray(observation.position_world_xy,float),now=now,times=[x[0] for x in h],velocity=[x[1] for x in h],main_velocity=[x[2] for x in h],active=tr._active_drive_direction,pending=tr._pending_drive_direction,braking=tr._braking_state,reference_direction=waypoint.drive_direction)
    else:decision=dict(used=False,source='disabled_or_planned_zero_parent_geometry',parent_curvature=selected)
    result,meta=old_compose(observation,waypoint=waypoint,geometric_curvature_1_m=selected)
    context['decision']=dict(decision,valid=True,enabled=bool(enabled))
    return result,dict(meta,forward_course_geometry=context['decision'])
   tr._lookahead_target=target;tr._compose_tracking_curvature=compose
   try:action=super().act(o,goal)
   finally:tr._lookahead_target=old_target;tr._compose_tracking_curvature=old_compose
   self.records[-1]['forward_course_geometry']=context['decision']
   return action
 return ForwardCourseGeometry

def policy_class(enabled):
 from current_tracking_chain_telemetry_v1 import policy_class as parent
 return with_forward_course_geometry(parent(True),enabled=enabled)
