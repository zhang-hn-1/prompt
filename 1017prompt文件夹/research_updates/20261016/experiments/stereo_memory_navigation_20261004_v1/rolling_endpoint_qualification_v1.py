"""One local-reserve context change; current route is a nominal TaskH hypothesis."""
import numpy as np
from measured_speed_consumer_v1 import with_measured_speed,FEEDFORWARD,KP,TARGET,RESERVE

def rolling_context(policy,o,goal):
 context=dict(eligible=False,reasons=[],local_end_to_route_end_m=None,route_end_to_current_global_goal_m=None,route_is_nominal_hypothesis=True,observed_free=False,certified_safe=False)
 # Validate causal boundary before even obtaining plan/reference/route payloads.
 try:
  d=policy.reference_decision;t=float(policy.time);bt=float(d['measured_time_s'])
  if d.get('decision') not in ('retain','rebuild') or d.get('shared_full_frozen_reference',False):
   context['reasons']=['not_ordinary_rolling_boundary'];return context
  if not np.isfinite([bt,t]).all() or bt>t+1e-8 or t-bt>.5+1e-6:
   context['reasons']=['not_current_past_boundary'];return context
 except (AttributeError,KeyError,TypeError,ValueError):
  context['reasons']=['missing_or_invalid_current_context'];return context
 try:
  plan=policy.plan;meta=policy.plan_meta
  if plan.status!='success' or meta.get('status')!='success' or plan.direction_changes!=0 or any(w.drive_direction!='forward' for w in plan.waypoints):
   context['reasons']=['not_successful_all_forward_plan'];return context
  path=np.asarray(policy.tracker._reference_xy,float);planned=np.array([[w.x_m,w.y_m] for w in plan.waypoints])
  if path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or not np.array_equal(path,planned):
   context['reasons']=['plan_reference_identity_mismatch'];return context
  xy=np.asarray(o.position_world_xy,float);g=np.asarray(goal,float);yaw=float(o.yaw_world_rad);route=np.asarray(meta['route_xy'],float)
  if xy.shape!=(2,) or g.shape!=(2,) or route.ndim!=2 or route.shape[1]!=2 or len(route)<2 or not np.isfinite(yaw) or not all(np.isfinite(v).all() for v in (xy,g,path,route)):raise ValueError('finite current geometry required')
  c,s=np.cos(yaw),np.sin(yaw);worldgoal=xy+np.array([[c,-s],[s,c]])@g;context['route_end_to_current_global_goal_m']=float(np.linalg.norm(route[-1]-worldgoal));context['local_end_to_route_end_m']=float(np.linalg.norm(path[-1]-route[-1]))
  if context['route_end_to_current_global_goal_m']>.2:context['reasons'].append('nominal_route_not_current_global_goal')
  if context['local_end_to_route_end_m']<RESERVE:context['reasons'].append('final_or_short_nominal_continuation')
 except (AttributeError,KeyError,TypeError,ValueError,IndexError):context['reasons'].append('missing_or_invalid_current_context')
 context['eligible']=not context['reasons'];return context

def with_rolling_endpoint(base):
 class RollingEndpoint(with_measured_speed(base)):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();row=self.records[-1];parent=action.copy();context=rolling_context(self,o,goal);bypass=False
   if row['measured_speed_control_reasons']==['preserve_original_within_same_reserve'] and row['measured_speed_global_distance_m']>=RESERVE and context['eligible']:
    main=float(np.clip(FEEDFORWARD+KP*(TARGET-row['measured_speed_worldXY_m_s']),.5,1.));action[::2]=main;bypass=True
    row.update(measured_speed_control_eligible=True,measured_speed_control_reasons=[],measured_speed_main_requested=main,measured_speed_control_applied=not np.array_equal(action,np.asarray(row['measured_speed_before_request'],np.float32)),request=action.tolist())
   row.update(rolling_local_reserve_bypassed=bypass,rolling_control_applied=not np.array_equal(action,parent),rolling_parent_request=parent.tolist(),rolling_endpoint_context=context)
   return action
 return RollingEndpoint
