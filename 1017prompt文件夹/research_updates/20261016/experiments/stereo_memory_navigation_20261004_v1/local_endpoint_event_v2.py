"""Read-only current endpoint event, version2 pre-payload time guard; original V1 stop arithmetic, no policy act."""
import numpy as np

def observe_local_endpoint(snapshot,stopping_distance):
 t=float(snapshot['time_s']);ft=float(snapshot['feedback_time_s']);
 if not np.isfinite(t) or not np.isfinite(ft) or abs(ft-t)>1e-8:raise ValueError('current feedback must share current decision clock')
 p=np.asarray(snapshot['position_xy'],float);v=np.asarray(snapshot['velocity_xy'],float);ref=np.asarray(snapshot['reference_xy'],float);goal=np.asarray(snapshot['global_goal_xy'],float);k=snapshot['path_index'];direction=snapshot['active_direction']
 if p.shape!=(2,) or v.shape!=(2,) or goal.shape!=(2,) or ref.ndim!=2 or ref.shape[1]!=2 or len(ref)<2 or not np.isfinite(np.r_[p,v,goal,ref.ravel()]).all():raise ValueError('finite current state and installed reference')
 if not isinstance(k,(int,np.integer)) or isinstance(k,bool) or not 0<=k<len(ref) or direction not in ('forward','reverse'):raise ValueError('current measured tracker cursor/direction')
 remaining=float(np.linalg.norm(ref[k]-p)+np.linalg.norm(np.diff(ref[k:],axis=0),axis=1).sum());speed=float(np.linalg.norm(v));stop=float(stopping_distance(speed,direction))
 if not np.isfinite(stop) or stop<0:raise ValueError('finite nonnegative frozen stopping distance')
 global_distance=float(np.linalg.norm(goal-p));local=remaining<=stop+.1;global_reached=global_distance<=.2
 event='global_goal_reached' if global_reached else 'local_endpoint_stop_with_distant_global_goal' if local else 'tracking_reference_has_stop_reserve'
 return dict(time_s=t,feedback_time_s=ft,event=event,local_stop_condition=bool(local),global_goal_reached=bool(global_reached),remaining_waypoint_length_m=remaining,endpoint_distance_m=float(np.linalg.norm(ref[-1]-p)),global_goal_distance_m=global_distance,feedback_speed_m_s=speed,stopping_distance_m=stop,stop_bound_m=stop+.1,reference_version=int(snapshot['reference_version']),input_path_index=int(k),input_active_direction=direction,input_pending_direction=snapshot['pending_direction'],input_braking_state=snapshot['braking_state'],cursor_projection_not_reexecuted=True,cursor_is_current_snapshot_not_future_after_act=True,requested_plan_constructed_now=[0.,0.] if local or global_reached else None,plan_is_only_stop_hypothesis_not_actual_future_request=True,actor_command_issued=False,physical_stop_bound_certified=False)
