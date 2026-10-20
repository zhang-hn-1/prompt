"""One current-speed final main producer; original aux/mapping/PID unchanged.
The static straight calibration is feedforward tuning, not a general turn model.
"""
import numpy as np
LOW_SPEED=0.19515018165111542
SLOPE=0.3393917977809906
TARGET=.34
FEEDFORWARD=.5+(TARGET-LOW_SPEED)/SLOPE
KP=.5/SLOPE
RESERVE=.9459377686277514

def with_measured_speed(base):
 class MeasuredSpeed(base):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();row=self.records[-1];before=action.copy();tr=self.tracker;diag=tr._tracking_audit;reasons=[];speed=None;distance=None;global_distance=None;requested=None
   if not diag.get('valid',False) or diag.get('source')!='tracking':reasons.append('not_current_normal_tracking')
   if tr._active_drive_direction!='forward' or tr._pending_drive_direction is not None or tr._braking_state!='tracking':reasons.append('direction_or_braking')
   if not np.all(action[::2]==np.float32(.5)):reasons.append('preserve_original_other_main')
   if not reasons:
    history=np.asarray(o.proprio_history,float);xy=np.asarray(o.position_world_xy,float);path=np.asarray(tr._reference_xy,float);g=np.asarray(goal,float)
    if history.ndim!=2 or not len(history) or history.shape[1]<2 or xy.shape!=(2,) or path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or g.shape!=(2,) or not all(np.isfinite(x).all() for x in [history,xy,path,g]):raise ValueError('finite currentproprio/reference/goal required')
    speed=float(np.linalg.norm(history[-1,:2]));distance=float(np.linalg.norm(path[-1]-xy));global_distance=float(np.linalg.norm(g))
    if min(distance,global_distance)<RESERVE:reasons.append('preserve_original_within_same_reserve')
    else:
     requested=float(np.clip(FEEDFORWARD+KP*(TARGET-speed),.5,1.));action[::2]=requested
   row.update(measured_speed_control_applied=not np.array_equal(action,before),measured_speed_control_eligible=not reasons,measured_speed_control_reasons=reasons,measured_speed_worldXY_m_s=speed,measured_speed_local_endpoint_m=distance,measured_speed_global_distance_m=global_distance,measured_speed_main_requested=requested,measured_speed_before_request=before.tolist(),measured_speed_target_m_s=TARGET,measured_speed_feedforward=FEEDFORWARD,measured_speed_Kp=KP,measured_speed_reserve_m=RESERVE,request=action.tolist(),measured_speed_no_high_main_inverse_admission=True)
   return action
 return MeasuredSpeed
