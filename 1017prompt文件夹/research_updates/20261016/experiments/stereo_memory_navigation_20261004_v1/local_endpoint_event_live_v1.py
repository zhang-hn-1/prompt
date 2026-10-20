"""Separate live adapter; frozen offline v1 bytes retained."""
import numpy as np
from local_endpoint_event_v2 import observe_local_endpoint

def observe_policy_endpoint(policy,observation,time_s,world_goal_xy):
 """Live before policy.act; pure snapshot only. No policy/clock/projection update."""
 tracker=policy.tracker
 if tracker._context is None or tracker._reference_xy is None:
  return dict(time_s=float(time_s),valid=False,reason='no_installed_reference',actor_command_issued=False)
 row=np.asarray(observation.proprio_history[-1],float)
 if row.shape!=(17,) or not np.isfinite(row).all():raise ValueError('current native proprio schema17')
 s=dict(time_s=float(time_s),feedback_time_s=float(time_s),position_xy=np.asarray(observation.position_world_xy).copy(),velocity_xy=row[:2].copy(),reference_xy=np.asarray(tracker._reference_xy).copy(),reference_version=policy.local_reference_version,path_index=tracker._path_index,active_direction=tracker._active_drive_direction,pending_direction=tracker._pending_drive_direction,braking_state=tracker._braking_state,global_goal_xy=np.asarray(world_goal_xy).copy())
 result=observe_local_endpoint(s,tracker.drive_model.stopping_distance);result.update(valid=True,created_live_before_policy_act=True);return result
