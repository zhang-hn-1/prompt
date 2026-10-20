"""One conditional main-request consumer. Geometry/PID and direction rules unchanged.
A straight point screen is not a turn model or stopping guarantee.
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import bank_from_quaternion

def with_straight_pacing(base):
 class StraightPacing(base):
  def act(self,observation,goal):
   action=np.asarray(super().act(observation,goal),np.float32).copy()
   if action.shape!=(10,) or not np.isfinite(action).all():raise ValueError('finite five-block request required')
   tracker=self.tracker;diag=tracker.tracking_diagnostics
   q=np.asarray(observation.quaternion_world_wxyz_history,float)
   if q.ndim!=2 or q.shape[1]!=4 or not len(q):raise ValueError('current quaternion history required')
   bank=bank_from_quaternion(q[-1]);bound=np.deg2rad(3.)
   reasons=[]
   if not diag.get('valid',False) or diag.get('source')!='tracking':reasons.append('invalid_current_tracking_geometry')
   if tracker._active_drive_direction!='forward' or tracker._pending_drive_direction is not None:reasons.append('direction_not_forward_settled')
   if tracker._braking_state!='tracking':reasons.append('not_tracking')
   if not np.all(action[::2]==np.float32(.5)):reasons.append('different_main_request')
   if abs(bank)>bound:reasons.append('measured_bank_outside_straight_screen')
   if np.any(np.abs(.75*action[1::2])>bound):reasons.append('requested_bank_outside_straight_screen')
   old=action.copy()
   if not reasons:action[::2]=1.
   self.records[-1].update(straight_pacing_applied=not reasons,straight_pacing_reasons=reasons,
    straight_pacing_before_request=old.tolist(),straight_pacing_bank_rad=float(bank),
    straight_pacing_bank_bound_rad=float(bound),request=action.tolist(),
    straight_pacing_scope='one_local_request_consumer_not_high_speed_geometry_admission')
   return action
 return StraightPacing
