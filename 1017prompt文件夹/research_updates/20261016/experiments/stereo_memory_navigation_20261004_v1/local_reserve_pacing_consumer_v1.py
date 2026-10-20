"""One bounded terminal-aware qualification; not a certified stopping bound.
Old straight point and old fixed-reference rejection stay frozen. This is a new,
not-yet-physically-evaluated candidate; source/planner/rollPID gains unchanged.
"""
import numpy as np
from straight_pacing_consumer_v2 import with_straight_pacing
# Fixed measured straight zero-request tail + existing tracking margin + half cell diagonal.
RESERVE_M=.6752270905090967+.2+np.sqrt(2)*.1/2

def with_local_reserve_pacing(base):
 class LocalReservePacing(with_straight_pacing(base)):
  def act(self,observation,goal):
   action=np.asarray(super().act(observation,goal),np.float32).copy() # one original policy act
   row=self.records[-1];attempted=bool(row['straight_pacing_applied']);reasons=[]
   local=None;global_distance=None
   if attempted:
    xy=np.asarray(observation.position_world_xy,float);path=np.asarray(self.tracker._reference_xy,float);g=np.asarray(goal,float)
    if xy.shape!=(2,) or path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or g.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(path).all() or not np.isfinite(g).all():raise ValueError('finite current state/reference/global body goal required')
    # Pure current geometric distance. Never advance policy projection/cursor again.
    local=float(np.linalg.norm(path[-1]-xy));global_distance=float(np.linalg.norm(g))
    if local<RESERVE_M:reasons.append('local_endpoint_inside_measured_reserve')
    if global_distance<RESERVE_M:reasons.append('global_goal_inside_measured_reserve')
    if reasons:action=np.asarray(row['straight_pacing_before_request'],np.float32).copy()
   row.update(straight_pacing_pre_reserve_applied=attempted,straight_pacing_applied=attempted and not reasons,local_reserve_pacing_applied=attempted and not reasons,local_reserve_pacing_attempted=attempted,local_reserve_pacing_reasons=reasons,local_endpoint_distance_m=local,current_global_goal_distance_m=global_distance,local_reserve_m=float(RESERVE_M),local_reserve_not_certified=True,request=action.tolist())
   return action
 return LocalReservePacing
