import sys
sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
import numpy as np
from reproduce_four_point_runtime_failure_v2 import reconstruct

def test_current_execution_domain_source_reaches_the_unchanged_tracker_inverse():
 import supported_four_point_consumer as old
 from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
 p,obs,goal=reconstruct()
 # Bind only nominal primitive-source consumer; original act chain, inverse, caps/PID untouched.
 old.local_curve_for_direction_tube=local_curve_for_direction_tube
 try:
  action=p.act(obs,goal)
 finally:
  # restore module for unrelated suite consumers
  import supported_four_point_consumer
  supported_four_point_consumer.local_curve_for_direction_tube=_original
 assert action.shape==(10,) and np.isfinite(action).all()
 assert np.all(action[::2]<0) and np.all(np.abs(action[::2])<=np.float32(.3))
 assert p.plan is not None and p.plan.waypoints[0].main_command_rad_s==-.5
 from execution_domain_four_point_source_v2 import source_identity
 assert source_identity()['reverse_layers']==dict(planned_carrier_main=-.5,actual_task_cap_main=-.3,geometry_point_main=-.3)

import supported_four_point_consumer
_original=supported_four_point_consumer.local_curve_for_direction_tube
