import numpy as np
from clock_prefix import prefix_matches

def sample():
 return {'position_world_xy':np.array([[0.,0.],[1.,0.],[2.,0.]]),'simulation_time_s':np.array([0.,.02,.04]),'raw_simulation_time_s':np.array([7.,7.02,7.04]),'raw_physics_step_index':np.array([350,351,352])}

def test_constant_global_epoch_is_not_motion_difference():
 a=sample();b=sample();a['raw_simulation_time_s']+=.2;a['raw_physics_step_index']+=10
 assert all(prefix_matches(a,b,3).values())

def test_clock_interval_drift_rejected():
 a=sample();b=sample();a['raw_simulation_time_s'][2]+=.001
 assert not all(prefix_matches(a,b,3).values())

def test_physical_difference_still_rejected():
 a=sample();b=sample();a['position_world_xy'][1,0]+=.0001
 assert not all(prefix_matches(a,b,3).values())
