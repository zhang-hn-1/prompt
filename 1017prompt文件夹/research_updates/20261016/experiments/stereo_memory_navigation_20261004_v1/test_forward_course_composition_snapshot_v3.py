import numpy as np
from test_forward_course_geometry_real_factory_v4 import make,observation,current

def test_composition_state_is_snapshot_before_original_mutates_it():
 p=make(True);p._course_history=[(6.6+i*.1,np.array([.2,.2]),.5) for i in range(4)]
 mutated=[]
 original=p.tracker._compose_tracking_curvature
 def mutate(*a,**kw):
  result=original(*a,**kw)
  p.tracker._braking_state='goal_braking'
  mutated.append(p.tracker._braking_state)
  return result
 p.tracker._compose_tracking_curvature=mutate
 current(p,70,7.);p.act(observation(),np.array([3.,0.]))
 snap=p.records[-1]['forward_course_geometry']['composition_input']
 assert snap['braking']=='tracking' and snap['active']=='forward' and snap['pending'] is None
 assert snap['time_s']==7. and snap['root_yaw_rad']==0. and snap['position_world_xy']==[0.,0.]
 assert snap['nominal_main_rad_s']==.5 and snap['reference_direction']=='forward'
 assert snap['lookahead_world_xy']==[p.records[-1]['current_tracking_chain']['lookahead_x_m'],p.records[-1]['current_tracking_chain']['lookahead_y_m']]
 assert mutated==['goal_braking'] # Parent later restores tracking; snapshot remains the earlier state.
