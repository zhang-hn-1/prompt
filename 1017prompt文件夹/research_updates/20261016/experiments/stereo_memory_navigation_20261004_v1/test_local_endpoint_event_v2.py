import numpy as np,pytest
from local_endpoint_event_v2 import observe_local_endpoint

def snap():return dict(time_s=2.,feedback_time_s=2.,position_xy=[.72,0],velocity_xy=[.2,0],reference_xy=[[0,0],[1,0]],reference_version=1,path_index=0,active_direction='forward',pending_direction=None,global_goal_xy=[5,0],braking_state='tracking')
def test_stop_formula_uses_tracker_waypoint_remaining_not_geometric_projection():
 s=snap();r=observe_local_endpoint(s,lambda speed,direction:.25)
 assert r['remaining_waypoint_length_m']==pytest.approx(1.72)
 assert r['endpoint_distance_m']==pytest.approx(.28)
 assert not r['local_stop_condition']
 s['path_index']=1;r=observe_local_endpoint(s,lambda speed,direction:.25)
 assert r['local_stop_condition'] and not r['global_goal_reached']
 assert r['event']=='local_endpoint_stop_with_distant_global_goal'
 assert r['requested_plan_constructed_now']==[0.,0.]
def test_readonly_inputs_and_distinct_goal_event():
 s=snap();s['path_index']=1;s['global_goal_xy']=[.75,0];before=repr(s)
 r=observe_local_endpoint(s,lambda speed,direction:.25)
 assert repr(s)==before and r['event']=='global_goal_reached'
 assert not r['actor_command_issued']
def test_reject_future_feedback_and_invalid_cursor():
 s=snap();s['feedback_time_s']=2.1
 with pytest.raises(ValueError):observe_local_endpoint(s,lambda speed,direction:.25)
 s=snap();s['path_index']=2
 with pytest.raises(ValueError):observe_local_endpoint(s,lambda speed,direction:.25)
