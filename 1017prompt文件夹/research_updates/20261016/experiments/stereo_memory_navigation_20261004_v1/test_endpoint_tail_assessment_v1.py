import numpy as np
from assess_endpoint_tail_e1_closed_v1 import phase_metrics,geometric

def test_before_action_replacement_state_counts_for_real_endpoint():
 states=[dict(time_s=0.,position=[0,0,0]),dict(time_s=.1,position=[1,0,0]),dict(time_s=.2,position=[5,0,0])]
 rows=[dict(planner_act_called=True,local_reference_version=1),dict(planner_act_called=True,local_reference_version=2),dict(planner_act_called=True,local_reference_version=2)]
 m,_=phase_metrics(states,rows,np.array([[0.,0.],[1.,0.]]),.2)
 assert m['samples']==2 and m['real_endpoint_reached'] and m['last_time_s']==.1
 # A post-contact endpoint must never qualify the phase.
 m,_=phase_metrics(states,rows,np.array([[0.,0.],[1.,0.]]),.05);assert not m['real_endpoint_reached']

def test_geometric_score_has_no_cursor_or_candidate_projection_input():
 c,s=geometric(np.array([[.25,.1],[.75,-.2]]),np.array([[0.,0.],[1.,0.]]));np.testing.assert_allclose(c,[.1,.2]);np.testing.assert_allclose(s,[.25,.75])

def test_last_capture_of_last_executed_first_reference_interval_included():
 states=[dict(time_s=0.,position=[.6,0,0]),dict(time_s=.1,position=[.9,0,0])];rows=[dict(planner_act_called=True,local_reference_version=1)]
 m,_=phase_metrics(states,rows,np.array([[0.,0.],[1.,0.]]),.1)
 assert m['samples']==2 and m['real_endpoint_reached'] and m['last_time_s']==.1
 m,_=phase_metrics(states,rows,np.array([[0.,0.],[1.,0.]]),.05);assert m['samples']==1 and not m['real_endpoint_reached']
