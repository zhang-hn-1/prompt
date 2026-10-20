import pytest
from assess_bounded_prefix_e1_v1 import verify_current_root,intervention_evidence

def test_independent_current_root_rejects_forged_yaw_and_time():
 state=dict(time_s=12.,position=[1.,2.,.4],quaternion=[1.,0.,0.,0.])
 d=dict(current_root_xy=[1.,2.],current_yaw=0.)
 verify_current_root(d,state,12.)
 with pytest.raises(AssertionError):verify_current_root(dict(d,current_yaw=.01),state,12.)
 with pytest.raises(AssertionError):verify_current_root(d,state,12.1)

def test_first_reference_intervention_is_the_request_fork():
 d=dict(intervention=True,parent_full_plan=[dict(drive_direction='reverse')],measured_state_before_install={'active_direction':'forward'})
 rows=[dict(frame=i,bounded_prefix_decision=dict(intervention=False)) for i in range(120)]+[dict(frame=120,bounded_prefix_decision=d)]
 # Actual driver arrays use global-frame indexing; this helper only tests identity logic.
 assert intervention_evidence(120,None,None,rows,[{}]*120+[dict(bounded_prefix_decision=d)])['consumer_fork_isolated']
 assert not intervention_evidence(119,None,None,rows,[{}]*121)['consumer_fork_isolated']

@pytest.mark.parametrize('intervened,isolated,prefix,accepted,expected',[
 (False,False,True,False,'unassessed_event_or_interface'),
 (True,False,True,False,'unassessed_event_or_interface'),
 (True,True,False,False,'unassessed_event_or_interface'),
 (True,True,True,False,'measured_navigation_reject'),
 (True,True,True,True,'conditional_knownpose_TaskH_accept')])
def test_classification_separates_event_interface_and_performance(intervened,isolated,prefix,accepted,expected):
 from assess_bounded_prefix_e1_v1 import classify
 assert classify(intervened,isolated,prefix,accepted)==expected
