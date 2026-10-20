from copy import deepcopy
import numpy as np
import pytest

def fixture():
 from test_forward_course_geometry_real_factory_v4 import make,observation,current
 p=make(True);p._course_history=[(6.6+i*.1,np.array([.2,.2]),.5) for i in range(4)]
 current(p,70,7.);p.act(observation(),np.array([3.,0.]))
 row=deepcopy(p.records[-1]);row['time_s']=7.;row['frame']=70
 row['current_reference_waypoints_after_act']=[vars(w) for w in p.tracker._reference]
 state=dict(time_s=7.,position=[0.,0.,0.],quaternion=[1.,0.,0.,0.])
 return row,state,p._course_history

def test_independent_state_and_clock_tampering_rejected():
 from forward_course_score_evidence_v3 import verify_geometry
 row,state,h=fixture();verify_geometry(row,state,h,enabled=True)
 for key,value in [('time_s',7.1),('position',[.1,0.,0.]),('quaternion',[.9238795,0.,0.,.3826834])]:
  bad=deepcopy(state);bad[key]=value
  with pytest.raises(AssertionError):verify_geometry(row,bad,h,enabled=True)
 bad=deepcopy(h);bad[-1]=(7.1,bad[-1][1],bad[-1][2])
 with pytest.raises(AssertionError):verify_geometry(row,state,bad,enabled=True)

def test_post_act_braking_is_not_composition_eligibility():
 from forward_course_score_evidence_v3 import verify_geometry
 row,state,h=fixture();row['tracker_state_after_act']={'braking':'goal_braking'}
 verify_geometry(row,state,h,enabled=True)
 row['forward_course_geometry']['composition_input']['braking']='direction_change_braking'
 with pytest.raises(AssertionError):verify_geometry(row,state,h,enabled=True)

@pytest.mark.parametrize('event,changed,isolated,prefix,performance,expected',[
 (False,False,False,True,True,'unassessed_event_or_interface'),
 (True,False,False,True,True,'unassessed_event_or_interface'),
 (True,True,False,True,False,'unassessed_event_or_interface'),
 (True,True,True,False,True,'unassessed_event_or_interface'),
 (True,True,True,True,False,'measured_navigation_reject'),
 (True,True,True,True,True,'conditional_knownpose_TaskH_accept'),
])
def test_no_event_equivalent_or_unrelated_fork_are_not_performance_rejection(event,changed,isolated,prefix,performance,expected):
 from forward_course_score_evidence_v3 import classify
 assert classify(event=event,changed=changed,isolated=isolated,prefix=prefix,performance=performance)==expected


def test_actual_first_fork_requires_this_calls_geometry_not_later_event():
 from assess_forward_course_geometry_e1_v4 import intervention_evidence
 row,state,h=fixture();b=deepcopy(row);b['forward_course_geometry']['used']=False
 rows_b=[deepcopy(b) for _ in range(3)];rows_c=[deepcopy(b) for _ in range(3)]
 for i in range(3):rows_b[i]['frame']=rows_c[i]['frame']=i
 rows_c[2]=deepcopy(row);rows_c[2]['frame']=2
 assert not intervention_evidence(1,rows_b,rows_c)['consumer_fork_isolated']
 assert intervention_evidence(2,rows_b,rows_c)['consumer_fork_isolated']
 assert not intervention_evidence(None,rows_b,rows_c)['consumer_fork_isolated']
 rows_c[2]['forward_course_geometry']['composition_input']['position_world_xy']=[.1,0.]
 assert not intervention_evidence(2,rows_b,rows_c)['consumer_fork_isolated']


def test_paired_online_target_pollution_rejected_by_reference_reconstruction():
 from forward_course_score_evidence_v3 import verify_geometry
 row,state,h=fixture();chain=row['current_tracking_chain'];snap=row['forward_course_geometry']['composition_input']
 chain['lookahead_x_m']=snap['lookahead_world_xy'][0]=1.7
 # Parent and selected curvature also recomputed so simple mutual checks pass.
 delta=np.asarray(snap['lookahead_world_xy']);den=delta@delta
 chain['geometric_pure_pursuit_curvature_1_m']=2*delta[1]/den
 k=2*(-np.sin(np.pi/4)*delta[0]+np.cos(np.pi/4)*delta[1])/den
 chain['requested_curvature_1_m']=row['forward_course_geometry']['selected_curvature']=k
 with pytest.raises(AssertionError):verify_geometry(row,state,h,enabled=True)
