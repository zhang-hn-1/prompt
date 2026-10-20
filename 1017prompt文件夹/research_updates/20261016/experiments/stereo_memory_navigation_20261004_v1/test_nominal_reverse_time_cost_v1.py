"""Cost request conversion must preserve the original geometry and request."""
import dataclasses
import numpy as np
import pytest
from minimum_prefix_transition_planning_v1 import KinodynamicPlanRequest
from nominal_reverse_time_cost_v1 import nominal_time_request, SPEED_RATIO

def test_only_reverse_cost_changes_and_input_is_immutable():
    r=KinodynamicPlanRequest(np.zeros((20,20),np.uint8),.1,(0.,0.),(.5,.5),0.,(1.,.5))
    s=nominal_time_request(r)
    assert s is not r
    np.testing.assert_array_equal(s.occupancy_grid,r.occupancy_grid)
    for f in dataclasses.fields(r):
        if f.name not in ['reverse_cost_multiplier','occupancy_grid']:
            a,b=getattr(r,f.name),getattr(s,f.name)
            assert (a is b) or a==b
    assert r.reverse_cost_multiplier==1.25
    assert s.reverse_cost_multiplier==pytest.approx(1.25*SPEED_RATIO)
    assert SPEED_RATIO==pytest.approx(.18854965269565582/.09878528863191605)
    assert s.reverse_cost_multiplier>r.reverse_cost_multiplier>=1

def test_reverse_length_penalty_is_time_normalized_but_cusp_unchanged():
    r=KinodynamicPlanRequest(np.zeros((20,20),np.uint8),.1,(0.,0.),(.5,.5),0.,(1.,.5))
    s=nominal_time_request(r)
    assert s.primitive_length_m*s.reverse_cost_multiplier==pytest.approx(.3/.09878528863191605*.18854965269565582*1.25)
    assert s.direction_change_cost_m==r.direction_change_cost_m

def test_outgoing_waypoint_labels_count_first_edge_once_and_root_cusp():
    from audit_nominal_reverse_time_cost_v1 import nominal_cost
    w=[dict(drive_direction=d) for d in ['forward','reverse','reverse']]
    costs=nominal_cost(w,None)
    assert costs['reverse_nominal_length_m']==pytest.approx(.3)
    assert costs['normalized_nominal_cost_m']==pytest.approx(.3+.3*1.25*SPEED_RATIO+.5)
    assert nominal_cost(w,'reverse')['normalized_nominal_cost_m']==pytest.approx(costs['normalized_nominal_cost_m']+.5)
    reverse=[dict(drive_direction=d) for d in ['reverse','forward','forward']]
    assert nominal_cost(reverse,'forward')['normalized_nominal_cost_m']==pytest.approx(.3+.3*1.25*SPEED_RATIO+1.)
