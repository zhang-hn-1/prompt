import numpy as np
from route_qualification import qualify_curves


def test_two_blocked_curves_cannot_win_by_relative_clearance():
    result=qualify_curves(np.array([[1.,.6],[1.,-.6]]),np.zeros(2),np.array([8.,0.]),known_clear_swept_region=True)
    assert result['selected_side'] is None
    assert all(not r['observed_clearance_pass'] for r in result['curves'].values())


def test_no_observation_is_unknown_not_certified_free():
    result=qualify_curves(np.empty((0,2)),np.zeros(2),np.array([8.,0.]))
    assert result['selected_side'] is None
    assert all(r['reason']=='no_observed_space' for r in result['curves'].values())


def test_distant_obstacles_do_not_certify_unseen_swept_space():
    result=qualify_curves(np.array([[1.,-6.]]),np.zeros(2),np.array([8.,0.]))
    assert all(r['observed_clearance_pass'] for r in result['curves'].values())
    assert result['selected_side'] is None


def test_clear_observed_side_requires_separate_space_evidence():
    result=qualify_curves(np.array([[1.,-1.2]]),np.zeros(2),np.array([8.,0.]),known_clear_swept_region=True)
    assert result['selected_side']==1
