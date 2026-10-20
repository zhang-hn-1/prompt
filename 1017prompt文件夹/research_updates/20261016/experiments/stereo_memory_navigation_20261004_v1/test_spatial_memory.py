"""Breaks caught: losing occluded hazards, invented free space, stale free evidence."""
import numpy as np
import spatial_memory as s
import pytest

def test_observed_obstacle_persists_after_visibility_is_lost():
    m=s.SpatialMemory(origin=(-2,-2),size=60,resolution=.1)
    m.update(np.array([[1.,0.,.3]]),time_s=0.,pose_sigma_m=0.)
    m.update(np.empty((0,3)),time_s=1.,pose_sigma_m=0.)
    assert m.status((1,0),1.)=='occupied'
    assert m.status((.7,0),1.)=='unknown'

def test_floor_cannot_overwrite_hazard_or_fill_unseen_gap():
    m=s.SpatialMemory(origin=(-2,-2),size=60,resolution=.1)
    m.update(np.array([[1.,0.,.3],[1.2,0.,0.]]),0.,0.)
    m.update(np.array([[1.,0.,0.]]),1.,0.)
    assert m.status((1,0),1.)=='occupied'
    assert m.status((1.2,0),1.)=='historical_floor'
    assert m.status((.5,0),1.)=='unknown'

def test_expired_or_uncertain_pose_does_not_certify_free():
    m=s.SpatialMemory(origin=(-2,-2),size=60,resolution=.1,max_age_s=10.)
    m.update(np.array([[1.,0.,0.]]),0.,0.)
    assert m.status((1,0),.1)=='historical_floor'
    assert m.status((1,0),11.)=='unknown'
    m.update(np.array([[1.2,0.,0.]]),12.,.2)
    assert m.status((1.2,0),12.)=='unknown'

def test_only_observed_history_can_become_current():
    m=s.SpatialMemory(origin=(-2,-2),size=60,resolution=.1)
    m.update(np.array([[1.,0.,0.]]),0.,0.)
    assert m.status((1,0),0.)=='current_floor'
    m.update(np.empty((0,3)),1.,0.)
    assert m.status((1,0),1.)=='historical_floor'

def test_future_evidence_cannot_answer_past_queries():
    m=s.SpatialMemory(origin=(-2,-2),size=60,resolution=.1)
    m.update(np.array([[1.,0.,0.]]),1.,0.)
    with pytest.raises(ValueError):m.labels(0.)
    with pytest.raises(ValueError):m.status((1,0),0.)
