import numpy as np
import local_map as m

def test_unobserved_space_remains_unknown():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1)
    assert np.all(b.labels==-1)
    b.seed_body(np.array([0.,0.]))
    assert b.labels[0,0]==-1
    assert b.labels[20,20]==0

def test_obstacle_wins_over_free_update():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1)
    b.update_points(np.array([[1.,0.,.4]]),np.array([0.,0.,.4]),0.)
    assert b.labels[30,20]==1
    b.update_points(np.array([[1.,0.,0.]]),np.array([0.,0.,.4]),.1)
    assert b.labels[30,20]==1

def test_nan_and_body_hits_never_mark_scene_free():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1)
    b.update_points(np.array([[np.nan,0,0],[.1,0,.4]]),np.array([0.,0.,.4]),0.)
    assert np.all(b.labels==-1)

def test_unknown_ring_prevents_unsafe_frontier():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1);b.seed_body(np.array([0.,0.]))
    path,reason=b.plan(np.array([0.,0.]),np.array([1.,0.]))
    assert path is None and reason=='no_known_free_route'

def test_whole_known_free_map_can_reach_goal():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1);b.labels[:]=0
    p,reason=b.plan(np.array([0.,0.]),np.array([1.,0.]))
    assert p is not None and np.allclose(p[-1],[1.,0.])

def test_history_persists_when_not_seen_again():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1)
    b.update_points(np.array([[1.,0.,.4]]),np.array([0.,0.,.4]),0.)
    b.update_points(np.empty((0,3)),np.array([0.,0.,.4]),.1)
    assert b.labels[30,20]==1

def test_partial_unknown_tube_is_not_certified():
    b=m.BeliefGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1)
    b.labels[20:31,20]=0
    p,reason=b.plan(np.array([0.,0.]),np.array([1.,0.]))
    assert p is None

def test_direction_reversal_requires_five_low_samples_and_rebound_clears():
    d=m.DirectionLatch();assert d.request(1,.0)==1
    assert d.request(-1,.1)==0
    for _ in range(4):assert d.request(-1,.01)==0
    assert d.request(-1,.1)==0
    for _ in range(4):assert d.request(-1,.01)==0
    assert d.request(-1,.01)==-1
