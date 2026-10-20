"""Breaks caught: labelling exploration as observed freedom or crossing sensed hazards."""
import numpy as np
import risk_grid as r

def test_explicit_exploration_path_preserves_unknown_labels():
    g=r.ExplorationGrid(origin=(-2,-2),shape=(50,50),resolution=.1)
    path,reason=g.plan(np.array([0.,0.]),np.array([1.,0.]))
    assert path is not None and reason=='unknown_exploration'
    assert np.all(g.labels==-1)
    assert g.exploration_unknown_fraction>0

def test_observed_obstacle_clearance_is_not_removed_by_exploration():
    g=r.ExplorationGrid(origin=(-2,-2),shape=(50,50),resolution=.1)
    g.labels[27,:]=1  # x=.7m wall, no crossing with .6m inflation
    path,reason=g.plan(np.array([0.,0.]),np.array([1.5,0.]))
    assert path is None and reason=='observed_obstacle_blocks_route'

def test_known_path_needs_no_unknown_exploration():
    g=r.ExplorationGrid(origin=(-2,-2),shape=(50,50),resolution=.1);g.labels[:]=0
    path,reason=g.plan(np.array([0.,0.]),np.array([1.,0.]))
    assert path is not None and reason=='observed_route'
    assert g.exploration_unknown_fraction==0

def test_unknown_exploration_limits_actual_main_request_only():
    a=np.tile([2.,.2],5).astype(np.float32)
    limited=r.limit_action(a,.1)
    assert np.allclose(limited[::2],.3) and np.array_equal(limited[1::2],a[1::2])
    assert np.array_equal(r.limit_action(a,0.),a)
