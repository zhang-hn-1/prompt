"""Bugs caught: footprint fabricates free, history forgotten, future query leaks."""
import numpy as np
import pytest
from provenance_grid import ProvenanceGrid


def make(mode='history'):
    return ProvenanceGrid(np.array([-2.,-2.]),shape=(40,40),resolution=.1,memory_mode=mode)


def test_footprint_does_not_create_observation_or_free_cells():
    g=make();g.update_points(np.empty((0,3)),np.array([0,0,.4]),6.)
    g.seed_body(np.array([0.,0.]))
    assert np.all(g.labels==-1)
    assert not np.isfinite(g.last_seen).any()
    assert g.diagnostics()['footprint_unknown']==49


def test_hidden_obstacle_persists_only_in_history_arm():
    for mode,expected in [('current',-1),('history',1)]:
        g=make(mode);g.update_points(np.array([[.8,0,.3]]),np.array([0,0,.4]),6.)
        g.update_points(np.empty((0,3)),np.array([0,0,.4]),6.1)
        i,j=g.cells([[.8,0]])[0]
        assert g.labels[i,j]==expected
        assert g.provenance()[i,j]==(4 if mode=='history' else 0)


def test_floor_expires_without_becoming_new_observation():
    g=make();g.update_points(np.array([[.8,0,0.]]),np.array([0,0,.4]),6.)
    g.update_points(np.empty((0,3)),np.array([0,0,.4]),7.)
    i,j=g.cells([[.8,0]])[0]
    assert g.labels[i,j]==0 and g.provenance()[i,j]==2
    g.update_points(np.empty((0,3)),np.array([0,0,.4]),16.1)
    assert g.labels[i,j]==-1 and g.provenance()[i,j]==0
    assert g.last_seen[i,j]==6.


def test_noncausal_update_and_unmodelled_surface_are_rejected():
    g=make();g.update_points(np.empty((0,3)),np.array([0,0,.4]),6.)
    with pytest.raises(ValueError):g.update_points(np.empty((0,3)),np.array([0,0,.4]),5.)
    with pytest.raises(ValueError):g.update_points(np.empty((0,3)),np.array([0,0,.4]),7.,ground_meshes=[np.zeros((2,2,3))])
