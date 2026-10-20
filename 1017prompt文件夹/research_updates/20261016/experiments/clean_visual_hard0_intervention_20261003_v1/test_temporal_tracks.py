"""Meaningful geometry/causality tests; depth/map never an input."""
import numpy as np
from temporal_tracks import triangulate_matches

def test_known_world_point_recovered_from_two_past_only_views():
    # A real 0.3m translation and a world point at (1, .2, 4).
    p=np.array([[1.,.2,4.]])
    a=(np.zeros(3),np.eye(3));b=(np.array([.3,0,0]),np.eye(3))
    uv=lambda pose:np.column_stack((319.5+584.8888889*((p-pose[0])@pose[1])[:,0]/((p-pose[0])@pose[1])[:,2],239.5+584.8888889*((p-pose[0])@pose[1])[:,1]/((p-pose[0])@pose[1])[:,2]))
    points,accepted=triangulate_matches(uv(a),a,uv(b),b)
    assert accepted.tolist()==[True]
    np.testing.assert_allclose(points,p,atol=1e-6)

def test_epipolar_inconsistent_feature_cannot_become_obstacle():
    a=(np.zeros(3),np.eye(3));b=(np.array([.3,0,0]),np.eye(3))
    _,accepted=triangulate_matches(np.array([[400.,240.]]),a,np.array([[355.,260.]]),b)
    assert not accepted.any()

def test_negligible_baseline_not_measurable_even_if_images_match():
    a=(np.zeros(3),np.eye(3));b=(np.array([.001,0,0]),np.eye(3))
    _,accepted=triangulate_matches(np.array([[400.,240.]]),a,np.array([[399.9,240.]]),b)
    assert not accepted.any()
