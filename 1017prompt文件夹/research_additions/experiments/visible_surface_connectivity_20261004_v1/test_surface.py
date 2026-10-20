import numpy as np
from local_map import BeliefGrid

def grid():
    return BeliefGrid([-2.,-2.],shape=(50,50))

def mesh(x=1.,side=.2):
    return np.array([[[x,-.1,0],[x+side,-.1,0]],[[x,.1,0],[x+side,.1,0]]])

def update(b,m,t=0.):
    b.update_points(m.reshape(-1,3),np.array([0.,0.,.4]),t,ground_meshes=[m])

def test_supported_patch_interior_is_observed_free():
    b=grid();m=mesh(side=.1);update(b,m)
    assert b.labels[30,20]==0
    assert b.labels[30,21]==0
    assert b.labels[29,20]==-1

def test_depth_discontinuity_is_not_bridged():
    b=grid();m=mesh(side=.8);update(b,m)
    assert b.labels[34,20]==-1

def test_masked_neighbor_does_not_form_ground_patch():
    b=grid();m=mesh(side=.1);m[0,0]=np.nan;m[1,1]=np.nan;update(b,m)
    assert b.labels[30,20]==-1

def test_positive_height_neighbor_cannot_certify_floor():
    b=grid();m=mesh(side=.1);m[0,0,2]=.5;m[1,1,2]=.5;update(b,m)
    assert b.labels[30,20]==-1

def test_obstacle_has_precedence_over_later_surface():
    b=grid();b.update_points(np.array([[1.,0.,.4]]),np.array([0.,0.,.4]),0.)
    update(b,mesh(side=.1),.1)
    assert b.labels[30,20]==1

def test_surface_cannot_bridge_body_to_far_observed_floor():
    b=grid();update(b,mesh(side=.1));b.seed_body(np.zeros(2))
    assert b.plan(np.zeros(2),np.array([2.,0.]))[0] is None
    assert b.labels[25,20]==-1

def test_single_vertex_or_empty_frame_does_not_fill_area():
    b=grid();m=np.full((2,2,3),np.nan);m[0,0]=[1.,0.,0.];update(b,m)
    assert (b.labels==0).sum()==1
    before=b.labels.copy();b.update_points(np.empty((0,3)),np.array([0.,0.,.4]),.1,ground_meshes=[])
    assert np.array_equal(before,b.labels)
