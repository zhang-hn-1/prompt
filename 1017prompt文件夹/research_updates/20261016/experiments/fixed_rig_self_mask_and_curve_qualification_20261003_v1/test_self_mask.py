import numpy as np
from self_mask import mask_triangles


def test_front_triangle_masks_itself_not_unrelated_pixels():
    triangles=np.array([[[-.2,-.2,1],[.2,-.2,1],[0,.2,1]]])
    mask=mask_triangles(triangles,64,48,50,31.5,23.5)
    assert mask[24,32]
    assert not mask[0,0]


def test_geometry_behind_camera_cannot_hide_the_scene():
    triangles=np.array([[[-.2,-.2,-1],[.2,-.2,-1],[0,.2,-1]]])
    assert not mask_triangles(triangles,64,48,50,31.5,23.5).any()


def test_triangle_crossing_near_plane_remains_finite():
    triangles=np.array([[[.2,-.2,1],[.3,.2,1],[.3,0,-.1]]])
    mask=mask_triangles(triangles,64,48,50,31.5,23.5)
    assert mask.dtype==bool and mask.shape==(48,64)
    assert not mask[24,0]
