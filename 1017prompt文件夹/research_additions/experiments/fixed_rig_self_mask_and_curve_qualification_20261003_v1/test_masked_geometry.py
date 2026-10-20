import numpy as np
from geometry import camera_pose,recover_depth
from test_geometry import plane_image


def inputs():
    q=np.array([1.,0,0,0]);current=camera_pose(np.zeros(2),q,'left')
    past=[camera_pose(np.array([-.3,0.]),q,'left'),camera_pose(np.array([-.6,0.]),q,'left')]
    return current,[(plane_image(*p),p) for p in past]


def test_known_body_pixels_cannot_create_depth_points():
    current,past=inputs()
    out=recover_depth(plane_image(*current),current,past,valid_mask=np.zeros((480,640),bool))
    assert len(out['points_world'])==0


def test_clear_mask_preserves_unmasked_scene_geometry():
    current,past=inputs();yy,xx=np.mgrid[80:400:20,100:540:20];uv=np.c_[xx.ravel(),yy.ravel()]
    out=recover_depth(plane_image(*current),current,past,uv=uv,valid_mask=np.ones((480,640),bool))
    assert len(out['points_world'])>=30
    assert np.percentile(abs(out['depth_m']-2.9),90)<.35
