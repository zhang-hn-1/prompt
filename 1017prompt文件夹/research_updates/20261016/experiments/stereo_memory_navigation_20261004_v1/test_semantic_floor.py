import numpy as np
import pytest
from semantic_floor import upright_rgb,native_floor_mask,project_floor,validate_rgb_packet

def test_camera_rotation_inverse_for_pixel_classes():
    a=np.arange(12).reshape(3,4);rgb=np.repeat(a[...,None],3,axis=-1);up=upright_rgb(rgb);assert np.array_equal(up[...,0],np.rot90(a));mask=native_floor_mask(np.ones((4,3)),np.rot90(a)==5,.9);assert mask.shape==(3,4);assert mask[1,1] and mask.sum()==1

def test_nonfloor_highconfidence_not_selected():
    assert not native_floor_mask(np.ones((4,3)),np.zeros((4,3),bool),.9).any()
def test_uncertainty_abstains():
    assert not native_floor_mask(np.full((4,3),.89),np.ones((4,3),bool),.9).any()
def test_planeprojection_no_depthmeasurement_claim():
    pose=np.eye(4);pose[2,3]=1;mask=np.ones((9,9),bool);selfmask=np.zeros((9,9),bool)
    p=project_floor(mask,pose,selfmask,focal=20,cx=4,cy=4,patch_radius=1,depth_min=.5,depth_max=3)
    assert p['confidence'][4,4];assert p['plane_axis_depth_m'][4,4]==pytest.approx(1)
    selfmask[4,4]=True;p=project_floor(mask,pose,selfmask,focal=20,cx=4,cy=4,patch_radius=1,depth_min=.5,depth_max=3);assert not p['confidence'][4,4]
def test_plane_behind_abstains():
    pose=np.eye(4);pose[2,3]=-1;p=project_floor(np.ones((9,9),bool),pose,np.zeros((9,9),bool),focal=20,cx=4,cy=4,patch_radius=1,depth_min=.5,depth_max=3);assert not p['confidence'].any()
def test_model_packet_forbids_geometry_or_truth():
    r=dict(rgb=np.zeros((9,9,3),np.uint8),frame=0,time_s=0.,eye='left');validate_rgb_packet(r)
    with pytest.raises(ValueError):validate_rgb_packet(dict(**r,depth=np.ones((9,9))))
