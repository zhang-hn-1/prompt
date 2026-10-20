"""Breaks caught: wrong rectification sign/eye order and uncertified correspondence."""
import numpy as np
import stereo_geometry as s

def test_vertical_native_baseline_rectifies_to_positive_horizontal_disparity():
    # A point with native (u,v)=(2,4) in left and (2,2) in right.
    left=np.zeros((6,8,3),np.uint8);right=left.copy()
    left[4,2]=255;right[2,2]=255
    l,r=s.rectify_pair(left,right)
    assert l.shape==(8,6,3)
    assert np.all(l[5,4]==255) and np.all(r[5,2]==255)

def test_metric_depth_and_roundtrip_to_native_pixels():
    d=np.full((8,480),117.,np.float32);dr=-d
    mask=np.zeros_like(d,bool)
    depth,valid=s.disparity_depth(d,dr,mask,mask)
    assert np.isclose(depth[2,300],3.,atol=.002)
    assert valid[2,300] and not valid[2,100]
    a=np.arange(12).reshape(3,4)
    assert np.array_equal(s.restore_native(np.rot90(a)),a)

def test_bilateral_body_mask_and_inconsistent_match_are_rejected():
    d=np.full((8,480),117.,np.float32);dr=-d
    lm=np.zeros_like(d,bool);rm=lm.copy()
    lm[0,300]=True;rm[1,183]=True;dr[2,183]=-110
    _,valid=s.disparity_depth(d,dr,lm,rm)
    assert not valid[0,300] and not valid[1,300] and not valid[2,300]
    assert valid[3,300]

def test_invalid_disparity_is_unknown():
    d=np.zeros((8,480),np.float32);dr=-np.ones_like(d)
    d[0,300]=np.nan;d[1,300]=-1
    depth,valid=s.disparity_depth(d,dr,np.zeros_like(d,bool),np.zeros_like(d,bool))
    assert not valid.any() and np.isnan(depth).all()
