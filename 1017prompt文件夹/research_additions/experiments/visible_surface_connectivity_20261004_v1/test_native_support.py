import numpy as np
import surface_support as s

def data():
    # 9x9 native hits, .01m spacing; root far away so exclusion cannot mask test.
    y,x=np.mgrid[0:9,0:9];pts=np.stack((1+x*.01,y*.01,np.zeros((9,9))),axis=-1)
    return pts,np.ones((9,9),bool),np.array([0.,0.,.4])

def test_native_floor_quad_allows_supported_interpolation():
    p,v,r=data();assert s.quad_support(p,v,r,4).all()

def test_unsampled_native_invalid_hole_blocks_quad():
    p,v,r=data();v[2,2]=False
    assert not s.quad_support(p,v,r,4)[0,0]

def test_unsampled_native_obstacle_blocks_quad():
    p,v,r=data();p[2,2,2]=.2
    assert not s.quad_support(p,v,r,4)[0,0]

def test_unsampled_native_nan_blocks_quad():
    p,v,r=data();p[2,2]=np.nan
    assert not s.quad_support(p,v,r,4)[0,0]

def test_near_body_support_blocks_quad():
    p,v,r=data();p[2,2]=[.1,0,0]
    assert not s.quad_support(p,v,r,4)[0,0]
