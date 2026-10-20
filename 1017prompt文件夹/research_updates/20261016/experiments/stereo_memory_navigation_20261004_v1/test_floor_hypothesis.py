import numpy as np
import pytest
from floor_hypothesis import predict_floor

def packet(i=1,eye='left',x=0):
    pose=np.eye(4);pose[:3,3]=[x,0,1.]
    yy,xx=np.mgrid[:32,:32];rgb=np.repeat((((xx+3*yy)%11)*20).astype(np.uint8)[...,None],3,2)
    return dict(frame=i,time_s=i*.1,eye=eye,pose=pose,rgb=rgb,mask=np.zeros((32,32),bool))
def opts():return dict(focal=40,cx=15.5,cy=15.5,patch_radius=1,min_sources=1,parallax_min=0.,texture_min=3.,max_cost=.2,depth_min=.5,depth_max=3.)
def test_identity_floor_projection_and_pixels():
    r=packet();s=packet(eye='right');p=predict_floor(r,[s],**opts());assert p['confidence'][16,16];assert p['plane_axis_depth_m'][16,16]==pytest.approx(1);assert p['cost'][16,16]<1e-5

def test_textureless_is_not_observation():
    r=packet();s=packet(eye='right');r['rgb'][:]=80;s['rgb'][:]=80;assert not predict_floor(r,[s],**opts())['confidence'].any()
def test_masked_source_is_not_evidence():
    r=packet();s=packet(eye='right');s['mask'][:]=True;assert not predict_floor(r,[s],**opts())['confidence'].any()
def test_future_payload_never_read():
    class Poison(dict):
        def __getitem__(self,k):
            if k!='time_s':raise AssertionError('future payload')
            return .2
    r=packet();s=packet(eye='right');a=predict_floor(r,[s],**opts());b=predict_floor(r,[s,Poison()],**opts());assert np.array_equal(a['confidence'],b['confidence']);assert np.array_equal(a['cost'],b['cost'])
def test_plane_behind_camera_rejected():
    r=packet();s=packet(eye='right');r['pose'][2,3]=-1;s['pose'][2,3]=-1;assert not predict_floor(r,[s],**opts())['confidence'].any()
def test_source_count_not_invented():
    r=packet();s=packet(eye='right');o=opts();o['min_sources']=2;assert not predict_floor(r,[s],**o)['confidence'].any()

def test_displaced_parallel_view_plane_homography_literal():
    r=packet();s=packet(eye='right',x=.1);s['rgb']=np.roll(r['rgb'],-4,axis=1)
    o=opts();o['parallax_min']=.5;p=predict_floor(r,[s],**o)
    assert p['confidence'][16,16];assert p['cost'][16,16]<1e-5;assert p['view_bits'][16,16]==1
