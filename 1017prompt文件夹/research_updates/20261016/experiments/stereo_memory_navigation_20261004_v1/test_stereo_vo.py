import numpy as np
import cv2
from stereo_vo import temporal_matches,pose_from_matches,PoseChain,estimate_increment
import pytest

K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])

def fixture():
    pts=np.random.default_rng(73).uniform([-1,-.5,2],[1,.5,3],(32,3))
    rv=np.array([.025,-.04,.01]);R=cv2.Rodrigues(rv)[0];t=np.array([-.03,.02,.01]);cam=pts@R.T+t
    px=cam@K.T;px=px[:,:2]/px[:,2,None];T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    return pts,px,T

def test_metric_camera_motion_from_independent_hand_projected_3d_points():
    pts,px,T=fixture();r=pose_from_matches(pts,px,K)
    assert r['accepted'];np.testing.assert_allclose(r['transform'],T,atol=1e-5)

def test_outlier_observations_are_not_used_as_metric_landmarks():
    pts,px,T=fixture();px[:8]+=np.array([100,-130]);r=pose_from_matches(pts,px,K)
    assert r['accepted'];assert len(r['inliers'])==24;np.testing.assert_allclose(r['transform'],T,atol=1e-5)

def test_too_few_nonfinite_and_collinear_landmarks_reject():
    pts,px,_=fixture()
    for p,q in [(pts[:7],px[:7]),(pts*np.nan,px),(np.tile([0,0,2],(32,1)),px)]:
        assert not pose_from_matches(p,q,K)['accepted']

def test_temporal_mutual_ratio_literal_unique_and_ambiguous_codes():
    desc=np.array([np.zeros(32,np.uint8),np.full(32,255,np.uint8),np.full(32,85,np.uint8)])
    xy=np.zeros((3,2));assert temporal_matches(xy,desc,xy,desc)==[(0,0),(1,1),(2,2)]
    duplicated=np.concatenate([desc,desc[:1]],axis=0)
    assert (0,0) not in temporal_matches(xy,desc,np.zeros((4,2)),duplicated)

def test_transform_accumulation_is_inverse_and_failure_never_silently_resets():
    chain=PoseChain();T=np.eye(4);T[0,3]=-.1
    assert abs(chain.update(T)[0,3]-.1)<1e-9
    assert chain.update(None) is None
    assert chain.update(T) is None

def pair_features():
    pts,px,T=fixture();pts[:,1]=pts[:,2]*np.repeat(np.array([-.15,-.05,.05,.15]),8)
    desc=np.random.default_rng(88).integers(0,256,(32,32),dtype=np.uint8)
    def project(p):
        a=p@K.T;return a[:,:2]/a[:,2,None]
    prev=dict(time_s=0.,left_xy=project(pts),left_desc=desc,right_xy=project(pts-np.array([.6,0,0])),right_desc=desc.copy())
    cur=dict(time_s=.1,left_xy=project(pts@T[:3,:3].T+T[:3,3]),left_desc=desc.copy())
    rig=np.eye(4);rig[0,3]=.6
    return prev,cur,rig,T

def test_complete_feature_to_stereo_to_temporal_to_pose_is_metric_and_truth_free():
    prev,cur,rig,T=pair_features();r=estimate_increment(prev,cur,K,rig)
    assert r['accepted'];np.testing.assert_allclose(r['transform'],T,atol=1e-5)
    prev['pose']='forbidden truth';cur['renderer_depth']='forbidden depth'
    assert estimate_increment(prev,cur,K,rig)==r

def test_invalid_time_is_rejected_before_any_feature_payload():
    class Bad(dict):
        def __getitem__(self,k):
            if k!='time_s':raise AssertionError('read bad time feature payload')
            return super().__getitem__(k)
    for a,b in [(0.,0.),(.1,0.),(0.,float('nan'))]:
        with pytest.raises(ValueError):estimate_increment(Bad(time_s=a),Bad(time_s=b),K,np.eye(4))
