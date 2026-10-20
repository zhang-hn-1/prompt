import numpy as np,cv2,pytest
from cad_rgbd_odometry import estimate_rgbd_increment
K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
def fixture():
 xy=np.random.default_rng(7).integers([80,80],[550,400],(32,2)).astype(float);z=np.linspace(1.8,3.8,32);points=np.c_[xy,np.ones(32)]@np.linalg.inv(K).T*z[:,None];R=cv2.Rodrigues(np.array([.025,-.04,.01]))[0];t=np.array([-.03,.02,.01]);cam=points@R.T+t;q=cam@K.T;q=q[:,:2]/q[:,2,None];desc=np.random.default_rng(88).integers(0,256,(32,32),dtype=np.uint8);d=np.full((480,640),np.nan);v=np.zeros((480,640),bool);ix=xy.astype(int);d[ix[:,1],ix[:,0]]=z;v[ix[:,1],ix[:,0]]=True;T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
 return dict(time_s=0.,xy=xy,desc=desc,depth_m=d,valid=v),dict(time_s=.5,xy=q,desc=desc.copy()),T
def test_literal_metric_motion_previous_depth_only():
 p,c,T=fixture();r=estimate_rgbd_increment(p,c,K);assert r['accepted'];np.testing.assert_allclose(r['transform'],T,atol=1e-5)
def test_invalid_or_masked_depth_not_landmark_and_no_identity_filling():
 p,c,T=fixture();p['valid'][:]=False;r=estimate_rgbd_increment(p,c,K);assert not r['accepted'] and r['transform'] is None and r['landmarks']==0
def test_unrelated_truth_inputs_ignored_and_inputs_immutable():
 p,c,T=fixture();before=p['depth_m'].copy();r=estimate_rgbd_increment(p,c,K);p['pose']='forbidden';c['renderer_depth']='forbidden';assert estimate_rgbd_increment(p,c,K)==r;np.testing.assert_equal(before,p['depth_m'])
def test_future_bad_time_rejected_before_payload():
 class Bad(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('futurepayload');return super().__getitem__(k)
   return super().__getitem__(k)
 for t in (0.,-.1,float('nan')):
  with pytest.raises(ValueError):estimate_rgbd_increment(Bad(time_s=0.),Bad(time_s=t),K)
