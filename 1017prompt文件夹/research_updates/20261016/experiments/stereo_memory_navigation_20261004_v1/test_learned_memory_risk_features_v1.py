import numpy as np,pytest

def packet():
 return dict(time_s=1.,frame=10,world=np.array([[1,2,.06],[1.02,2,.1],[2,3,.3]]),query_pixel_id=np.array([240*640+320,242*640+320,244*640+320]),partner_pixel_id=np.array([140*640+320,142*640+320,144*640+320]),sigma_m=np.array([.01,.02,.01]),depth_m=np.array([1.,1.2,1.4]),disparity_native_px=np.array([100.,120.,140.]),left_rgb=np.full((480,640,3),100,np.uint8),right_rgb=np.full((480,640,3),110,np.uint8))
def test_features_current_past_only_shapes_and_confirmations():
 from learned_memory_risk_features_v1 import build_features,FEATURE_NAMES
 p=packet();past=[dict(frame=5,time_s=.5,world=[[1,2,.07]])];f=build_features(p,past,1.)
 assert f.shape==(3,len(FEATURE_NAMES)) and len(FEATURE_NAMES)==23
 assert np.isfinite(f).all();np.testing.assert_array_equal(f[:,-2],[1,1,0]);np.testing.assert_allclose(f[:,-1],[.5,.5,10.])
 assert not any('absolute' in n or 'episode' in n or 'teacher' in n for n in FEATURE_NAMES)
class Poison:
 def __array__(self,*a,**k):raise AssertionError('future payload read')
def test_future_metadata_rejected_before_payload():
 from learned_memory_risk_features_v1 import build_features
 p=packet();p['time_s']=1.1;p['world']=Poison()
 with pytest.raises(ValueError):build_features(p,[],1.)
 p=packet()
 with pytest.raises(ValueError):build_features(p,[dict(time_s=1.1,frame=15,world=Poison())],1.)
def test_teacher_field_rejected_and_empty_valid():
 from learned_memory_risk_features_v1 import build_features
 p=packet();p['GT_depth']=Poison()
 with pytest.raises(ValueError):build_features(p,[],1.)
 p=packet()
 for k in ['world','query_pixel_id','partner_pixel_id','sigma_m','depth_m','disparity_native_px']:p[k]=np.asarray(p[k])[:0]
 assert build_features(p,[],1.).shape==(0,23)
