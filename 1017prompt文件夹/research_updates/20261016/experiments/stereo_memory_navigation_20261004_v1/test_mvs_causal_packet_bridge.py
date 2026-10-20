import numpy as np,pytest
from mvs_causal_packet_bridge import build_request

def packet(t,tx=0):
 T=np.eye(4);T[0,3]=tx
 return dict(time_s=t,pose=T,rgb=np.zeros((480,640,3),np.uint8),mask=np.zeros((480,640),bool))

def test_optical_positive_depth_and_pixel_centre_conversion():
 r=build_request(packet(2),[packet(1,.1),packet(0,.2)],2);K=r['cur_intrinsics'];T=r['cur_pose'];pixel=np.array([320.,240.,1.])
 assert np.allclose(np.linalg.inv(K[:3,:3])@pixel,[0,0,1])
 assert np.allclose(T[:3,:3]@[0,0,1],[0,0,-1])
 assert np.allclose(K[:2,2],[320,240])

def test_request_does_not_mutate_captured_inputs_or_share_payloads():
 p=packet(2);q=[packet(1,.1),packet(0,.2)];r=build_request(p,q,2);r['cur_image'][0,0]=123;r['cur_pose'][0,3]=17;r['current_mask'][0,0]=True
 assert not p['rgb'].any() and p['pose'][0,3]==0 and not p['mask'].any()
 assert r['src_pose'].shape==(2,4,4)

def test_future_or_noncurrent_clock_rejected_before_any_image_read():
 class Trap(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('payload read before causality check')
   return super().__getitem__(k)
 with pytest.raises(ValueError):build_request(Trap(time_s=2),[Trap(time_s=2.1),Trap(time_s=0)],2)
 with pytest.raises(ValueError):build_request(Trap(time_s=1),[Trap(time_s=0),Trap(time_s=-1)],2)

def test_fixed_two_distinct_past_sources_and_calibration_shape():
 with pytest.raises(ValueError):build_request(packet(2),[packet(1),packet(1)],2)
 p=packet(2);p['rgb']=np.zeros((240,320,3),np.uint8)
 with pytest.raises(ValueError):build_request(p,[packet(1),packet(0)],2)
