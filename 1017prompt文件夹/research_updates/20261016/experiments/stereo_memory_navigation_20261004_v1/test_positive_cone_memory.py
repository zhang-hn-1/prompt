import numpy as np
from positive_cone_memory import possible_set

def view(frame,time,centre):
 T=np.eye(4);T[:3,3]=centre
 return dict(frame=frame,time_s=time,optical_world=T,positive=np.ones((480,640),bool),self_mask=np.zeros((480,640),bool))

def test_depth_interval_baseline_and_unobserved_are_not_free():
 p=np.array([[0,0,1],[0,0,.1],[10,0,1.]])
 a=view(0,0,[0,0,0]);b=view(1,1,[.1,0,0]);r=possible_set(p,1,1,[a,b]);np.testing.assert_array_equal(r['keep'],[True,False,False]);assert not r['certified_free'];assert r['label']=='POSITIVE_CONE_MODEL_POSSIBILITY';assert r['support_count'][0]==2
 b['optical_world'][:3,3]=[.01,0,0];assert not possible_set(p,1,1,[a,b])['keep'].any()

def test_future_payload_never_read_and_duplicate_exposure_not_support():
 p=np.array([[0,0,1.]])
 a=view(0,0,[0,0,0]);b=view(1,1,[.1,0,0]);future=dict(frame=2,time_s=2,optical_world=None,positive=None,self_mask=None)
 r=possible_set(p,1,1,[a,b,future]);np.testing.assert_equal(r,possible_set(p,1,1,[a,b]));assert not possible_set(p,1,1,[a,a])['keep'].any()

def test_negative_pixels_and_self_mask_abstain():
 p=np.array([[0,0,1.]])
 a=view(0,0,[0,0,0]);b=view(1,1,[.1,0,0]);b['positive'][:]=False
 r=possible_set(p,1,1,[a,b]);assert not r['keep'].any();assert not r['certified_free']
 b['positive'][:]=True;b['self_mask'][:]=True;assert not possible_set(p,1,1,[a,b])['keep'].any()
