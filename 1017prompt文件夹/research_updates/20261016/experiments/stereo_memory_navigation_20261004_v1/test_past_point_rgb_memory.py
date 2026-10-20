import numpy as np,pytest
from past_point_rgb_memory import PastPointRGBMemory

def fixture():
 rng=np.random.default_rng(7);im=rng.integers(0,256,(480,640,3),dtype=np.uint8);pixel=240*640+320;p=np.array([[(320-319.5)/584.8888888888889,(240-239.5)/584.8888888888889,1.]])
 # Native optical->world USD conversion used by actual capture.
 T=np.diag([1.,-1.,-1.,1.]);world=p.copy();rgb=dict(left_rgb=im,right_rgb=im,time_s=0.);source=dict(world=world,query_pixel_id=np.array([pixel]),partner_pixel_id=np.array([pixel]),frame=0,time_s=0.,source_model_admitted=False);pose=dict(left_camera_usd_world=T,right_camera_usd_world=T,time_s=1.);mask=dict(left=np.zeros((480,640),bool),right=np.zeros((480,640),bool));return source,rgb,pose,mask

def test_identical_current_appearance_preserves_old_geometry():
 s,r,p,m=fixture();m0=PastPointRGBMemory();m0.seed(s,r,0,qualified=True);d=m0.query(dict(r,time_s=1.),p,m,1.);assert d['appearance_corroborated'].tolist()==[True];assert d['source_geometry_time_s']==0 and d['current_appearance_time_s']==1;np.testing.assert_equal(d['past_world'],s['world']);assert not d['certified_free']

def test_replay_expiry_cannot_refresh_source():
 s,r,p,m=fixture();a=PastPointRGBMemory();a.seed(s,r,0,qualified=True);x=a.query(dict(r,time_s=1),p,m,1);y=a.query(dict(r,time_s=1),p,m,1);np.testing.assert_equal(x['appearance_corroborated'],y['appearance_corroborated']);z=a.query(dict(r,time_s=11),dict(p,time_s=11),m,11);assert not z['appearance_corroborated'].any() and z['geometry_expired'] and z['source_geometry_time_s']==0

def test_future_before_payload_and_clock_mismatch():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return super().__getitem__(k)
 s,r,p,m=fixture();a=PastPointRGBMemory()
 with pytest.raises(ValueError):a.seed(Bomb(time_s=2),r,1,qualified=True)
 a.seed(s,r,0,qualified=True)
 with pytest.raises(ValueError):a.query(Bomb(time_s=2),p,m,1)
 with pytest.raises(ValueError):a.query(dict(r,time_s=1),dict(p,time_s=.5),m,1)

def test_single_seed_and_owned_geometry():
 s,r,p,m=fixture();a=PastPointRGBMemory();a.seed(s,r,0,qualified=True);s['world'][:]=9
 with pytest.raises(ValueError):a.seed(s,r,0,qualified=True)
 assert a.query(dict(r,time_s=1),p,m,1)['appearance_corroborated'].tolist()==[True]
