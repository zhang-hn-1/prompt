import numpy as np,pytest
from memory_geometry_prompt import memory_geometry_prompt,world_up_pair,restore_native_prediction

def fixture(n=2):
 T=np.eye(4);T[2,3]=2.;xyz=np.array([[0.,0.,1.],[0.,0.,.5]])[:n];z=2-xyz[:,2];uv=np.tile([319.5,239.5],(n,1))
 f=dict(past_world=xyz,past_query_id=np.arange(n),past_partner_id=np.arange(n)+10,source_geometry_time_s=18.,source_frame=180,current_appearance_time_s=23.5,right_uv=uv,right_z=z,left_uv=uv,left_z=z,patch_NCC=np.ones((n,2)),patch_std=np.full((n,2,2),10.))
 p=dict(time_s=23.5,left_camera_usd_world=T,right_camera_usd_world=T);m=dict(time_s=23.5,frame=235,left=np.zeros((480,640),bool),right=np.zeros((480,640),bool))
 return f,p,m

def test_same_pixel_keeps_nearest_historical_Z_and_original_identity():
 f,p,m=fixture();d=memory_geometry_prompt(f,p,m,23.5,235,'right');assert d['prompt_depth_m'][240,320]==1.;assert d['past_point_index'][240,320]==0;assert d['source_geometry_time_s']==18 and d['source_frame']==180;assert d['nonzero_prompts']==1;assert not d['is_current_depth_observation'] and not d['certified_free']

def test_all_bilinear_patch_taps_required_not_rounded_center_only():
 f,p,m=fixture();m['right'][240,323]=True;d=memory_geometry_prompt(f,p,m,23.5,235,'right');assert d['nonzero_prompts']==0;assert d['status']=='NOT_EXERCISED_EMPTY_PROMPT'

def test_future_clock_rejected_before_geometry_or_pose_payload():
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('future payload read')
 f,p,m=fixture();f['past_world']=Trap();f['current_appearance_time_s']=23.6
 with pytest.raises(ValueError,match='clock'):memory_geometry_prompt(f,p,m,23.5,235,'right')
 f,p,m=fixture();p['right_camera_usd_world']=Trap();p['time_s']=23.6
 with pytest.raises(ValueError,match='clock'):memory_geometry_prompt(f,p,m,23.5,235,'right')

def test_expired_geometry_unknown_without_payload_read_or_age_refresh():
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('expired payload read')
 f,p,m=fixture();f['source_geometry_time_s']=0.;f['past_world']=Trap();d=memory_geometry_prompt(f,p,m,23.5,235,'right');assert d['nonzero_prompts']==0 and d['status']=='EXPIRED_HISTORICAL_GEOMETRY';assert d['source_geometry_time_s']==0.

def test_native_rotation_is_paired_depth_not_rescaled_and_restores_pixels():
 rgb=np.zeros((480,640,3),np.uint8);depth=np.zeros((480,640),np.float32);rgb[40,120]=[7,8,9];depth[40,120]=1.2345;r,z=world_up_pair(rgb,depth);assert r.shape==(640,480,3);assert z.shape==(640,480);assert (r[519,40]==[7,8,9]).all();assert z[519,40]==depth[40,120];np.testing.assert_array_equal(restore_native_prediction(z),depth)

def test_wrong_current_projection_cannot_silently_supply_old_camera_depth():
 f,p,m=fixture();f['right_z']=f['right_z']+.2
 with pytest.raises(ValueError,match='projection'):memory_geometry_prompt(f,p,m,23.5,235,'right')
