import numpy as np,pytest
from paired_current_evidence_v1 import qualify_packet

def fixture():
 rgb=np.ones((480,640,3),np.uint8);mask=np.zeros((480,640),bool);p=dict(world=np.array([[1,0,.5],[2,0,.5]]),sigma_m=np.array([.03,.04]),query_pixel_id=np.array([641,642]),partner_pixel_id=np.array([1281,1282]),frame=5,time_s=.5,frame_width=640,pose_sigma_m=0.,source_model_admitted=False,certified_free=False)
 return p,rgb,rgb.copy(),mask,mask.copy()
def test_both_current_eyes_required_raw_and_unknown_preserved():
 p,l,r,lm,rm=fixture();r[2,2]=0;a=qualify_packet(p,l,r,lm,rm,.5,.5,5)
 assert a['eligible'].tolist()==[True,False];assert len(a['eligible_packet']['world'])==1 and len(a['unknown_packet']['world'])==1;assert len(p['world'])==2
 assert not a['certified_free'] and not a['source_model_admitted'] and not a['measured_geometry_admitted']
def test_black_is_unknown_and_one_nonzero_channel_not_intensity_threshold():
 p,l,r,lm,rm=fixture();l[1,1]=[0,0,0];l[1,2]=[0,0,1];a=qualify_packet(p,l,r,lm,rm,.5,.5,5);assert a['eligible'].tolist()==[False,True]
def test_original_CAD_and_partner_ID_used():
 p,l,r,lm,rm=fixture();rm[2,1]=True;a=qualify_packet(p,l,r,lm,rm,.5,.5,5);assert a['eligible'].tolist()==[False,True]
def test_future_or_wrong_exposure_rejected_before_payload():
 p,l,r,lm,rm=fixture();p['world']=object()
 with pytest.raises(ValueError):qualify_packet(p,l,r,lm,rm,.6,.5,5)
 with pytest.raises(ValueError):qualify_packet(p,l,r,lm,rm,.5,.5,6)
def test_pixel_bounds_native_shape_boolean_mask_enforced():
 p,l,r,lm,rm=fixture();p['partner_pixel_id'][1]=640*480
 with pytest.raises(ValueError):qualify_packet(p,l,r,lm,rm,.5,.5,5)
 p,l,r,lm,rm=fixture()
 with pytest.raises(ValueError):qualify_packet(p,l[:2],r,lm,rm,.5,.5,5)
 with pytest.raises(ValueError):qualify_packet(p,l,r,lm.astype(float),rm,.5,.5,5)
def test_returned_packets_detached_and_empty_allowed():
 p,l,r,lm,rm=fixture();a=qualify_packet(p,l,r,lm,rm,.5,.5,5);a['eligible_packet']['world'][0,0]=99;assert p['world'][0,0]==1
 p,l,r,lm,rm=fixture();l.fill(0);a=qualify_packet(p,l,r,lm,rm,.5,.5,5);assert a['eligible_packet']['world'].shape==(0,3) and len(a['unknown_packet']['world'])==2

def test_empty_packet_roles_and_negative_exposure_rejected():
 p,l,r,lm,rm=fixture()
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:p[k]=p[k][:0]
 a=qualify_packet(p,l,r,lm,rm,.5,.5,5);assert a['unknown_packet']['evidence_role']=='UNKNOWN_unsupported'
 p['frame']=-5;p['time_s']=-.5
 with pytest.raises(ValueError):qualify_packet(p,l,r,lm,rm,-.5,-.5,-5)
