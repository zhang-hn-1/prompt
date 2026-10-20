import numpy as np
import pytest
from perfect_visible_points import perfect_packet

def data():
 p=np.eye(4);p[2,3]=2.;r=p.copy();r[1,3]=-.6
 return dict(left_depth_m=np.full((480,640),2.),right_depth_m=np.full((480,640),2.),left_camera_usd_world=p,right_camera_usd_world=r,root_position_world_m=np.array([0,0,2]),time_s=6.)

def test_actual_projection_and_floor():
 d=data();m=np.zeros((480,640),bool);a=perfect_packet(d,m,m,60)
 idx=np.flatnonzero(a['query_pixel_id']==240*640+320);assert len(idx)==1
 i=idx[0];assert a['partner_pixel_id'][i]==65*640+320
 assert np.allclose(a['world'][i],[(320-319.5)*2/584.8888888888889,-(240-239.5)*2/584.8888888888889,0])
 assert (a['sigma_m']==0).all() and a['privileged_source'] is True

def test_true_right_surface_and_mask_required():
 d=data();m=np.zeros((480,640),bool);d['right_depth_m'][:]=3
 assert len(perfect_packet(d,m,m,60)['world'])==0
 d=data();m[:]=True;assert len(perfect_packet(d,m,m,60)['world'])==0

def test_current_input_whitelist_and_invalid_time():
 d=data();m=np.zeros((480,640),bool);d['future_trajectory']=[]
 with pytest.raises(ValueError):perfect_packet(d,m,m,60)
 d=data();d['time_s']=float('nan')
 with pytest.raises(ValueError):perfect_packet(d,m,m,60)
