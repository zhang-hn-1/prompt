import numpy as np
import pytest

def fixture():
 shape=(480,640);z=np.ones(shape);d=np.full(shape,584.8888888888889*.6);confidence=np.zeros(shape,bool)
 confidence[400,320]=True;confidence[401,321]=True
 prediction=dict(depth_m=z,confidence=confidence,disparity_native_px=d,frame=5,time_s=.5)
 pose=dict(left_camera_usd_world=np.eye(4),right_camera_usd_world=np.eye(4),root_position_world_m=np.array([10.,0.,0.]),time_s=.5)
 return prediction,pose,np.zeros(shape,bool),np.zeros(shape,bool)

def test_dense_preserves_sparse_endpoints_exact_and_adds_valid_odd_pixel():
 from dense_current_source_packet_v1 import dense_point_packet
 from dense_source_points import point_packet
 a=fixture();sparse=point_packet(*a);dense=dense_point_packet(*a)
 assert dense['query_pixel_id'].tolist()==[400*640+320,401*640+321]
 for key in ['world','sigma_m','query_pixel_id','partner_pixel_id']:
  assert np.array_equal(dense[key][:1],sparse[key])
 assert not dense['certified_free'] and not dense['source_model_admitted']

def test_partner_body_mask_rejects_odd_pixel_without_inventing_support():
 from dense_current_source_packet_v1 import dense_point_packet
 a=fixture();a[3][int(np.rint(401-584.8888888888889*.6)),321]=True
 assert dense_point_packet(*a)['query_pixel_id'].tolist()==[400*640+320]

def test_future_source_rejected_before_depth_read():
 from dense_current_source_packet_v1 import dense_point_packet
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('future depth touched')
 a=fixture();a[0]['time_s']=1.;a[0]['depth_m']=Poison()
 with pytest.raises(ValueError):dense_point_packet(*a)
