import importlib,numpy as np,pytest

def m():return importlib.import_module('dense_source_points')
def inputs():
 z=np.ones((4,4),np.float32)*2;mask=np.zeros((4,4),bool);t=np.diag([1.,-1.,-1.,1.]);t[:3,3]=[1,-2,.4]
 return dict(depth_m=z,confidence=np.ones((4,4),bool),disparity_native_px=np.ones((4,4))*2,frame=0,time_s=0.),dict(left_camera_usd_world=t,right_camera_usd_world=t.copy(),root_position_world_m=np.array([0,0,.4]),time_s=0.),mask,mask.copy()
def test_native_vertical_partner_and_world_projection_are_literal():
 p,pose,a,b=inputs();r=m().point_packet(p,pose,a,b,focal=4,baseline=1,cx=1.5,cy=1.5)
 assert len(r['world'])==2 and np.allclose(r['world'][0],[.25,-1.75,2.4]);assert r['query_pixel_id'].tolist()==[8,10] and r['partner_pixel_id'].tolist()==[0,2]
 assert r['sigma_m'][0]==pytest.approx(2*np.sqrt(1+(.375)**2+(.125)**2))
def test_confidence_and_both_masks_cannot_be_ignored():
 p,pose,a,b=inputs();p['confidence'][2,0]=False;b[0,2]=True
 r=m().point_packet(p,pose,a,b,focal=4,baseline=1,cx=1.5,cy=1.5);assert len(r['world'])==0
@pytest.mark.parametrize('extra',['left_depth_m','future_action'])
def test_truth_or_future_action_fields_are_rejected(extra):
 p,pose,a,b=inputs();pose[extra]=np.zeros((4,4))
 with pytest.raises(ValueError):m().point_packet(p,pose,a,b,focal=4,baseline=1,cx=1.5,cy=1.5)
def test_bad_or_mismatched_time_and_pose_are_rejected():
 p,pose,a,b=inputs();pose['time_s']=1
 with pytest.raises(ValueError):m().point_packet(p,pose,a,b)
 pose['time_s']=0;p['time_s']=np.nan
 with pytest.raises(ValueError):m().point_packet(p,pose,a,b)
