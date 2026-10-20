import numpy as np,pytest

def ray(t=2.):
 T=np.diag([1.,-1.,-1.,1.]);return dict(time_s=t,frame=round(t*10),frame_width=640,world=np.array([[0.,0.,2.]]),query_pixel_id=np.array([240*640+320]),partner_pixel_id=np.array([64*640+320])),dict(time_s=t,left_camera_usd_world=T)

def test_nearer_core_vetoed_but_occluded_or_unobserved_core_preserved():
 from ray_contradiction_memory import contradict_cores
 q,p=ray();v,proof=contradict_cores(np.array([[0,0,1.],[0,0,2.5],[.5,0,1.]]),[(q,p)],2.)
 np.testing.assert_equal(v,[True,False,False]);assert proof[0][0]['query_pixel_id']==240*640+320
 assert proof[0][0]['endpoint_minus_core_Z_m']==pytest.approx(1.)

def test_future_nan_and_pose_clock_fail_before_ray_payload():
 from ray_contradiction_memory import contradict_cores
 c=np.array([[0.,0.,1.]])
 with pytest.raises(ValueError):contradict_cores(c,[({'time_s':3.},{'time_s':3.})],2.)
 with pytest.raises(ValueError):contradict_cores(c,[],float('nan'))
 q,p=ray();p['time_s']=1.5
 with pytest.raises(ValueError):contradict_cores(c,[(q,p)],2.)

def test_fractional_identity_and_nonrigid_camera_cannot_define_ray():
 from ray_contradiction_memory import contradict_cores
 q,p=ray();q['frame_width']=640.9
 with pytest.raises(ValueError):contradict_cores(np.array([[0.,0.,1.]]),[(q,p)],2.)
 q,p=ray();p['left_camera_usd_world'][0,0]=2.
 with pytest.raises(ValueError):contradict_cores(np.array([[0.,0.,1.]]),[(q,p)],2.)
