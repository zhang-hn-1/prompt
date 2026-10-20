import numpy as np
import pytest
from test_current_stereo_memory_ray_v1 import args

@pytest.mark.parametrize('z,status',[(.75,'support_compatible'),(1.25,'support_compatible'),(np.nextafter(.75,0),'occluded'),(np.nextafter(1.25,2),'contradiction')])
def test_fixed_margin_boundary(z,status):
 from current_stereo_memory_ray_v2 import ray_evidence
 a=args();T=a['current_pose'];ray=np.array([.5/584.8888888888889,-.5/584.8888888888889,-1.])
 a['old_point']=T[:3,3]+ray;a['old_sigma']=.075;a['current_packet']['sigma_m'][0]=0.;a['current_packet']['world'][0]=T[:3,3]+ray*z
 assert ray_evidence(**a,pose_time_s=.5)['status']==status

@pytest.mark.parametrize('bad',['negative_id','floating_id','duplicate','negative_sigma','nonfinite_world','reflection'])
def test_invalid_current_packet_or_pose_rejected(bad):
 from current_stereo_memory_ray_v2 import ray_evidence
 a=args();p=a['current_packet']
 if bad=='negative_id':p['query_pixel_id'][0]=-1
 if bad=='floating_id':p['query_pixel_id']=p['query_pixel_id'].astype(float)
 if bad=='duplicate':
  for key in ['world','sigma_m','query_pixel_id']:p[key]=np.concatenate([p[key],p[key]])
 if bad=='negative_sigma':p['sigma_m'][0]=-.1
 if bad=='nonfinite_world':p['world'][0,0]=np.nan
 if bad=='reflection':a['current_pose'][0,0]=-1.
 with pytest.raises(ValueError):ray_evidence(**a,pose_time_s=.5)

def test_future_pose_clock_rejected_before_point_payload():
 from current_stereo_memory_ray_v2 import ray_evidence
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('future pose leaked into point read')
 a=args();a['old_point']=Poison()
 with pytest.raises(ValueError):ray_evidence(**a,pose_time_s=1.)
