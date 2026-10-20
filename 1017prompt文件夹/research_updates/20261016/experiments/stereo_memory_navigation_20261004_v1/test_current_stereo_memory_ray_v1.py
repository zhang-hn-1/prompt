import numpy as np
import pytest

def args():
 T=np.eye(4);T[2,3]=1.
 f=584.8888888888889;ray=np.array([.5/f,-.5/f,-1.])
 return dict(old_point=T[:3,3]+ray*.8,old_sigma=.01,past_time_s=0.,current_time_s=.5,current_pose=T,current_packet=dict(time_s=.5,query_pixel_id=np.array([240*640+320]),world=np.array([T[:3,3]+ray]),sigma_m=np.array([.01])))

def test_deeper_matched_endpoint_is_advisory_not_free():
 from current_stereo_memory_ray_v1 import ray_evidence
 r=ray_evidence(**args());assert r['status']=='contradiction' and r['clear_memory'] is False

def test_closer_endpoint_is_occlusion_not_removal():
 from current_stereo_memory_ray_v1 import ray_evidence
 a=args();a['current_packet']['world'][0,2]=.6
 assert ray_evidence(**a)['status']=='occluded'

def test_no_matched_packet_pixel_keeps_unknown():
 from current_stereo_memory_ray_v1 import ray_evidence
 a=args();a['current_packet']['query_pixel_id'][0]=0
 assert ray_evidence(**a)['status']=='unavailable'

def test_uncertain_depth_difference_not_called_contradiction():
 from current_stereo_memory_ray_v1 import ray_evidence
 a=args();a['old_sigma']=.2
 assert ray_evidence(**a)['status']=='support_compatible'

def test_noncausal_rejected_before_payload():
 from current_stereo_memory_ray_v1 import ray_evidence
 a=args();a['current_packet']['time_s']=1.;a['current_packet']['world']=object()
 with pytest.raises(ValueError):ray_evidence(**a)
