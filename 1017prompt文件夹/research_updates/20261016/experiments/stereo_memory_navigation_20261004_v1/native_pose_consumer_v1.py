"""Current native gyro/encoder estimate shared by registration and navigation.
Initial gauge and flat height remain labelled privileged/prior. No truth fallback.
"""
import numpy as np
from rolling_pose_bridge import vector, rigid_rotation

def current_native_pose(record, time_s):
 t=float(time_s);record_t=float(record['time_s'])
 if not np.isfinite(t) or not np.isfinite(record_t) or not record.get('available') or not record.get('live_before_next_action') or abs(record_t-t)>1e-8:
  raise ValueError('exact current live native estimate required; no truth fallback')
 p=vector(record['position_world']);R=rigid_rotation(record['rotation_world']);v=vector(record['velocity_world_m_s'])
 out=dict(time_s=t,root_position_world_m=p,root_rotation_world=R,velocity_world_m_s=v,pose_estimated=True,initial_gauge_privileged=True,flat_height_is_prior=bool(record['flat_height_is_prior']))
 body_from_optical=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
 for eye,y in [('left',.3),('right',-.3)]:
  T=np.eye(4);T[:3,:3]=R@body_from_optical@np.diag([1,-1,-1]);T[:3,3]=p+R@np.array([.2,y,.05]);out[eye+'_camera_usd_world']=T
 return out
