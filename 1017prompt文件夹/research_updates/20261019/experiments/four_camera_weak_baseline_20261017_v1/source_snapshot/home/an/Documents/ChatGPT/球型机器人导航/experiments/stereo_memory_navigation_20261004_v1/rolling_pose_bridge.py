"""One initial known gauge + causal frozen rolling records; no world truth lookup."""
import numpy as np

def rigid_rotation(value):
 R=np.asarray(value,float)
 if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1,atol=1e-6):raise ValueError('finite rigid rotation required')
 return R.copy()

def vector(value):
 v=np.asarray(value,float)
 if v.shape!=(3,) or not np.isfinite(v).all():raise ValueError('finite vector required')
 return v.copy()

class RollingPoseBridge:
 def __init__(self,initial_world_position,initial_world_rotation):
  self.p0=vector(initial_world_position);self.R0=rigid_rotation(initial_world_rotation);self.history={};self.latest=None
 def consume(self,record):
  if set(record)!={'time_s','position','rotation','velocity','shell_omega_gauge'}:raise ValueError('rolling record whitelist required')
  t=float(record['time_s'])
  if not np.isfinite(t) or (self.latest is not None and t<=self.latest):raise ValueError('strictly increasing finite time')
  p=vector(record['position']);R=rigid_rotation(record['rotation']);v=vector(record['velocity']);vector(record['shell_omega_gauge'])
  self.history[t]=(self.p0+self.R0@p,self.R0@R,self.R0@v);self.latest=t
 def at(self,time_s):
  t=float(time_s)
  if not np.isfinite(t) or self.latest is None or t>self.latest or t not in self.history:raise ValueError('exact available current/past exposure time required')
  p,R,v=self.history[t];out=dict(time_s=t,root_position_world_m=p.copy(),root_rotation_world=R.copy(),velocity_world_m_s=v.copy(),yaw_world_rad=float(np.arctan2(R[1,0],R[0,0])),initial_gauge_privileged=True,pose_estimated=True)
  body_from_optical=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
  for eye,y in [('left',.3),('right',-.3)]:
   T=np.eye(4);T[:3,:3]=R@body_from_optical@np.diag([1,-1,-1]);T[:3,3]=p+R@np.array([.2,y,.05]);out[eye+'_camera_usd_world']=T
  return out
