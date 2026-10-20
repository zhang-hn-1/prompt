"""Diagnostic only. Current measured course, qualified by five past samples.
No fallback request/latch inference, no predictor, no actor installation.
"""
import numpy as np

def derive_motion_heading(now,time_s,velocity_xy,main_velocity):
 t=np.asarray(time_s,float)
 if t.ndim!=1 or not np.isfinite(t).all() or not np.isfinite(now) or np.any(t>now+1e-8) or np.any(np.diff(t)<=0):raise ValueError('causal monotonic clocks before motion payload')
 if len(t)<5:return None
 if abs(t[-1]-now)>1e-6 or not np.allclose(np.diff(t[-5:]),.1,atol=1e-6,rtol=0):raise ValueError('current 10Hz measured history required')
 v=np.asarray(velocity_xy,float);m=np.asarray(main_velocity,float)
 if v.shape!=(len(t),2) or m.shape!=(len(t),) or not np.isfinite(v).all() or not np.isfinite(m).all():raise ValueError('finite measured motion arrays')
 v=v[-5:];m=m[-5:]
 if np.any(np.linalg.norm(v,axis=1)<=.05) or np.any(np.abs(m)<.1) or np.any(np.sign(m)!=np.sign(m[-1])):return None
 yaw=np.arctan2(v[-1,1],v[-1,0])-(np.pi if m[-1]<0 else 0.)
 return float((yaw+np.pi)%(2*np.pi)-np.pi)
