"""One current-measurement acquisition/stop handoff; never certifies unseen space."""
import numpy as np
class FrontViewAcquisition:
 def __init__(self):
  self.origin=None;self.time=-np.inf;self.stopped=False;self.handed_off=False;self.failed=False;self.low_count=0;self.halt_reason=None;self.handoff_time_s=None;self.nearest_RGB_point_m=None
 def choose(self,time_s,position_xy,speed_m_s,source):
  t=float(time_s);xy=np.asarray(position_xy,float);speed=float(speed_m_s)
  if not np.isfinite(t) or t<=self.time or t<0 or xy.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(speed) or speed<0:raise ValueError('strict current finite state')
  # Validate all acquisition clocks before any world payload is inspected.
  if not isinstance(source,dict) or not np.isfinite(source['time_s']) or not 0<=source['time_s']<=t:raise ValueError('source clock before payload')
  records=source['records']
  if any(not np.isfinite(r['time_s']) or not 0<=r['time_s']<=source['time_s'] for r in records):raise ValueError('source clock before payload')
  arrays=[];fresh_count=0
  for r in records:
   p=np.asarray(r['world'],float)
   if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():raise ValueError('finite current/past partial surface')
   if t-r['time_s']<=10.+1e-6:arrays.append(p)
   if r['frame']==source['frame'] and r['time_s']==source['time_s']:fresh_count+=len(p)
  if self.origin is None:self.origin=xy.copy()
  self.time=t;displacement=float(np.linalg.norm(xy-self.origin));points=np.concatenate(arrays) if arrays else np.empty((0,3));self.nearest_RGB_point_m=float(np.linalg.norm(points[:,:2]-xy,axis=1).min()) if len(points) else None
  if self.handed_off:return None
  if t>=6.-1e-6 and not self.stopped:
   if displacement>=.5:self.stopped=True;self.halt_reason='measured_displacement'
   elif self.nearest_RGB_point_m is not None and self.nearest_RGB_point_m<=.8707106781186548:self.stopped=True;self.halt_reason='current_RGB_point_proximity'
   elif t>=14.-1e-6:self.stopped=True;self.halt_reason='bounded_drive_time'
  self.low_count=self.low_count+1 if self.stopped and speed<.05 else 0
  ready=t-source['time_s']<=.5+1e-6 and fresh_count>=100
  if self.stopped and self.low_count>=5 and ready:
   self.handed_off=True;self.handoff_time_s=t;return None
  if t>=20.-1e-6:self.failed=True
  return np.tile(np.array([.5,np.pi/6] if t>=6.-1e-6 and not self.stopped else [0.,0.],np.float32),5)
 @property
 def diagnostics(self):
  return dict(stopped=self.stopped,halt_reason=self.halt_reason,low_count=self.low_count,handed_off=self.handed_off,handoff_time_s=self.handoff_time_s,failed=self.failed,nearest_RGB_point_m=self.nearest_RGB_point_m,partial_surface_not_free_volume=True,task_prior_not_observation=True)
