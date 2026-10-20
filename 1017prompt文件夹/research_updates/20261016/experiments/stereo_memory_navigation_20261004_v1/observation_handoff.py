"""One initial observation sequence followed by unchanged B0. No depth truth."""
import numpy as np
from retreat_observation_schedule import request as frozen_observation_request

class ObservationHandoff:
 def __init__(self):self.low_count=0;self.handed_off=False;self.time=-np.inf;self.handoff_time_s=None
 def choose(self,time_s,measured_XY_speed,packet,nominal_request):
  t=float(time_s);v=float(measured_XY_speed);a=np.asarray(nominal_request).copy()
  if not np.isfinite(t) or t<self.time or not np.isfinite(v) or v<0 or a.shape!=(10,) or not np.isfinite(a).all():raise ValueError('current finite measurements required')
  self.time=t
  if self.handed_off:return a
  self.low_count=self.low_count+1 if t>=12-1e-6 and v<.05 else 0
  if t<12-1e-6:return frozen_observation_request(t)
  ready=False
  if t>=18-1e-6 and self.low_count>=5 and packet is not None:
   qt=float(packet['time_s'])
   if np.isfinite(qt) and 0<=t-qt<=.5:
    xyz=np.asarray(packet['world'],float)
    if xyz.ndim!=2 or xyz.shape[1]!=3:raise ValueError('current provisional point shape')
    ready=int((np.isfinite(xyz).all(1)&(xyz[:,2]>.05)&(xyz[:,2]<=1.5)).sum())>=100
  if ready:
   self.handed_off=True;self.handoff_time_s=t;return a
  return np.zeros(10,np.float32)
