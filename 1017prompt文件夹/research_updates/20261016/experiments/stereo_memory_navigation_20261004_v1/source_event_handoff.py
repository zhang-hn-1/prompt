"""One readiness interface: historical source confirmation, fresh current source."""
import numpy as np
from observation_handoff import ObservationHandoff,frozen_observation_request
class SourceEventHandoff(ObservationHandoff):
 def choose(self,time_s,measured_XY_speed,packet,nominal_request,event=None,*,current_frame=None):
  t=float(time_s);v=float(measured_XY_speed);a=np.asarray(nominal_request).copy();frame=int(round(t*10)) if current_frame is None else int(current_frame)
  if not np.isfinite(t) or t<self.time or not np.isfinite(v) or v<0 or a.shape!=(10,) or not np.isfinite(a).all():raise ValueError('current finite measurements required')
  if event is not None:
   et=float(event['time_s']);ef=float(event['frame'])
   if not np.isfinite(et) or et>t+1e-9 or not np.isfinite(ef) or ef<0 or int(ef)!=ef or ef>frame:raise ValueError('historical event clock required before payload')
  self.time=t
  if self.handed_off:return a
  self.low_count=self.low_count+1 if t>=12-1e-6 and v<.05 else 0
  if t<12-1e-6:return frozen_observation_request(t)
  ready=False
  if self.low_count>=5 and event is not None and event['confirmed'] is True and packet is not None:
   qt=float(packet['time_s']);pf=float(packet['frame'])
   if not np.isfinite(qt) or qt>t+1e-9 or not np.isfinite(pf) or int(pf)!=pf or pf>frame:raise ValueError('current packet clock required before payload')
   if 0<=t-qt<=.5 and pf>=ef:
    xyz=np.asarray(packet['world'],float)
    if xyz.ndim!=2 or xyz.shape[1]!=3:raise ValueError('current provisional point shape')
    ready=int((np.isfinite(xyz).all(1)&(xyz[:,2]>.05)&(xyz[:,2]<=1.5)).sum())>=100
  if ready:self.handed_off=True;self.handoff_time_s=t;return a
  return np.zeros(10,np.float32)
