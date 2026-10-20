"""Fixed 8s observation program inside an explicit prior; no learned decisions."""
import numpy as np
class SafeStartMotion:
 def __init__(self,initial_xy):
  self.initial=np.asarray(initial_xy,dtype=float)
  if self.initial.shape!=(2,) or not np.isfinite(self.initial).all():raise ValueError('finite initial XY required')
  self.last_frame=-1;self.low_count=0;self.aborted=False;self.maximum_distance=0.
 @property
 def settled(self):return self.low_count>=5
 def choose(self,frame,time_s,current_xy,actual_XY_speed_m_s):
  p=np.asarray(current_xy,dtype=float);v=float(actual_XY_speed_m_s)
  if not isinstance(frame,int) or frame<=self.last_frame or frame<0 or frame>=80 or not np.isfinite(time_s) or abs(time_s-frame*.1)>1e-5:raise ValueError('current increasing 10Hz acquisition clock required')
  if p.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(v) or v<0:raise ValueError('finite current state required')
  self.last_frame=frame;dist=float(np.linalg.norm(p-self.initial));self.maximum_distance=max(self.maximum_distance,dist)
  if dist>=.35:self.aborted=True
  drive=20<=frame<40 and not self.aborted
  self.low_count=self.low_count+1 if frame>=40 and v<.05 else 0
  return np.array([.5 if drive else 0.,0.])

def encode_request(command):
 c=np.asarray(command,dtype=np.float32)
 if c.shape!=(2,) or not (np.array_equal(c,[0,0]) or np.array_equal(c,[.5,0])):raise ValueError('frozen request pair required')
 return np.tile(c,5)

def assess_complete_states(rows):
 if len(rows)!=81:raise ValueError('81 complete captures required')
 t=np.array([r['time_s'] for r in rows]);p=np.array([r['position'][:2] for r in rows],dtype=float);v=np.array([r['velocity'][:2] for r in rows],dtype=float)
 if p.shape!=(81,2) or v.shape!=(81,2) or not np.isfinite(np.r_[t,p.ravel(),v.ravel()]).all() or np.max(np.abs(t-np.arange(81)*.1))>1e-5:raise ValueError('finite complete 10Hz state sequence required')
 maximum=float(np.max(np.linalg.norm(p-p[0],axis=1)));low=bool(np.all(np.linalg.norm(v[-5:],axis=1)<.05));inside=maximum<.7;baseline=maximum>=.1
 return dict(maximum_actual_displacement_m=maximum,inside_center_prior=inside,final_five_XY_low=low,minimum_actual_baseline=baseline,passes=inside and low and baseline,not_navigation_or_source_admission=True)
