"""Source-only opportunity event, not admission/actual handoff."""
import numpy as np
class EarlierSourceOpportunity:
 def __init__(self):self.previous=None;self.latched=False;self.diagnostics={}
 def update(self,source,past_XY_speeds,now,frame):
  t=float(source['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or abs(t-now)>1e-6:raise ValueError('current source clock required before payload')
  if self.latched:return None
  speeds=np.asarray(past_XY_speeds,float);low=speeds.shape==(5,2) and np.isfinite(speeds).all() and bool((np.linalg.norm(speeds,axis=1)<.05).all());w=np.asarray(source['world'],float)
  if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():raise ValueError('finite geometry required')
  hazard=w[(w[:,2]>.05)&(w[:,2]<=1.5)];bins=len(np.unique(np.floor(hazard[:,:2]/.1).astype(np.int64),axis=0)) if len(hazard) else 0;qualified=low and len(hazard)>=100 and bins>=2;self.diagnostics=dict(frame=int(frame),time_s=float(now),low_five=bool(low),current_hazards=len(hazard),XY_bins=bins,opportunity_qualifying=bool(qualified),source_model_admitted=False)
  if not qualified:self.previous=None;return None
  if self.previous is not None and frame-self.previous==5:
   pair=[self.previous,int(frame)];self.latched=True;return pair
  self.previous=int(frame);return None
