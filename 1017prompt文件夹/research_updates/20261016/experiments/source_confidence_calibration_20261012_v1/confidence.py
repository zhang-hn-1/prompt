"""Confidence-only point consumer. Missing/excluded is unknown, never free.
Consumes current SGBM packet selected by existing range/CAD masks. Does not
alter past risk, infer completed volume, or consult renderer truth.
"""
import numpy as np
from scipy.special import expit
from features import features
class SourceConfidence:
 def __init__(self,model_path):
  with np.load(model_path) as m:self.mean=m['mean'].copy();self.std=m['std'].copy();self.weights=m['weights'].copy()
  if self.mean.shape!=(12,) or self.std.shape!=(12,) or self.weights.shape!=(13,) or not np.isfinite(np.concatenate([self.mean,self.std,self.weights])).all() or np.any(self.std<=0):raise ValueError('invalid frozen model')
 def qualify_packet(self,rgb,pred,packet,now):
  t=float(now)
  for r in [rgb,pred,packet]:
   if not np.isfinite(float(r['time_s'])) or float(r['time_s'])!=t:raise ValueError('matching finite current capture clocks required')
  if not np.isfinite(t) or t<0:raise ValueError('finite current time')
  if int(pred['frame'])!=int(packet['frame']):raise ValueError('capture identity')
  q=np.asarray(packet['query_pixel_id']);p=np.asarray(packet['partner_pixel_id']);n=len(q)
  if q.shape!=(n,) or p.shape!=(n,) or q.dtype.kind not in 'iu' or p.dtype.kind not in 'iu' or np.any(q<0) or np.any(q>=640*480) or np.any(p<0) or np.any(p>=640*480):raise ValueError('native paired pixel identity')
  X,_=features(rgb,pred,packet);prob=expit(np.column_stack([np.ones(n),(X-self.mean)/self.std])@self.weights);keep=prob<.5
  result={k:np.asarray(v).copy() for k,v in packet.items()}
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:
   if result[k].shape[0]!=n:raise ValueError('unaligned point fields')
   result[k]=result[k][keep]
  result.update(confidence_filter_applied=np.array(True),source_bad_probability=prob[keep],excluded_count=np.array(int((~keep).sum())),excluded_is_unknown=np.array(True),certified_free=np.array(False),model_point_quality_not_volume_certificate=np.array(True))
  return result
