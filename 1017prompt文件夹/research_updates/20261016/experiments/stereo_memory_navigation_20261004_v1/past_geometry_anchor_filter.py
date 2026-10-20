"""One static past-point consistency filter; never a free-space certificate."""
import numpy as np
from scipy.spatial import cKDTree

class PastGeometryAnchorFilter:
 def __init__(self):
  self.time=-np.inf;self.seeded=False;self.seen={};self.accepted={}
 def ingest(self,packet,current_time_s,qualified_seed=False):
  t=float(current_time_s);qt=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(qt) or qt>t or t<self.time:raise ValueError('past/current timestamps required before payload read')
  required={'world','sigma_m','query_pixel_id','partner_pixel_id','frame','time_s','frame_width'}
  allowed=required|{'pose_sigma_m','source_model_admitted','certified_free'}
  if not required.issubset(packet) or not set(packet).issubset(allowed):raise ValueError('measurement whitelist')
  xyz=np.asarray(packet['world'],float);q=np.asarray(packet['query_pixel_id']);partner=np.asarray(packet['partner_pixel_id']);sigma=np.asarray(packet['sigma_m'],float);n=len(xyz);frame=int(packet['frame'])
  if xyz.shape!=(n,3) or q.shape!=(n,) or partner.shape!=(n,) or sigma.shape!=(n,) or not np.isfinite(xyz).all() or not np.isfinite(sigma).all():raise ValueError('finite aligned measurements required')
  if not np.issubdtype(q.dtype,np.integer) or not np.issubdtype(partner.dtype,np.integer) or (q<0).any() or (partner<0).any() or len(np.unique(q))!=n:raise ValueError('unique integer source identities')
  keys=[(frame,'left',int(x),qt) for x in q]
  values=[(tuple(x),float(s),int(p)) for x,s,p in zip(xyz,sigma,partner)]
  for k,v in zip(keys,values):
   if k in self.seen and self.seen[k][0]!=v:raise ValueError('original source identity changed')
  alive={k:v for k,v in self.accepted.items() if t-k[3]<=10.}
  past=[v for k,v in alive.items() if k[0]!=frame and k[3]<qt]
  tree=cKDTree(np.array(past)) if past else None
  hazard=(xyz[:,2]>.05)&(xyz[:,2]<=1.5)
  seed=bool(qualified_seed and not self.seeded and hazard.any() and t-qt<=.5)
  decision=np.zeros(n,bool)
  distance=tree.query(xyz,k=1)[0] if tree is not None else np.full(n,np.inf)
  for j,(k,v) in enumerate(zip(keys,values)):
   decision[j]=self.seen[k][1] if k in self.seen else bool(hazard[j] and t-qt<=10 and (seed or distance[j]<=.10))
  self.accepted=alive
  for k,v,keep in zip(keys,values,decision):
   self.seen.setdefault(k,(v,bool(keep)))
   if keep and t-k[3]<=10:self.accepted[k]=v[0]
  self.seeded=self.seeded or seed;self.time=t
  return decision
