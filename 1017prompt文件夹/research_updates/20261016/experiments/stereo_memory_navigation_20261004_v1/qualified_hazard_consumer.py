"""Experimental static hazard-source qualification; no free-space certification."""
import copy
from types import SimpleNamespace
import numpy as np
from past_geometry_anchor_filter import PastGeometryAnchorFilter
from prior_rgb_navigation import PriorRGBGrid
from dense_source_points import records

def subset(packet, select):
 out=copy.deepcopy(dict(packet))
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:
  out[k]=np.asarray(packet[k])[select].copy()
 return out

class QualifiedHazardConsumer:
 def __init__(self):self.filter=PastGeometryAnchorFilter();self.seed=None;self.time=-np.inf;self.diagnostics={}
 def consume(self,source,proof,now,qualified_seed):
  t=float(now);qt=float(source['time_s'])
  if not np.isfinite(t) or not np.isfinite(qt) or qt>t or t<self.time:raise ValueError('current/past source before payload')
  if proof is not None:
   pt=float(proof['current_appearance_time_s'])
   if not np.isfinite(pt) or pt>t or abs(pt-t)>1e-9:raise ValueError('current appearance before payload')
  xyz=np.asarray(source['world']);hazard=(xyz[:,2]>.05)&(xyz[:,2]<=1.5)
  if self.seed is None and qualified_seed:
   accepted=self.filter.ingest(source,t,True)
   if not self.filter.seeded:raise ValueError('qualified seed lacks hazards')
   self.seed=subset(source,hazard)
  elif self.seed is not None:accepted=self.filter.ingest(source,t,False)
  else:accepted=hazard.copy()
  current=subset(source,(~hazard)|accepted)
  past=subset(source,np.zeros(len(xyz),bool))
  if self.seed is not None and proof is not None:
   if float(proof['source_geometry_time_s'])!=float(self.seed['time_s']) or int(proof['source_frame'])!=int(self.seed['frame']):raise ValueError('appearance source identity mismatch')
   for a,b in [('past_world','world'),('past_query_id','query_pixel_id'),('past_partner_id','partner_pixel_id')]:
    if not np.array_equal(proof[a],self.seed[b]):raise ValueError('old geometry changed')
   selected=np.asarray(proof['appearance_corroborated'])
   if selected.dtype!=np.bool_ or selected.shape!=(len(self.seed['world']),):raise ValueError('boolean current appearance selection')
   if t-float(self.seed['time_s'])>10:selected=np.zeros_like(selected)
   past=subset(self.seed,selected)
  self.time=t
  self.diagnostics=dict(time_s=t,seeded=self.seed is not None,current_hazards=int(hazard.sum()),qualified_current_hazards=int((hazard&accepted).sum()),old_appearance_hazards=len(past['world']),old_geometry_time_s=float(self.seed['time_s']) if self.seed is not None else None,unsupported_hazards_unknown=True,source_model_admitted=False,certified_free=False)
  return current,past

class QualifiedHazardGrid(PriorRGBGrid):
 def set_memory_source(self,packet):self.memory_source=packet
 def update_points(self,points,root,time_s,ground_meshes=()):
  super().update_points(points,root,time_s,ground_meshes)
  past=getattr(self,'memory_source',None)
  if past is None:return
  self.mapper.update(records(past),float(time_s),0.)
  s=self.mapper.snapshot(float(time_s));self.labels=s['labels'];self.codes=s['provenance'];self.memory=SimpleNamespace(floor_time=s['floor_time'],hazard_time=s['hazard_time']);self.last_seen=np.maximum(s['floor_time'],s['hazard_time'])
