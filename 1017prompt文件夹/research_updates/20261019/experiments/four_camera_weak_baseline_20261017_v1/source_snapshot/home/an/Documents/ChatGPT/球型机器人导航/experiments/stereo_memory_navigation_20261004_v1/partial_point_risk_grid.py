"""Experimental positive-surface risk only; unknown never becomes observed free.
No completed shape, calibrated uncertainty, or complete blind-space certificate.
"""
from types import SimpleNamespace
import numpy as np
from execution_domain_four_point_consumer_v2 import TrackingTubeGrid
from point_support_shadow import swept_indices

class PartialPointRiskGrid(TrackingTubeGrid):
 def __init__(self,origin,retention_s):
  if retention_s!=10:raise ValueError("frozen partial point TTL10s only")
  super().__init__(origin,retention_s)
 def consume_records(self,records,now,root):
  t=float(now);root=np.asarray(root,float)
  if not np.isfinite(t) or t<self.time or root.shape not in ((2,),(3,)) or not np.isfinite(root).all():raise ValueError('finite causal query/root')
  # Validate clocks before consulting each potentially poisoned world payload.
  for r in records:
   b=float(r['time_s'])
   if not np.isfinite(b) or b>t:raise ValueError('future acquisition before payload')
  clock=np.full(self.labels.shape,-np.inf)
  for r in records:
   b=float(r['time_s']);p=np.asarray(r['world'],float)
   if p.shape==(0,):p=p.reshape(0,3)
   if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or np.any(p[:,2]<=.05) or np.any(p[:,2]>1.5):raise ValueError('finite partial hazard surface')
   if t-b>10.:continue
   cells=self.cells(p[:,:2]);cells=cells[self.inside(cells)]
   if len(cells):np.maximum.at(clock,tuple(cells.T),b)
  active=np.isfinite(clock);self.labels=np.full(self.labels.shape,-1,np.int8);self.labels[active]=1
  self.codes=np.zeros(self.labels.shape,np.uint8);self.codes[active]=9;self.codes[active&(clock==t)]=8
  self.hazard_acquisition_time=clock;self.last_seen=clock.copy();self.hypothesis_birth_time.fill(-np.inf)
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=clock.copy())
  if not self._started:
   c=swept_indices([root[:2]],self.origin,self.resolution,.7);c=c[self.inside(c)];self.safe_start_prior[tuple(c.T)]=True;self._started=True
  self.current_disk=swept_indices([root[:2]],self.origin,self.resolution,.7);self.time=t
 def update_points(self,points,root,time_s,ground_meshes=()):
  if ground_meshes or len(points) or self.source is None:raise ValueError('partial record source only')
  st=float(self.source['time_s'])
  if not np.isfinite(st) or st>float(time_s):raise ValueError('future source before records')
  records=self.source['records']
  for r in records:
   rt=float(r['time_s'])
   if not np.isfinite(rt) or rt>st:raise ValueError('record newer than its source envelope before payload')
  self.consume_records(records,time_s,root)
 def diagnostics(self):
  d=super().diagnostics();d.update(current_partial_surface_cells=int((self.codes==8).sum()),past_partial_surface_cells=int((self.codes==9).sum()),full_volume_MODEL_cells=0,measured_free=0,source_domain='new_appearance_development_partial_surface_only',full_3d_certified=False,source_volume_admitted=False,controlled_unknown_exploration=True)
  return d
