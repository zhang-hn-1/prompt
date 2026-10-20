"""Posthoc verification only; no change to the running navigation producer."""
from prior_rgb_navigation import PriorRGBGrid,PointSupportMap,records,np,SimpleNamespace,swept_indices
from grid_only_snapshot import grid_arrays_snapshot

class VerificationGrid(PriorRGBGrid):
 def update_points(self,points,root,time_s,ground_meshes=()):
  t=float(time_s)
  if not np.isfinite(t) or t<self.time or ground_meshes:raise ValueError('noncausal query or disallowed mesh')
  if self.source is None or not np.isfinite(float(self.source['time_s'])) or float(self.source['time_s'])>t:raise ValueError('current/past RGB source required')
  if self.memory_mode=='current':self.mapper=PointSupportMap(self.origin,self.labels.shape,self.resolution)
  self.mapper.update(records(self.source),t,0.)
  if not self._started:
   c=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7);c=c[self.inside(c)];self.safe_start_prior[c[:,0],c[:,1]]=True;self._started=True
  s=grid_arrays_snapshot(self.mapper,t);self.labels=s['labels'];self.codes=s['provenance'];self.memory=SimpleNamespace(floor_time=s['floor_time'],hazard_time=s['hazard_time']);self.last_seen=np.maximum(s['floor_time'],s['hazard_time']);self.time=t;self.current_disk=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7)
