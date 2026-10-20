"""Immutable RGB-derived model barrier; never a measured occupancy/free map."""
import copy
from types import SimpleNamespace
import numpy as np
from prior_rgb_navigation import PriorRGBGrid
from point_support_shadow import swept_indices

class TypedHypothesisGrid(PriorRGBGrid):
 def __init__(self,origin,retention_s):
  if retention_s not in [10,60]:raise ValueError('frozen arms')
  super().__init__(origin,memory_mode='current');self.retention_s=retention_s;self.birth=None;self.hypothesis_birth_time=np.full(self.labels.shape,-np.inf)
 def seed(self,packet,now):
  t=float(packet['time_s'])
  if not np.isfinite(t) or t>now:raise ValueError('future before payload')
  if self.birth is not None:raise ValueError('immutable birth cannot be refreshed')
  if not packet['model_hypothesis'] or packet['measured_geometry'] or packet['certified_free']:raise ValueError('hypothesis-only source')
  if float(packet['hypothesis_birth_time_s'])!=t or float(packet['source_seed_time_s'])!=t:raise ValueError('initial source birth')
  xy=np.asarray(packet['possible_xy'])
  if xy.ndim!=2 or xy.shape[1]!=2 or not np.isfinite(xy).all() or not len(xy):raise ValueError('nonempty hypothesis centres')
  self.birth=copy.deepcopy(packet)
 def update_points(self,points,root,time_s,ground_meshes=()):
  t=float(time_s)
  if not np.isfinite(t) or t<self.time or ground_meshes or len(points):raise ValueError('typed model grid disallows measured geometry')
  if self.source is None or float(self.source['time_s'])>t:raise ValueError('causal source clock')
  self.labels=np.full(self.labels.shape,-1,np.int8);self.codes=np.zeros(self.labels.shape,np.uint8);self.hypothesis_birth_time.fill(-np.inf)
  if self.birth is not None:
   birth=float(self.birth['hypothesis_birth_time_s'])
   if t-birth<=self.retention_s+1e-9:
    cells=self.cells(self.birth['possible_xy']);cells=cells[self.inside(cells)];idx=tuple(cells.T)
    self.labels[idx]=1;self.codes[idx]=6 if t-birth<=.5+1e-9 else 7;self.hypothesis_birth_time[idx]=birth
  if not self._started:
   c=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7);c=c[self.inside(c)];self.safe_start_prior[tuple(c.T)]=True;self._started=True
  self.current_disk=swept_indices([np.asarray(root)[:2]],self.origin,self.resolution,.7)
  self.last_seen=np.full(self.labels.shape,-np.inf);self.time=t
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=np.full(self.labels.shape,-np.inf))
 def diagnostics(self):
  d=super().diagnostics();d.update(current_model_hypothesis=int((self.codes==6).sum()),past_model_hypothesis=int((self.codes==7).sum()),model_retention_s=self.retention_s,measured_occupied=0,measured_free=0,hypothesis_not_observation=True,source_birth_time_s=None if self.birth is None else float(self.birth['time_s']))
  return d

class BootstrapNotExercised(RuntimeError):pass
def require_bootstrap_progress(gate,current_frame=None):
 if current_frame is not None and current_frame>=80 and not gate.handed_off:gate.blocked=True
 if gate.blocked and not gate.handed_off:raise BootstrapNotExercised('SHARED_BOOTSTRAP_NOT_EXERCISED_BY_8S')
