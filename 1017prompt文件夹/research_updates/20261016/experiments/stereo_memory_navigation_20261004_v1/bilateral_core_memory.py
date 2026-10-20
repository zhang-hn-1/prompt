"""Persistent MODEL cores from original bilateral support, never measured free."""
import copy
import numpy as np
from partial_cue_source_v2 import inflate_xy

class BilateralCoreMemory:
 def __init__(self):
  self.last_time=-np.inf;self.windows=[];self.cores={};self.model_birth=None;self.frozen=None
 def update(self,packet,now):
  t=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or abs(t-now)>1e-9 or t<=self.last_time or self.frozen is not None:raise ValueError('current monotone unfrozen source clock before payload')
  if not np.isfinite(packet['frame']) or float(packet['frame'])!=round(float(packet['frame'])) or int(packet['frame'])!=round(t*10) or abs(int(packet['frame'])*.1-t)>1e-5:raise ValueError('frame clock')
  if not bool(packet['model_hypothesis']) or bool(packet['measured_geometry']) or bool(packet['certified_free']):raise ValueError('MODEL only')
  if float(packet['calibrated_uncertainty_m'])!=.2:raise ValueError('frozen uncertainty')
  w=np.asarray(packet['raw_world_voxel_centres'],float)
  if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():raise ValueError('finite raw cores')
  if bool(packet['pending_seed']):
   if len(w):raise ValueError('pending cannot invent cores')
   self.last_time=t;return
  birth=float(packet['source_model_birth_time_s'])
  ts=np.asarray(packet['support_times_s'],float);fs=np.asarray(packet['support_frames'])
  if not np.isfinite(birth) or birth>t or float(packet['source_seed_time_s'])!=birth or (self.model_birth is not None and self.model_birth!=birth):raise ValueError('frozen class birth')
  if fs.ndim!=1 or not np.isfinite(fs).all() or not np.equal(fs,np.rint(fs)).all() or ts.ndim!=1 or len(ts)!=len(fs) or not len(ts) or not np.isfinite(ts).all() or (ts>t+1e-9).any() or not np.allclose(fs*.1,ts,atol=1e-5,rtol=0) or not np.all(np.diff(fs)>0):raise ValueError('causal support windows')
  self.model_birth=birth;self.last_time=t
  self.windows.append(dict(frame=int(packet['frame']),time_s=t,support_frames=fs.astype(int).tolist(),support_times_s=ts.tolist(),raw_core_count=len(w)))
  for p in w:
   key=tuple(np.round(p,10))
   if key not in self.cores:self.cores[key]=dict(xyz=list(key),first_derived_time_s=t,last_derived_time_s=t,derived_frames=[])
   r=self.cores[key];r['last_derived_time_s']=t
   if not r['derived_frames'] or r['derived_frames'][-1]!=int(packet['frame']):r['derived_frames'].append(int(packet['frame']))
 def core_provenance(self):return copy.deepcopy(list(self.cores.values()))
 def freeze(self,now):
  if not np.isfinite(now) or abs(now-self.last_time)>1e-9:raise ValueError('freeze at current clock')
  if self.frozen is None:
   if not self.cores:raise ValueError('empty MODEL pool')
   w=np.array(list(self.cores),float)
   self.frozen=dict(time_s=now,frame=round(now*10),hypothesis_birth_time_s=now,source_seed_time_s=self.model_birth,source_model_birth_time_s=self.model_birth,raw_world_voxel_centres=w,possible_xy=inflate_xy(w[:,:2],.2),model_hypothesis=True,measured_geometry=False,certified_free=False,source_model_admitted=False,calibrated_uncertainty_m=.2,pending_seed=False,static_persistence_prior=True,per_point_measured_timestamps_available=False)
  return copy.deepcopy(self.frozen)
