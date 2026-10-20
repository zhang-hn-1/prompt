"""One memory lifetime consumer: recent risk unchanged; corroborated risk persists.
Distinct capture epochs are not a claim of independent depth errors or 3D safety.
"""
from types import SimpleNamespace
import numpy as np
from partial_point_risk_grid import PartialPointRiskGrid
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid

class RepeatedExposureMemoryGrid(NonbindingSideHypothesisGrid):
 def __init__(self,origin,retention_s,*,static_scene_assumption):
  super().__init__(origin,retention_s,static_scene_assumption=static_scene_assumption)
  self._confirmation_epochs={};self._epoch_times={}
  self._confirmed_clock=np.full(self.labels.shape,-np.inf)
 def consume_records(self,records,now,root):
  # Original validation checks ALL clocks before any future world payload.
  PartialPointRiskGrid.consume_records(self,records,now,root)
  for r in records:
   f=r['frame'];b=float(r['time_s'])
   if not isinstance(f,(int,np.integer)) or int(f)<0:raise ValueError('actual capture frame ID required')
   if int(f) in self._epoch_times and self._epoch_times[int(f)]!=b:raise ValueError('frame acquisition clock cannot change')
  for r in records:
   f=int(r['frame']);b=float(r['time_s']);self._epoch_times[f]=b
   if float(now)-b>10.:continue
   points=np.asarray(r['world'],float).reshape(-1,3);cells=self.cells(points[:,:2]);cells=cells[self.inside(cells)]
   for c in np.unique(cells,axis=0):
    key=tuple(c);epochs=self._confirmation_epochs.setdefault(key,set());epochs.add(f)
    if len(epochs)>=2:self._confirmed_clock[key]=max(self._confirmed_clock[key],b,self.hazard_acquisition_time[key])
  clock=np.maximum(self.hazard_acquisition_time,self._confirmed_clock);active=np.isfinite(clock)
  self.labels=np.full(self.labels.shape,-1,np.int8);self.labels[active]=1
  self.codes=np.zeros(self.labels.shape,np.uint8);self.codes[active]=9;self.codes[active&(clock==float(now))]=8
  self._persistent_clock=self._confirmed_clock.copy();self.hazard_acquisition_time=clock;self.last_seen=clock.copy()
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=clock.copy())
 def diagnostics(self):
  d=super().diagnostics();d.update(hazard_lifetime='original_recent10s_or_two_capture_epoch_confirmed_persistent',persistent_confirmation_required_epochs=2,confirmed_persistent_hazard_cells=int(np.isfinite(self._confirmed_clock).sum()),recent_unconfirmed_hazard_cells=int(((self.labels==1)&~np.isfinite(self._confirmed_clock)).sum()),confirmation_not_statistical_independence=True,expired_unconfirmed_is_unknown_not_free=True)
  return d
