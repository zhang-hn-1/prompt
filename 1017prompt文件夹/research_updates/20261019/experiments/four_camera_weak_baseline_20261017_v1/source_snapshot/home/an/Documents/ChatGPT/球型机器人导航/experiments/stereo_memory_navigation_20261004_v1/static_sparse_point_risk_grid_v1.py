"""TaskH static-scene assumption: retain only previously measured hazard cells.
This changes hazard expiry, not observation provenance or unknown/free semantics.
"""
from types import SimpleNamespace
import numpy as np
from partial_point_risk_grid import PartialPointRiskGrid

class StaticSparsePointRiskGrid(PartialPointRiskGrid):
 def __init__(self,origin,retention_s,*,static_scene_assumption):
  if static_scene_assumption is not True:raise ValueError('explicit static scene assumption required')
  super().__init__(origin,retention_s)
  self._persistent_clock=np.full(self.labels.shape,-np.inf)
  self.hazard_acquisition_time=self._persistent_clock.copy()
 def consume_records(self,records,now,root):
  super().consume_records(records,now,root)
  clock=np.maximum(self._persistent_clock,self.hazard_acquisition_time)
  active=np.isfinite(clock)
  self.labels=np.full(self.labels.shape,-1,np.int8);self.labels[active]=1
  self.codes=np.zeros(self.labels.shape,np.uint8);self.codes[active]=9;self.codes[active&(clock==float(now))]=8
  self._persistent_clock=clock.copy();self.hazard_acquisition_time=clock
  self.last_seen=clock.copy()
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=clock.copy())
 def diagnostics(self):
  d=super().diagnostics()
  d.update(static_scene_assumption=True,hazard_lifetime='persistent_past_measured_cells_until_episode_reset',raw_point_record_retention_s=10,source_domain='original_E1_development_partial_surface_only')
  return d
