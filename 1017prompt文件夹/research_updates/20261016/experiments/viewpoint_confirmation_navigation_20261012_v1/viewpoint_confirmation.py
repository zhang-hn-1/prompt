"""One persistent-risk admission change; immediate recent10s hazards unchanged.
One grid-cell camera displacement is a necessary heuristic, not independence
or a depth correctness certificate. Absence never retracts confirmed risks.
"""
from types import SimpleNamespace
import numpy as np
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from partial_point_risk_grid import PartialPointRiskGrid

class ViewpointConfirmationGrid(RepeatedExposureMemoryGrid):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self._epoch_positions={}
 def consume_records(self,records,now,root,viewpoints=None):
  t=float(now)
  if not np.isfinite(t) or t<self.time:raise ValueError('finite monotonic query')
  views=viewpoints if viewpoints is not None else self.source['viewpoints']
  # Check all clocks and identities before consulting physical payloads.
  for r in records:
   f=r['frame'];b=float(r['time_s'])
   if not isinstance(f,(int,np.integer)) or f<0 or not np.isfinite(b) or b>t:raise ValueError('causal acquisition')
   if f not in views or float(views[f]['time_s'])!=b:raise ValueError('matching causal camera acquisition required')
   if f in self._epoch_times and self._epoch_times[f]!=b:raise ValueError('changed acquisition time')
  positions={}
  for r in records:
   f=int(r['frame']);p=np.asarray(views[f]['position_world'],float)
   if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('finite native camera position')
   if f in self._epoch_positions and not np.array_equal(p,self._epoch_positions[f]):raise ValueError('changed camera acquisition')
   positions[f]=p.copy()
  PartialPointRiskGrid.consume_records(self,records,t,root)
  for r in records:
   f=int(r['frame']);b=float(r['time_s']);self._epoch_times[f]=b;self._epoch_positions[f]=positions[f]
   if t-b>10.:continue
   cells=self.cells(np.asarray(r['world'],float).reshape(-1,3)[:,:2]);cells=cells[self.inside(cells)]
   for c in np.unique(cells,axis=0):
    key=tuple(c);epochs=self._confirmation_epochs.setdefault(key,set())
    confirmed=np.isfinite(self._confirmed_clock[key])
    diverse=any(np.linalg.norm(positions[f]-self._epoch_positions[e])>=self.resolution-1e-8 for e in epochs)
    epochs.add(f)
    if confirmed or diverse:self._confirmed_clock[key]=max(self._confirmed_clock[key],b,self.hazard_acquisition_time[key])
  clock=np.maximum(self.hazard_acquisition_time,self._confirmed_clock);active=np.isfinite(clock)
  self.labels=np.full(self.labels.shape,-1,np.int8);self.labels[active]=1
  self.codes=np.zeros(self.labels.shape,np.uint8);self.codes[active]=9;self.codes[active&(clock==t)]=8
  self._persistent_clock=self._confirmed_clock.copy();self.hazard_acquisition_time=clock;self.last_seen=clock.copy()
  self.memory=SimpleNamespace(floor_time=np.full(self.labels.shape,-np.inf),hazard_time=clock.copy())
 def diagnostics(self):
  d=super().diagnostics();d.update(hazard_lifetime='recent10s_or_two_capture_epochs_separated_by_one_gridcell_camera_translation',persistent_viewpoint_separation_m=self.resolution,viewpoint_screen_not_independence_certificate=True)
  return d
