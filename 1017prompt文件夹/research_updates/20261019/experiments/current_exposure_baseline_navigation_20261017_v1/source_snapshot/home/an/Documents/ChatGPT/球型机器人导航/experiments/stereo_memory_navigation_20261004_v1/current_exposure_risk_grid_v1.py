"""Current-exposure positive risk ablation. No discarded observation becomes free.
The existing nonbinding route-side hypothesis, task priors and planner are unchanged.
"""
import numpy as np
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid

class CurrentExposureRiskGrid(RepeatedExposureMemoryGrid):
 def consume_records(self,records,now,root):
  envelope=self.source
  if envelope is None:raise ValueError('current source envelope required')
  frame=envelope['frame'];t=float(envelope['time_s']);now=float(now)
  if isinstance(frame,bool) or not isinstance(frame,(int,np.integer)) or frame<0 or not np.isfinite(t) or not np.isfinite(now) or t>now:raise ValueError('causal exposure envelope required')
  previous=getattr(self,'_latest_successful_exposure',None)
  if previous is not None and (frame<previous[0] or t<previous[1] or (frame==previous[0] and t!=previous[1])):raise ValueError('source exposure must be monotone')
  for r in records:
   rt=float(r['time_s']);rf=r['frame']
   if not np.isfinite(rt) or rt>t or isinstance(rf,bool) or not isinstance(rf,(int,np.integer)) or rf<0 or rf>frame:raise ValueError('future or invalid record before payload')
   if rf==frame and rt!=t:raise ValueError('current capture clock must match envelope')
  selected=[r for r in records if r['frame']==frame and float(r['time_s'])==t]
  self._confirmation_epochs={};self._epoch_times={};self._confirmed_clock.fill(-np.inf)
  super().consume_records(selected,now,root)
  self._latest_successful_exposure=(int(frame),t)
 def diagnostics(self):
  d=super().diagnostics();d.update(hazard_lifetime='most_recent_actual_stereo_exposure_only',risk_memory_ablation=True,nonbinding_route_side_hypothesis_unchanged=True,expired_unconfirmed_is_unknown_not_free=True,discarded_history_is_observed_free=False)
  return d
