"""ONE initial observation-to-tracker handoff; no source accuracy qualification or free-space certificate."""
from front_view_acquisition_v1 import FrontViewAcquisition
class StandbyHandoff(FrontViewAcquisition):
 def __init__(self):
  super().__init__();self.standby_low_count=0;self.standby_handoff=False
 def choose(self,time_s,position_xy,speed_m_s,source):
  # All old current/past packet validation and bounds retained, exactly one call.
  action=super().choose(time_s,position_xy,speed_m_s,source)
  self.standby_low_count=self.standby_low_count+1 if speed_m_s<.05 else 0
  if not self.handed_off and not self.failed and time_s>=6.-1e-6 and self.standby_low_count>=5:
   self.handed_off=True;self.standby_handoff=True;self.handoff_time_s=float(time_s);self.halt_reason='measured_standby_handoff_without_source_point_gate';return None
  return action
 @property
 def diagnostics(self):
  r=super().diagnostics;r.update(standby_low_count=self.standby_low_count,standby_handoff=self.standby_handoff,handoff_not_observation_or_safety_certificate=True,point_count_not_motion_qualification=True,unknown_exploration_still_task_prior=True);return r
