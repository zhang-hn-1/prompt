"""One initial handoff qualifier; causal packet checks and low count retained."""
from standby_handoff_v1 import StandbyHandoff
class ClockQualifiedStandby(StandbyHandoff):
    def __init__(self,*,enabled):
        super().__init__();self.enabled=bool(enabled);self.early_handoff=False
    def choose(self,time_s,position_xy,speed_m_s,source):
        action=super().choose(time_s,position_xy,speed_m_s,source)
        if self.enabled and not self.handed_off and not self.failed and self.standby_low_count>=5:
            self.handed_off=True;self.standby_handoff=True;self.early_handoff=True
            self.handoff_time_s=float(time_s);self.halt_reason='five_actual_low_initial_handoff_without_clock_floor'
            return None
        return action
    @property
    def diagnostics(self):
        r=super().diagnostics
        r.update(clock_floor_candidate_enabled=self.enabled,early_initial_handoff=self.early_handoff,
            source_validation_inherited=True,not_space_or_motion_prediction=True)
        return r
