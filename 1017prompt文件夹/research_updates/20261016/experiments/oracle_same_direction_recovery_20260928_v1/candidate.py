"""One bounded main-speed intervention; original reference and steering law."""
from probe import EventOracle

class RecoveryOracle(EventOracle):
    def reset(self,context):
        super().reset(context)
        self.recovery_active=False
        self.recovery_attempted=False
        self.recovery_direction=None
    def act(self,observation,pointgoal_base_xy):
        if not self.recovery_attempted and self.ready:
            self.recovery_attempted=True
            self.recovery_active=True
            self.recovery_direction=self._active_drive_direction
        return super().act(observation,pointgoal_base_xy)
    def _handle_direction_change(self,direction,speed):
        if self.recovery_active and direction!=self.recovery_direction:
            self.recovery_active=False
        return super()._handle_direction_change(direction,speed)
    def _schedule_main_magnitude(self,observation,*,baseline_magnitude,**kwargs):
        requested=max(float(baseline_magnitude),1.) if self.recovery_active else float(baseline_magnitude)
        self._tracking_audit.update(recovery_active=self.recovery_active,recovery_baseline_main_magnitude=float(baseline_magnitude),recovery_scheduled_main_magnitude=requested)
        return requested
