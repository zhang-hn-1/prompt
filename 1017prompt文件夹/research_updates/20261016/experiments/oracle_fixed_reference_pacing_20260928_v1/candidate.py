"""One diagnostic intervention: use the frozen reference's planned magnitude."""
from probe import EventOracle

class PlannedSpeedOracle(EventOracle):
    def _schedule_main_magnitude(self, observation, *, baseline_magnitude,
                                 planned_main_command_rad_s,
                                 desired_curvature_1_m, cross_track_error_m):
        magnitude = abs(float(planned_main_command_rad_s))
        self._tracking_audit.update(
            pacing_active=True,
            pacing_baseline_main_magnitude=float(baseline_magnitude),
            pacing_planned_main_magnitude=magnitude,
        )
        return magnitude
