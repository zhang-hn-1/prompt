"""One isolated operating-speed intervention on the frozen time-feasible path."""

import math


def with_moderate_tracking_speed(base_class):
    class ModerateTrackingSpeed(base_class):
        policy_name = "oracle_time_feasible_moderate_speed_experimental"

        def _schedule_main_magnitude(
            self,
            observation,
            *,
            baseline_magnitude,
            planned_main_command_rad_s,
            desired_curvature_1_m,
            cross_track_error_m,
        ):
            baseline = super()._schedule_main_magnitude(
                observation,
                baseline_magnitude=baseline_magnitude,
                planned_main_command_rad_s=planned_main_command_rad_s,
                desired_curvature_1_m=desired_curvature_1_m,
                cross_track_error_m=cross_track_error_m,
            )
            if not all(math.isfinite(x) for x in (baseline, planned_main_command_rad_s, desired_curvature_1_m, cross_track_error_m)):
                raise ValueError("non-finite speed schedule input")
            next_supported = 1.0 if baseline <= 0.5 + 1e-12 else 1.5
            return min(abs(planned_main_command_rad_s), next_supported)

    return ModerateTrackingSpeed
