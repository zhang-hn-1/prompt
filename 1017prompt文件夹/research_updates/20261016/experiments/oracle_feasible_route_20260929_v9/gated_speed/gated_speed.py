"""Boost only at low path error and moderate measured steering demand."""

import math


def with_confident_speed(base_class):
    class ConfidentSpeed(base_class):
        policy_name = "oracle_time_feasible_confident_speed_experimental"

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
            if not all(math.isfinite(x) for x in (
                baseline, planned_main_command_rad_s, desired_curvature_1_m,
                cross_track_error_m,
            )):
                raise ValueError("non-finite speed schedule input")
            if (
                math.isclose(baseline, 1.0, abs_tol=1e-12)
                and cross_track_error_m <= 0.10
                and abs(desired_curvature_1_m) <= 0.55
            ):
                return min(abs(planned_main_command_rad_s), 1.5)
            return baseline

    return ConfidentSpeed
