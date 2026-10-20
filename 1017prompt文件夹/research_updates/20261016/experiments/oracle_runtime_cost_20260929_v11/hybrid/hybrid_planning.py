"""Retain a measured quick reference; replace a slow one with the tested time route."""

import math
import numpy as np
from benchmark.kinodynamic_planning import KinodynamicPlanResult

EPISODE_BUDGET_S = 35.0
TRANSITION_RESERVE_S = 10.0
FROZEN_REFERENCE_CUTOFF_S = EPISODE_BUDGET_S - TRANSITION_RESERVE_S


def steady_execution_time_s(plan: KinodynamicPlanResult, model) -> float:
    if plan.status != "success" or len(plan.waypoints) < 2:
        return math.inf
    total = 0.0
    for before, after in zip(plan.waypoints, plan.waypoints[1:]):
        length = math.hypot(after.x_m-before.x_m, after.y_m-before.y_m)
        speed = model.speed(before.main_command_rad_s, before.auxiliary_angle_rad)
        if speed <= 0 or not math.isfinite(speed):
            return math.inf
        total += length / speed
    return total + plan.direction_changes


def choose_reference(frozen: KinodynamicPlanResult, time_route: KinodynamicPlanResult, model):
    frozen_time = steady_execution_time_s(frozen, model)
    time_route_time = steady_execution_time_s(time_route, model)
    if frozen.status == "success" and frozen_time <= FROZEN_REFERENCE_CUTOFF_S:
        selected = frozen
        source = "frozen_measured_time_with_reserve"
    elif time_route.status == "success":
        selected = time_route
        source = "time_feasible_curvature_limited"
    else:
        selected = frozen
        source = "frozen_after_time_route_failure"
    return selected, dict(
        selected_source=source,
        frozen_status=frozen.status,
        time_route_status=time_route.status,
        frozen_steady_time_s=frozen_time if math.isfinite(frozen_time) else None,
        time_route_steady_time_s=time_route_time if math.isfinite(time_route_time) else None,
        frozen_length_m=frozen.path_length_m,
        time_route_length_m=time_route.path_length_m,
        reserve_s=TRANSITION_RESERVE_S,
        cutoff_s=FROZEN_REFERENCE_CUTOFF_S,
    )
