"""Simulator-independent closed-loop episode runner for Benchmark V1."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import PurePosixPath
from types import MappingProxyType
from typing import Mapping, Protocol

import numpy as np

from training.action_protocol import ActionProtocol
from training.navigation_metrics import (
    episode_termination,
    path_length,
    success_weighted_path_length,
)

from .manifest import BenchmarkEpisode
from .kinodynamic_planning import KinodynamicPlanResult
from .dynamic_lattice_planning import DynamicPlanResult
from .policies import DynamicOraclePolicy, EpisodeContext, PolicyObservation


@dataclass(frozen=True)
class RunnerConfig:
    episode_budget_s: float = 35.0
    replan_hz: int = 10
    goal_tolerance_m: float = 0.2
    # Diagnostic settling is deliberately opt-in.  A positive value permits
    # zero-command blocks after the first nominal goal entry while preserving
    # the formal runner's goal/collision/timeout semantics.
    diagnostic_settling_budget_s: float = 0.0
    diagnostic_stop_hold_s: float = 0.5
    diagnostic_stop_speed_m_s: float = 0.05

    def __post_init__(self) -> None:
        budget = float(self.episode_budget_s)
        tolerance = float(self.goal_tolerance_m)
        settling_budget = float(self.diagnostic_settling_budget_s)
        stop_hold = float(self.diagnostic_stop_hold_s)
        stop_speed = float(self.diagnostic_stop_speed_m_s)
        if not math.isfinite(budget) or budget <= 0.0:
            raise ValueError("episode_budget_s must be finite and positive")
        if (
            isinstance(self.replan_hz, bool)
            or not isinstance(self.replan_hz, int)
            or self.replan_hz <= 0
        ):
            raise ValueError("replan_hz must be a positive integer")
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("goal_tolerance_m must be finite and positive")
        if not math.isfinite(settling_budget) or settling_budget < 0.0:
            raise ValueError(
                "diagnostic_settling_budget_s must be finite and non-negative"
            )
        if settling_budget > budget + 1e-12:
            raise ValueError(
                "diagnostic_settling_budget_s cannot exceed episode_budget_s"
            )
        if settling_budget > 3.0 + 1e-12:
            raise ValueError(
                "diagnostic_settling_budget_s cannot exceed three seconds"
            )
        if not math.isfinite(stop_hold) or stop_hold <= 0.0:
            raise ValueError("diagnostic_stop_hold_s must be finite and positive")
        if not math.isfinite(stop_speed) or stop_speed <= 0.0:
            raise ValueError(
                "diagnostic_stop_speed_m_s must be finite and positive"
            )
        blocks = budget * self.replan_hz
        if not math.isclose(blocks, round(blocks), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("episode budget must contain an integer number of action blocks")
        settling_blocks = settling_budget * self.replan_hz
        if not math.isclose(
            settling_blocks, round(settling_blocks), rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(
                "diagnostic settling budget must contain an integer number of action blocks"
            )
        object.__setattr__(self, "episode_budget_s", budget)
        object.__setattr__(self, "goal_tolerance_m", tolerance)
        object.__setattr__(self, "diagnostic_settling_budget_s", settling_budget)
        object.__setattr__(self, "diagnostic_stop_hold_s", stop_hold)
        object.__setattr__(self, "diagnostic_stop_speed_m_s", stop_speed)

    @property
    def max_action_blocks(self) -> int:
        return int(round(self.episode_budget_s * self.replan_hz))

    @property
    def max_diagnostic_settling_blocks(self) -> int:
        return int(round(self.diagnostic_settling_budget_s * self.replan_hz))

    @property
    def diagnostic_stop_hold_samples(self) -> int:
        return max(
            1,
            int(
                math.ceil(
                    self.diagnostic_stop_hold_s * self.replan_hz - 1e-12
                )
            ),
        )


@dataclass(frozen=True)
class RuntimeStep:
    observation: PolicyObservation
    truth_clearance_m: float
    physical_contact: bool
    collision: bool
    frames: Mapping[str, np.ndarray]
    # Telemetry is optional so simulator-independent test doubles and legacy
    # callers remain source-compatible.  Isaac runtime steps populate all
    # values from the world's measured clock and pose quaternion.
    simulation_time_s: float | None = None
    physics_step_index: int | None = None
    raw_simulation_time_s: float | None = None
    raw_physics_step_index: int | None = None
    roll_world_rad: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.observation, PolicyObservation):
            raise TypeError("observation must be a PolicyObservation")
        clearance = float(self.truth_clearance_m)
        if math.isnan(clearance) or clearance == -math.inf:
            raise ValueError("truth_clearance_m must be finite or positive infinity")
        if not isinstance(self.physical_contact, bool) or not isinstance(self.collision, bool):
            raise ValueError("physical_contact and collision must be booleans")
        if not isinstance(self.frames, Mapping):
            raise ValueError("frames must be a mapping")
        simulation_time = _optional_nonnegative_float(
            self.simulation_time_s, "simulation_time_s"
        )
        physics_step = _optional_nonnegative_int(
            self.physics_step_index, "physics_step_index"
        )
        raw_simulation_time = _optional_nonnegative_float(
            self.raw_simulation_time_s, "raw_simulation_time_s"
        )
        raw_physics_step = _optional_nonnegative_int(
            self.raw_physics_step_index, "raw_physics_step_index"
        )
        if (simulation_time is None) != (physics_step is None):
            raise ValueError(
                "simulation_time_s and physics_step_index must be provided together"
            )
        if (raw_simulation_time is None) != (raw_physics_step is None):
            raise ValueError(
                "raw_simulation_time_s and raw_physics_step_index must be provided together"
            )
        roll = _optional_finite_float(self.roll_world_rad, "roll_world_rad")
        frames: dict[str, np.ndarray] = {}
        for name, value in self.frames.items():
            if not isinstance(name, str) or not name:
                raise ValueError("frame names must be non-empty strings")
            frame = np.asarray(value)
            if frame.size == 0 or not np.all(np.isfinite(frame)):
                raise ValueError(f"frame {name!r} must contain finite image data")
            copy = frame.copy()
            copy.setflags(write=False)
            frames[name] = copy
        object.__setattr__(self, "truth_clearance_m", clearance)
        object.__setattr__(self, "simulation_time_s", simulation_time)
        object.__setattr__(self, "physics_step_index", physics_step)
        object.__setattr__(self, "raw_simulation_time_s", raw_simulation_time)
        object.__setattr__(self, "raw_physics_step_index", raw_physics_step)
        object.__setattr__(self, "roll_world_rad", roll)
        object.__setattr__(self, "frames", MappingProxyType(frames))


def _optional_finite_float(value: object, name: str) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite or None") from error
    if math.isnan(result):
        return None
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite or None")
    return result


def _optional_nonnegative_float(value: object, name: str) -> float | None:
    result = _optional_finite_float(value, name)
    if result is not None and result < 0.0:
        raise ValueError(f"{name} must be non-negative or None")
    return result


def _optional_nonnegative_int(value: object, name: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a non-negative integer or None")
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a non-negative integer or None") from error
    if math.isnan(numeric):
        return None
    if not math.isfinite(numeric) or not numeric.is_integer() or numeric < 0.0:
        raise ValueError(f"{name} must be a non-negative integer or None")
    return int(numeric)


class BenchmarkRuntime(Protocol):
    def reset(self, episode: BenchmarkEpisode) -> RuntimeStep: ...

    def execute(self, action_block: np.ndarray) -> RuntimeStep: ...

    def close(self) -> None: ...


class BenchmarkPolicy(Protocol):
    policy_name: str
    privileged: bool

    def reset(self, context: EpisodeContext) -> None: ...

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: np.ndarray,
    ) -> np.ndarray: ...


@dataclass(frozen=True)
class EpisodeRun:
    episode_id: str
    scene_id: str
    difficulty: str
    policy_name: str
    termination_reason: str
    action_blocks: int
    elapsed_s: float
    final_distance_m: float
    path_length_m: float
    reference_length_m: float
    spl: float
    goal_progress_m: float
    minimum_clearance_m: float | None
    time_to_goal_s: float | None
    runtime_error: str | None
    positions_world_xy: np.ndarray
    actions: np.ndarray
    clearances_m: np.ndarray
    physical_contacts: np.ndarray
    collisions: np.ndarray
    yaw_world_rad: np.ndarray
    linear_velocity_world_m_s: np.ndarray
    angular_velocity_world_rad_s: np.ndarray
    joint_position_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    applied_action: np.ndarray
    policy_reference_index: np.ndarray
    policy_cross_track_error_m: np.ndarray
    policy_active_drive_direction: np.ndarray
    policy_desired_drive_direction: np.ndarray
    policy_pending_drive_direction: np.ndarray
    policy_stopped_cycles: np.ndarray
    policy_braking_state_code: np.ndarray
    dynamic_reference_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.int64)
    )
    dynamic_position_error_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    dynamic_yaw_error_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    dynamic_auxiliary_error_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    # State-aligned measured telemetry.  Empty defaults preserve construction
    # compatibility for callers that predate the clock/roll contract.
    simulation_time_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    physics_step_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    raw_simulation_time_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    raw_physics_step_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    roll_world_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    # Diagnostic-only post-goal settling outcome.  These defaults retain
    # source compatibility for callers constructing EpisodeRun directly.
    diagnostic_stop_hold_passed: bool | None = None
    diagnostic_goal_entry_action_block: int | None = None
    diagnostic_settling_blocks: int = 0
    diagnostic_settling_runtime_error: str | None = None
    diagnostic_settling_mask: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=bool)
    )
    # Spatial trajectory-policy diagnostics.  These are action-aligned (one
    # row per policy action block) and default to empty arrays.  They are
    # appended after every pre-existing field so positional EpisodeRun
    # constructors remain source-compatible.
    reference_curvature_1_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    feedback_curvature_1_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    command_curvature_1_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    lateral_error_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    heading_error_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    main_cap_rad_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    auxiliary_feedforward_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    auxiliary_feedback_rad: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    # V2-specific action-aligned diagnostics.  They remain empty for the
    # baseline and V1 policies so their historical traces and defaults are
    # unchanged.  ``budget_infeasible`` is deliberately a boolean trace,
    # rather than a numeric sentinel, so false cannot be confused with an
    # unavailable diagnostic.
    projection_s_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    delay_distance_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    query_s_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    pure_pursuit_curvature_1_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    profile_curvature_1_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    blend_weight: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    budget_infeasible: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=bool)
    )
    terminal_runway_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    terminal_brake_start_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    goal_entry_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    stop_below_0p05_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    hold_complete_s: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    elapsed_s_action_aligned: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    # Runner-owned event times use measured relative simulation time when it
    # is available and an action-period fallback otherwise.  Policy-local
    # timing fields above are retained separately for provenance.
    diagnostic_terminal_brake_start_time_s: float | None = None
    diagnostic_goal_entry_time_s: float | None = None
    diagnostic_stop_below_speed_time_s: float | None = None
    diagnostic_hold_complete_time_s: float | None = None

    def __post_init__(self) -> None:
        for name in (
            "positions_world_xy",
            "actions",
            "clearances_m",
            "physical_contacts",
            "collisions",
            "yaw_world_rad",
            "linear_velocity_world_m_s",
            "angular_velocity_world_rad_s",
            "joint_position_rad",
            "joint_velocity_rad_s",
            "applied_action",
            "policy_reference_index",
            "policy_cross_track_error_m",
            "policy_active_drive_direction",
            "policy_desired_drive_direction",
            "policy_pending_drive_direction",
            "policy_stopped_cycles",
            "policy_braking_state_code",
            "reference_curvature_1_m",
            "feedback_curvature_1_m",
            "command_curvature_1_m",
            "lateral_error_m",
            "heading_error_rad",
            "main_cap_rad_s",
            "auxiliary_feedforward_rad",
            "auxiliary_feedback_rad",
            "projection_s_m",
            "delay_distance_m",
            "query_s_m",
            "pure_pursuit_curvature_1_m",
            "profile_curvature_1_m",
            "blend_weight",
            "budget_infeasible",
            "terminal_runway_s",
            "terminal_brake_start_s",
            "goal_entry_s",
            "stop_below_0p05_s",
            "hold_complete_s",
            "elapsed_s_action_aligned",
            "dynamic_reference_index",
            "dynamic_position_error_m",
            "dynamic_yaw_error_rad",
            "dynamic_auxiliary_error_rad",
            "simulation_time_s",
            "physics_step_index",
            "raw_simulation_time_s",
            "raw_physics_step_index",
            "roll_world_rad",
            "diagnostic_settling_mask",
        ):
            array = np.asarray(getattr(self, name)).copy()
            array.setflags(write=False)
            object.__setattr__(self, name, array)
        event_values: dict[str, float | None] = {}
        for name in (
            "diagnostic_terminal_brake_start_time_s",
            "diagnostic_goal_entry_time_s",
            "diagnostic_stop_below_speed_time_s",
            "diagnostic_hold_complete_time_s",
        ):
            value = getattr(self, name)
            if value is None:
                event_values[name] = None
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError(f"{name} must be finite and non-negative or None") from error
            if not math.isfinite(numeric) or numeric < 0.0:
                raise ValueError(f"{name} must be finite and non-negative or None")
            event_values[name] = numeric
        brake = event_values["diagnostic_terminal_brake_start_time_s"]
        goal = event_values["diagnostic_goal_entry_time_s"]
        stop = event_values["diagnostic_stop_below_speed_time_s"]
        hold = event_values["diagnostic_hold_complete_time_s"]
        if brake is not None and goal is not None and brake > goal + 1e-12:
            raise ValueError("terminal brake start must not follow goal entry")
        if stop is not None and goal is None:
            raise ValueError("low-speed event requires goal entry")
        if stop is not None and goal is not None and stop < goal - 1e-12:
            raise ValueError("low-speed event must not precede goal entry")
        if hold is not None and stop is None:
            raise ValueError("hold-complete event requires low-speed entry")
        if hold is not None and stop is not None and hold < stop - 1e-12:
            raise ValueError("hold-complete event must not precede low-speed entry")
        for name, value in event_values.items():
            object.__setattr__(self, name, value)

    @property
    def raw_simulation_time(self) -> np.ndarray:
        """Compatibility alias for the raw world ``current_time`` trace."""

        return self.raw_simulation_time_s

    @property
    def raw_physics_step(self) -> np.ndarray:
        """Compatibility alias for the raw world step-index trace."""

        return self.raw_physics_step_index

    @property
    def terminal_brake_start_time_s(self) -> float | None:
        """Compatibility alias for the runner-owned brake-start event."""

        return self.diagnostic_terminal_brake_start_time_s

    @property
    def goal_entry_time_s(self) -> float | None:
        """Compatibility alias for the runner-owned goal-entry event."""

        return self.diagnostic_goal_entry_time_s

    @property
    def stop_below_speed_time_s(self) -> float | None:
        """Compatibility alias for the runner-owned low-speed event."""

        return self.diagnostic_stop_below_speed_time_s

    @property
    def diagnostic_stop_below_0p05_time_s(self) -> float | None:
        """Alias using the policy's explicit 0.05 m/s threshold name."""

        return self.diagnostic_stop_below_speed_time_s

    @property
    def hold_complete_time_s(self) -> float | None:
        """Compatibility alias for the runner-owned hold-complete event."""

        return self.diagnostic_hold_complete_time_s

    @property
    def policy_elapsed_s(self) -> np.ndarray:
        """Action-aligned policy elapsed-time diagnostics for V2."""

        return self.elapsed_s_action_aligned

    @property
    def success(self) -> bool:
        return self.termination_reason == "success"

    def to_metrics_dict(self) -> dict[str, object]:
        metrics: dict[str, object] = {
            "episode_id": self.episode_id,
            "scene_id": self.scene_id,
            "difficulty": self.difficulty,
            "policy": self.policy_name,
            "termination_reason": self.termination_reason,
            "success": float(self.success),
            "collision": float(self.termination_reason == "collision"),
            "timeout": float(self.termination_reason == "timeout"),
            "runtime_error": float(self.termination_reason == "runtime_error"),
            "action_blocks": self.action_blocks,
            "elapsed_s": self.elapsed_s,
            "final_distance_m": self.final_distance_m,
            "path_length_m": self.path_length_m,
            "reference_length_m": self.reference_length_m,
            "spl": self.spl,
            "goal_progress_m": self.goal_progress_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "time_to_goal_s": self.time_to_goal_s,
            "runtime_error_message": self.runtime_error,
            "diagnostic_stop_hold_passed": self.diagnostic_stop_hold_passed,
            "diagnostic_goal_entry_action_block": self.diagnostic_goal_entry_action_block,
            "diagnostic_settling_blocks": self.diagnostic_settling_blocks,
            "diagnostic_settling_runtime_error": self.diagnostic_settling_runtime_error,
            "reference_curvature_1_m": self.reference_curvature_1_m.tolist(),
            "feedback_curvature_1_m": self.feedback_curvature_1_m.tolist(),
            "command_curvature_1_m": self.command_curvature_1_m.tolist(),
            "lateral_error_m": self.lateral_error_m.tolist(),
            "heading_error_rad": self.heading_error_rad.tolist(),
            "main_cap_rad_s": self.main_cap_rad_s.tolist(),
            "auxiliary_feedforward_rad": self.auxiliary_feedforward_rad.tolist(),
            "auxiliary_feedback_rad": self.auxiliary_feedback_rad.tolist(),
        }
        if self.policy_name == "oracle_trajectory_v2":
            metrics.update(
                {
                    "projection_s_m": self.projection_s_m.tolist(),
                    "delay_distance_m": self.delay_distance_m.tolist(),
                    "query_s_m": self.query_s_m.tolist(),
                    "pure_pursuit_curvature_1_m": self.pure_pursuit_curvature_1_m.tolist(),
                    "profile_curvature_1_m": self.profile_curvature_1_m.tolist(),
                    "blend_weight": self.blend_weight.tolist(),
                    "budget_infeasible": self.budget_infeasible.tolist(),
                    "terminal_runway_s": self.terminal_runway_s.tolist(),
                    "terminal_brake_start_s": self.terminal_brake_start_s.tolist(),
                    "goal_entry_s": self.goal_entry_s.tolist(),
                    "stop_below_0p05_s": self.stop_below_0p05_s.tolist(),
                    "hold_complete_s": self.hold_complete_s.tolist(),
                    "elapsed_s_action_aligned": self.elapsed_s_action_aligned.tolist(),
                    "diagnostic_terminal_brake_start_time_s": self.diagnostic_terminal_brake_start_time_s,
                    "diagnostic_goal_entry_time_s": self.diagnostic_goal_entry_time_s,
                    "diagnostic_stop_below_speed_time_s": self.diagnostic_stop_below_speed_time_s,
                    "diagnostic_stop_below_0p05_time_s": self.diagnostic_stop_below_speed_time_s,
                    "diagnostic_hold_complete_time_s": self.diagnostic_hold_complete_time_s,
                    "terminal_brake_start_time_s": self.diagnostic_terminal_brake_start_time_s,
                    "goal_entry_time_s": self.diagnostic_goal_entry_time_s,
                    "stop_below_speed_time_s": self.diagnostic_stop_below_speed_time_s,
                    "hold_complete_time_s": self.diagnostic_hold_complete_time_s,
                    "policy_elapsed_s": self.elapsed_s_action_aligned.tolist(),
                }
            )
        return metrics


def world_goal_to_base(
    *,
    goal_world_xy: object,
    position_world_xy: object,
    yaw_world_rad: float,
) -> np.ndarray:
    """Transform a world-frame goal into the current yaw-only base frame."""

    goal = np.asarray(goal_world_xy, dtype=np.float64)
    position = np.asarray(position_world_xy, dtype=np.float64)
    yaw = float(yaw_world_rad)
    if goal.shape != (2,) or position.shape != (2,) or not np.all(np.isfinite([goal, position])):
        raise ValueError("goal and position must be finite XY vectors")
    if not math.isfinite(yaw):
        raise ValueError("yaw_world_rad must be finite")
    delta = goal - position
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray(
        (
            cosine * delta[0] + sine * delta[1],
            -sine * delta[0] + cosine * delta[1],
        ),
        dtype=np.float64,
    )


def _validated_action(action: object, protocol: ActionProtocol) -> np.ndarray:
    try:
        values = np.asarray(action, dtype=np.float32)
    except (TypeError, ValueError) as error:
        raise ValueError("policy action block has invalid numeric data") from error
    if values.shape != (protocol.action_block_dim,):
        raise ValueError(
            f"policy action block must have shape ({protocol.action_block_dim},)"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("policy action block must contain finite values")
    bounds = protocol.block_bounds_numpy()
    if np.any(values < bounds[:, 0]) or np.any(values > bounds[:, 1]):
        raise ValueError("policy action block is outside the action protocol bounds")
    return values.copy()


def _distance_to_goal(step: RuntimeStep, goal_world_xy: np.ndarray) -> float:
    return float(np.linalg.norm(goal_world_xy - step.observation.position_world_xy))


def _optional_telemetry_array(
    values: list[float | int | None], *, integer: bool = False
) -> np.ndarray:
    """Convert optional step telemetry to a state-aligned numeric trace.

    Missing values stay explicit as NaN.  If every value is measured, integer
    traces retain an integer dtype; mixed/legacy traces remain float so the
    unavailable entries cannot be mistaken for measured zeroes.
    """

    result = np.asarray(
        [np.nan if value is None else value for value in values], dtype=np.float64
    )
    if integer and result.size and np.all(np.isfinite(result)):
        return result.astype(np.int64)
    return result


def _scene_id(scene_uri: str) -> str:
    return PurePosixPath(scene_uri).stem


_DRIVE_DIRECTION_CODES = {"reverse": -1, "forward": 1}
_BRAKING_STATE_CODES = {
    "tracking": 1,
    "direction_change": 2,
    "goal_braking": 3,
    "goal_reached": 4,
    "tracking_infeasible": 5,
}

_TRAJECTORY_DIAGNOSTIC_NAMES = (
    "reference_curvature_1_m",
    "feedback_curvature_1_m",
    "command_curvature_1_m",
    "lateral_error_m",
    "heading_error_rad",
    "main_cap_rad_s",
    "auxiliary_feedforward_rad",
    "auxiliary_feedback_rad",
)

# These names mirror the diagnostics emitted by
# ``DelayAwareTrajectoryOraclePolicy``.  They are kept separate from the V1
# tuple so legacy policies retain their historical zero-fill behavior.
_V2_TRAJECTORY_DIAGNOSTIC_NAMES = (
    "projection_s_m",
    "delay_distance_m",
    "query_s_m",
    "pure_pursuit_curvature_1_m",
    "profile_curvature_1_m",
    "blend_weight",
    "budget_infeasible",
    "terminal_runway_s",
    "terminal_brake_start_s",
    "goal_entry_s",
    "stop_below_0p05_s",
    "hold_complete_s",
    "elapsed_s",
)


def _finite_policy_diagnostic(value: object, default: float = 0.0) -> float:
    """Return a finite scalar diagnostic, defaulting safely when absent."""

    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return numeric if math.isfinite(numeric) else float(default)


class _InvalidPolicyDiagnostic(ValueError):
    """A policy diagnostic was missing or not a finite real scalar."""


def _trajectory_policy_diagnostic(
    diagnostics: Mapping[object, object], name: str, *, required: bool = False
) -> float:
    """Read one V1 diagnostic, defaulting only when its key is absent."""

    if name not in diagnostics:
        if required:
            raise _InvalidPolicyDiagnostic(
                f"policy diagnostic {name} is missing"
            )
        return 0.0
    value = diagnostics[name]
    if value is None:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        ) from error
    # Inspect dtype before any float cast so complex values cannot silently
    # discard their imaginary component and string/object values cannot be
    # accepted by a permissive numeric conversion.
    if array.ndim != 0 or array.dtype.kind not in "iuf":
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    try:
        numeric = float(array)
    except (TypeError, ValueError, OverflowError) as error:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        ) from error
    if not math.isfinite(numeric):
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    return numeric


def _v2_policy_diagnostic(
    diagnostics: Mapping[object, object], name: str
) -> float | bool:
    """Read one V2 action-aligned scalar without permissive coercion.

    V2 evidence is provenance, not a best-effort descriptive trace.  Missing,
    malformed, non-finite, and wrong-type values therefore invalidate the
    action before it is sent to the runtime.  The boolean budget diagnostic is
    intentionally kept distinct from numeric values.
    """

    if name not in diagnostics:
        raise _InvalidPolicyDiagnostic(f"policy diagnostic {name} is missing")
    value = diagnostics[name]
    if name == "budget_infeasible":
        if not isinstance(value, (bool, np.bool_)):
            raise _InvalidPolicyDiagnostic(
                "policy diagnostic budget_infeasible must be a boolean"
            )
        return bool(value)
    if value is None:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        ) from error
    if array.ndim != 0 or array.dtype.kind not in "iuf":
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    try:
        numeric = float(array)
    except (TypeError, ValueError, OverflowError) as error:
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        ) from error
    if not math.isfinite(numeric):
        raise _InvalidPolicyDiagnostic(
            f"policy diagnostic {name} must be a finite real scalar"
        )
    return numeric


def _policy_diagnostic_sample(
    policy: BenchmarkPolicy,
) -> tuple[object, ...]:
    diagnostics = getattr(policy, "diagnostics", {})
    if not isinstance(diagnostics, Mapping):
        diagnostics = {}
    is_v2 = str(getattr(policy, "policy_name", "")) == "oracle_trajectory_v2"
    reference_index = diagnostics.get("reference_index", -1)
    cross_track_error = diagnostics.get("cross_track_error_m", 0.0)
    stopped_cycles = diagnostics.get("stopped_cycles", 0)
    position_error = diagnostics.get("position_error_m", 0.0)
    yaw_error = diagnostics.get("yaw_error_rad", 0.0)
    auxiliary_error = diagnostics.get("auxiliary_error_rad", 0.0)
    try:
        reference_index = int(reference_index)
    except (TypeError, ValueError, OverflowError):
        reference_index = -1
    try:
        cross_track_error = float(cross_track_error)
    except (TypeError, ValueError, OverflowError):
        cross_track_error = 0.0
    if not math.isfinite(cross_track_error):
        cross_track_error = 0.0
    try:
        stopped_cycles = int(stopped_cycles)
    except (TypeError, ValueError, OverflowError):
        stopped_cycles = 0
    numeric_errors = []
    for value in (position_error, yaw_error, auxiliary_error):
        numeric_errors.append(_finite_policy_diagnostic(value))
    trajectory_diagnostics = tuple(
        _trajectory_policy_diagnostic(diagnostics, name, required=is_v2)
        for name in _TRAJECTORY_DIAGNOSTIC_NAMES
    )
    sample: tuple[object, ...] = (
        reference_index,
        cross_track_error,
        _DRIVE_DIRECTION_CODES.get(diagnostics.get("active_drive_direction"), 0),
        _DRIVE_DIRECTION_CODES.get(diagnostics.get("desired_drive_direction"), 0),
        _DRIVE_DIRECTION_CODES.get(diagnostics.get("pending_drive_direction"), 0),
        stopped_cycles,
        _BRAKING_STATE_CODES.get(diagnostics.get("braking_state"), 0),
        *numeric_errors,
        *trajectory_diagnostics,
    )
    if is_v2:
        sample += tuple(
            _v2_policy_diagnostic(diagnostics, name)
            for name in _V2_TRAJECTORY_DIAGNOSTIC_NAMES
        )
    return sample


def run_episode(
    runtime: BenchmarkRuntime,
    policy: BenchmarkPolicy,
    episode: BenchmarkEpisode,
    *,
    action_protocol: ActionProtocol,
    config: RunnerConfig,
    oracle_path_xy: np.ndarray | None = None,
    kinodynamic_plan: KinodynamicPlanResult | None = None,
    dynamic_plan: DynamicPlanResult | None = None,
) -> EpisodeRun:
    """Run one strict episode and preserve failures as runtime-error results."""

    if not isinstance(episode, BenchmarkEpisode):
        raise TypeError("episode must be a BenchmarkEpisode")
    if not isinstance(action_protocol, ActionProtocol):
        raise TypeError("action_protocol must be an ActionProtocol")
    if not isinstance(config, RunnerConfig):
        raise TypeError("config must be a RunnerConfig")
    policy_name = str(getattr(policy, "policy_name", type(policy).__name__))
    is_v2_policy = policy_name == "oracle_trajectory_v2"
    privileged = bool(getattr(policy, "privileged", False))
    goal_world_xy = np.asarray(episode.goal_position_world_m[:2], dtype=np.float64)

    positions: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    clearances: list[float] = []
    contacts: list[bool] = []
    collisions: list[bool] = []
    yaws: list[float] = []
    simulation_times: list[float | None] = []
    physics_steps: list[int | None] = []
    raw_simulation_times: list[float | None] = []
    raw_physics_steps: list[int | None] = []
    rolls: list[float | None] = []
    settling_mask: list[bool] = []
    speed_trace: list[float] = []
    proprio_states: list[np.ndarray] = []
    policy_diagnostics: list[tuple[object, ...]] = []
    termination_reason = "runtime_error"
    runtime_error: str | None = None
    diagnostic_settling_runtime_error: str | None = None
    diagnostic_stop_hold_passed: bool | None = None
    diagnostic_goal_entry_action_block: int | None = None
    diagnostic_settling_blocks = 0
    diagnostic_hold_run = 0
    diagnostic_settling_active = False
    nominal_success = False
    initial_distance = float(episode.euclidean_distance_m)
    final_distance = initial_distance
    latest_policy_diagnostic = None
    reset_simulation_time_s: float | None = None
    last_relative_time_s = 0.0
    diagnostic_terminal_brake_start_time_s: float | None = None
    diagnostic_goal_entry_time_s: float | None = None
    diagnostic_stop_below_speed_time_s: float | None = None
    diagnostic_hold_complete_time_s: float | None = None

    def relative_step_time(step: RuntimeStep, action_index: int) -> float:
        """Return a finite monotonic time relative to the reset endpoint."""

        nonlocal last_relative_time_s
        if reset_simulation_time_s is not None and step.simulation_time_s is not None:
            relative = float(step.simulation_time_s - reset_simulation_time_s)
        else:
            relative = float(action_index) / float(config.replan_hz)
        if not math.isfinite(relative) or relative < -1e-12:
            raise ValueError("relative simulation time must be finite and non-negative")
        relative = max(0.0, relative)
        if relative < last_relative_time_s - 1e-9:
            raise ValueError("relative simulation time must be monotonic")
        last_relative_time_s = relative
        return relative

    def record_brake_start(sample: tuple[object, ...], action_start_time_s: float) -> None:
        """Derive the first brake event from an action-aligned V2 sample."""

        nonlocal diagnostic_terminal_brake_start_time_s
        if not is_v2_policy or diagnostic_terminal_brake_start_time_s is not None:
            return
        # The V2 suffix starts after the historical 18 fields.  A -1 policy
        # sentinel explicitly means that braking has not begun; the braking
        # state code remains a useful fallback for policies that expose only
        # the established state diagnostic.
        terminal_index = 18 + _V2_TRAJECTORY_DIAGNOSTIC_NAMES.index(
            "terminal_brake_start_s"
        )
        terminal_value = float(sample[terminal_index])
        braking_code = int(sample[6])
        if terminal_value >= 0.0 or braking_code == _BRAKING_STATE_CODES["goal_braking"]:
            diagnostic_terminal_brake_start_time_s = float(action_start_time_s)

    def record_goal_entry(event_time_s: float) -> None:
        nonlocal diagnostic_goal_entry_time_s
        if not is_v2_policy:
            return
        if diagnostic_goal_entry_time_s is not None:
            return
        if (
            diagnostic_terminal_brake_start_time_s is not None
            and diagnostic_terminal_brake_start_time_s > event_time_s + 1e-12
        ):
            raise ValueError("terminal brake start must not follow goal entry")
        diagnostic_goal_entry_time_s = float(event_time_s)

    def record_low_speed(event_time_s: float) -> None:
        nonlocal diagnostic_stop_below_speed_time_s
        if not is_v2_policy:
            return
        if diagnostic_stop_below_speed_time_s is None:
            if diagnostic_goal_entry_time_s is None:
                raise ValueError("low-speed event requires goal entry")
            if event_time_s < diagnostic_goal_entry_time_s - 1e-12:
                raise ValueError("low-speed event must not precede goal entry")
            diagnostic_stop_below_speed_time_s = float(event_time_s)

    def record_hold_complete(event_time_s: float) -> None:
        nonlocal diagnostic_hold_complete_time_s
        if not is_v2_policy:
            return
        if diagnostic_hold_complete_time_s is None:
            if diagnostic_stop_below_speed_time_s is None:
                raise ValueError("hold-complete event requires low-speed entry")
            if event_time_s < diagnostic_stop_below_speed_time_s - 1e-12:
                raise ValueError("hold-complete event must not precede low-speed entry")
            diagnostic_hold_complete_time_s = float(event_time_s)

    try:
        current = runtime.reset(episode)
        if not isinstance(current, RuntimeStep):
            raise TypeError("runtime.reset() must return RuntimeStep")
        reset_simulation_time_s = current.simulation_time_s
        relative_step_time(current, 0)
        positions.append(current.observation.position_world_xy.copy())
        yaws.append(current.observation.yaw_world_rad)
        simulation_times.append(current.simulation_time_s)
        physics_steps.append(current.physics_step_index)
        raw_simulation_times.append(current.raw_simulation_time_s)
        raw_physics_steps.append(current.raw_physics_step_index)
        rolls.append(current.roll_world_rad)
        proprio_states.append(current.observation.proprio_history[-1].copy())
        speed_trace.append(
            float(np.linalg.norm(current.observation.proprio_history[-1, :2]))
        )
        clearances.append(current.truth_clearance_m)
        contacts.append(current.physical_contact)
        collisions.append(current.collision)
        initial_distance = _distance_to_goal(current, goal_world_xy)
        final_distance = initial_distance

        dynamic_oracle = isinstance(policy, DynamicOraclePolicy)
        resolved_dynamic_plan = dynamic_plan
        resolved_oracle_path = oracle_path_xy
        resolved_kinodynamic_plan = kinodynamic_plan
        if dynamic_oracle and not privileged:
            raise PermissionError("DynamicOraclePolicy must be privileged")
        if not dynamic_oracle and resolved_dynamic_plan is not None:
            raise ValueError(
                "dynamic_plan injection requires DynamicOraclePolicy"
            )
        if dynamic_oracle and resolved_dynamic_plan is None:
            provider = getattr(runtime, "dynamic_plan", None)
            if not callable(provider):
                raise ValueError(
                    "DynamicOraclePolicy requires a dynamic plan from the caller or runtime"
                )
            resolved_dynamic_plan = provider(episode)
        if dynamic_oracle:
            if not isinstance(resolved_dynamic_plan, DynamicPlanResult):
                raise TypeError("dynamic plan provider returned an invalid result")
            if resolved_dynamic_plan.status != "success":
                raise ValueError(
                    f"dynamic planning failed: {resolved_dynamic_plan.status}"
                )
        if privileged and not dynamic_oracle and resolved_oracle_path is None:
            provider = getattr(runtime, "oracle_path_xy", None)
            if not callable(provider):
                raise ValueError(
                    "privileged policy requires an oracle path from the caller or runtime"
                )
            resolved_oracle_path = provider(episode)

        if privileged and not dynamic_oracle and resolved_kinodynamic_plan is None:
            provider = getattr(runtime, "kinodynamic_plan", None)
            if not callable(provider):
                raise ValueError(
                    "privileged policy requires a kinodynamic plan from the caller or runtime"
                )
            resolved_kinodynamic_plan = provider(episode)
        if privileged and not dynamic_oracle:
            if not isinstance(resolved_kinodynamic_plan, KinodynamicPlanResult):
                raise TypeError("kinodynamic plan provider returned an invalid result")
            if resolved_kinodynamic_plan.status != "success":
                raise ValueError(
                    f"kinodynamic planning failed: {resolved_kinodynamic_plan.status}"
                )

        context = EpisodeContext(
            episode_id=episode.episode_id,
            privileged=privileged,
            oracle_path_xy=(
                resolved_oracle_path if privileged and not dynamic_oracle else None
            ),
            action_protocol_id=action_protocol.protocol_id,
            kinodynamic_reference=(
                resolved_kinodynamic_plan.waypoints
                if privileged and not dynamic_oracle and resolved_kinodynamic_plan is not None
                else None
            ),
            dynamic_reference=(
                resolved_dynamic_plan.waypoints
                if dynamic_oracle and resolved_dynamic_plan is not None
                else None
            ),
        )
        policy.reset(context)

        termination = episode_termination(
            final_distance,
            current.collision,
            0,
            config.max_action_blocks,
            config.goal_tolerance_m,
        )
        if termination == "success":
            nominal_success = True
            diagnostic_goal_entry_action_block = 0
            record_goal_entry(0.0)
            if config.diagnostic_settling_budget_s > 0.0:
                # There is no real policy act when an episode starts inside
                # the goal tolerance.  Capture one finite post-reset sample
                # as the cache for every subsequent settling block.
                latest_policy_diagnostic = _policy_diagnostic_sample(policy)
                diagnostic_settling_active = True
                diagnostic_stop_hold_passed = False
                if speed_trace[-1] < config.diagnostic_stop_speed_m_s:
                    # The reset endpoint is the measured low-speed event when
                    # an episode starts inside the goal tolerance.  It is not
                    # a post-entry hold sample: the consecutive zero-action
                    # hold still begins at the next settling endpoint.
                    record_low_speed(0.0)
        for block_index in range(1, config.max_action_blocks + 1):
            if termination in ("collision", "runtime_error"):
                break
            if termination == "success" and not diagnostic_settling_active:
                break
            settling_action = termination == "success" and diagnostic_settling_active
            action_start_time_s = relative_step_time(current, block_index - 1)
            if settling_action:
                if diagnostic_settling_blocks >= config.max_diagnostic_settling_blocks:
                    diagnostic_settling_active = False
                    break
                action = np.zeros(action_protocol.action_block_dim, dtype=np.float32)
                # No policy act() occurs during settling.  Retain the latest
                # policy diagnostics and mark the block explicitly below.
                if latest_policy_diagnostic is None:
                    raise RuntimeError(
                        "diagnostic settling has no cached policy diagnostics"
                    )
                diagnostic_sample = latest_policy_diagnostic
            else:
                if termination is not None:
                    break
                pointgoal_base = world_goal_to_base(
                    goal_world_xy=goal_world_xy,
                    position_world_xy=current.observation.position_world_xy,
                    yaw_world_rad=current.observation.yaw_world_rad,
                )
                action = _validated_action(
                    policy.act(current.observation, pointgoal_base),
                    action_protocol,
                )
                diagnostic_sample = _policy_diagnostic_sample(policy)
                latest_policy_diagnostic = diagnostic_sample
                record_brake_start(diagnostic_sample, action_start_time_s)
            if settling_action:
                diagnostic_settling_blocks += 1
            current = runtime.execute(action)
            if not isinstance(current, RuntimeStep):
                raise TypeError("runtime.execute() must return RuntimeStep")
            post_action_time_s = relative_step_time(current, block_index)
            actions.append(action)
            settling_mask.append(settling_action)
            policy_diagnostics.append(diagnostic_sample)
            positions.append(current.observation.position_world_xy.copy())
            yaws.append(current.observation.yaw_world_rad)
            simulation_times.append(current.simulation_time_s)
            physics_steps.append(current.physics_step_index)
            raw_simulation_times.append(current.raw_simulation_time_s)
            raw_physics_steps.append(current.raw_physics_step_index)
            rolls.append(current.roll_world_rad)
            proprio_states.append(current.observation.proprio_history[-1].copy())
            speed_trace.append(
                float(np.linalg.norm(current.observation.proprio_history[-1, :2]))
            )
            clearances.append(current.truth_clearance_m)
            contacts.append(current.physical_contact)
            collisions.append(current.collision)
            final_distance = _distance_to_goal(current, goal_world_xy)
            # Collision always wins, including during diagnostic settling.
            if current.collision:
                termination = "collision"
                if diagnostic_settling_active:
                    diagnostic_stop_hold_passed = False
                    diagnostic_settling_active = False
                break

            if not nominal_success and final_distance <= config.goal_tolerance_m:
                nominal_success = True
                termination = "success"
                diagnostic_goal_entry_action_block = block_index
                record_goal_entry(post_action_time_s)
                if config.diagnostic_settling_budget_s > 0.0:
                    diagnostic_settling_active = True
                    diagnostic_stop_hold_passed = False
                    if is_v2_policy:
                        # The goal-entry endpoint is the nominal terminal
                        # event, not a post-entry zero-action hold sample.
                        # Record a low-speed observation at entry when
                        # applicable, but start the consecutive hold run at
                        # the next endpoint so it matches the independent
                        # acceptance harness.  Legacy baseline/V1 settling
                        # retains its historical counting behavior.
                        diagnostic_hold_run = 0
                        if speed_trace[-1] < config.diagnostic_stop_speed_m_s:
                            record_low_speed(post_action_time_s)
                    elif speed_trace[-1] < config.diagnostic_stop_speed_m_s:
                        diagnostic_hold_run = 1
                        if diagnostic_hold_run >= config.diagnostic_stop_hold_samples:
                            diagnostic_stop_hold_passed = True
                            diagnostic_settling_active = False
                continue

            if nominal_success:
                if diagnostic_settling_active:
                    if (
                        final_distance <= config.goal_tolerance_m
                        and speed_trace[-1] < config.diagnostic_stop_speed_m_s
                    ):
                        diagnostic_hold_run += 1
                        record_low_speed(post_action_time_s)
                    else:
                        diagnostic_hold_run = 0
                    if diagnostic_hold_run >= config.diagnostic_stop_hold_samples:
                        diagnostic_stop_hold_passed = True
                        record_hold_complete(post_action_time_s)
                        diagnostic_settling_active = False
                continue

            termination = episode_termination(
                final_distance,
                current.collision,
                block_index,
                config.max_action_blocks,
                config.goal_tolerance_m,
            )
        if termination is None:
            raise RuntimeError("episode loop ended without a terminal condition")
        termination_reason = termination
    except Exception as error:
        message = f"{type(error).__name__}: {error}"
        if isinstance(error, _InvalidPolicyDiagnostic):
            # A present malformed trajectory diagnostic invalidates the trace,
            # including when it is observed during post-goal settling.  Do not
            # preserve nominal success for an untrustworthy action-aligned
            # record.
            termination_reason = "runtime_error"
            runtime_error = message
            diagnostic_settling_active = False
            diagnostic_stop_hold_passed = False
        elif nominal_success and diagnostic_settling_active:
            # Formal success occurred at goal entry; preserve it while
            # exposing the post-goal diagnostic failure separately.
            diagnostic_settling_runtime_error = message
            diagnostic_stop_hold_passed = False
            diagnostic_settling_active = False
            termination_reason = "success"
        else:
            termination_reason = "runtime_error"
            runtime_error = message

    positions_array = (
        np.stack(positions).astype(np.float64)
        if positions
        else np.empty((0, 2), dtype=np.float64)
    )
    actions_array = (
        np.stack(actions).astype(np.float32)
        if actions
        else np.empty((0, action_protocol.action_block_dim), dtype=np.float32)
    )
    proprio_array = (
        np.stack(proprio_states).astype(np.float32)
        if proprio_states
        else np.empty((0, 17), dtype=np.float32)
    )
    diagnostic_array = (
        np.asarray(policy_diagnostics, dtype=np.float64)
        if policy_diagnostics
        else np.empty(
            (0, 18 + len(_V2_TRAJECTORY_DIAGNOSTIC_NAMES) if is_v2_policy else 18),
            dtype=np.float64,
        )
    )
    if nominal_success and diagnostic_goal_entry_action_block is not None:
        nominal_state_count = min(
            positions_array.shape[0], diagnostic_goal_entry_action_block + 1
        )
    else:
        nominal_state_count = positions_array.shape[0]
    nominal_positions = positions_array[:nominal_state_count]
    path_length_m = path_length(nominal_positions)
    success = termination_reason == "success"
    spl = success_weighted_path_length(
        success,
        path_length_m,
        episode.oracle_path_length_m,
    )
    action_blocks = len(actions)
    nominal_action_blocks = (
        diagnostic_goal_entry_action_block
        if nominal_success and diagnostic_goal_entry_action_block is not None
        else action_blocks
    )
    # Formal elapsed/time-to-goal remain anchored to the nominal terminal
    # event.  Diagnostic settling blocks are present in the traces and in
    # ``action_blocks`` but do not redefine the formal timing metrics.
    elapsed_s = nominal_action_blocks / config.replan_hz
    return EpisodeRun(
        episode_id=episode.episode_id,
        scene_id=_scene_id(episode.scene_uri),
        difficulty=episode.difficulty,
        policy_name=policy_name,
        termination_reason=termination_reason,
        action_blocks=action_blocks,
        elapsed_s=elapsed_s,
        final_distance_m=final_distance,
        path_length_m=path_length_m,
        reference_length_m=episode.oracle_path_length_m,
        spl=spl,
        goal_progress_m=initial_distance - final_distance,
        minimum_clearance_m=min(clearances) if clearances else None,
        time_to_goal_s=elapsed_s if success else None,
        runtime_error=runtime_error,
        positions_world_xy=positions_array,
        actions=actions_array,
        clearances_m=np.asarray(clearances, dtype=np.float64),
        physical_contacts=np.asarray(contacts, dtype=bool),
        collisions=np.asarray(collisions, dtype=bool),
        yaw_world_rad=np.asarray(yaws, dtype=np.float64),
        linear_velocity_world_m_s=proprio_array[:, 0:3],
        angular_velocity_world_rad_s=proprio_array[:, 3:6],
        joint_position_rad=proprio_array[:, 9:12],
        joint_velocity_rad_s=proprio_array[:, 12:15],
        applied_action=proprio_array[:, 15:17],
        policy_reference_index=diagnostic_array[:, 0].astype(np.int64),
        policy_cross_track_error_m=diagnostic_array[:, 1],
        policy_active_drive_direction=diagnostic_array[:, 2].astype(np.int8),
        policy_desired_drive_direction=diagnostic_array[:, 3].astype(np.int8),
        policy_pending_drive_direction=diagnostic_array[:, 4].astype(np.int8),
        policy_stopped_cycles=diagnostic_array[:, 5].astype(np.int64),
        policy_braking_state_code=diagnostic_array[:, 6].astype(np.int8),
        reference_curvature_1_m=diagnostic_array[:, 10],
        feedback_curvature_1_m=diagnostic_array[:, 11],
        command_curvature_1_m=diagnostic_array[:, 12],
        lateral_error_m=diagnostic_array[:, 13],
        heading_error_rad=diagnostic_array[:, 14],
        main_cap_rad_s=diagnostic_array[:, 15],
        auxiliary_feedforward_rad=diagnostic_array[:, 16],
        auxiliary_feedback_rad=diagnostic_array[:, 17],
        projection_s_m=(
            diagnostic_array[:, 18]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        delay_distance_m=(
            diagnostic_array[:, 19]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        query_s_m=(
            diagnostic_array[:, 20]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        pure_pursuit_curvature_1_m=(
            diagnostic_array[:, 21]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        profile_curvature_1_m=(
            diagnostic_array[:, 22]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        blend_weight=(
            diagnostic_array[:, 23]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        budget_infeasible=(
            diagnostic_array[:, 24].astype(bool)
            if is_v2_policy
            else np.empty(0, dtype=bool)
        ),
        terminal_runway_s=(
            diagnostic_array[:, 25]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        terminal_brake_start_s=(
            diagnostic_array[:, 26]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        goal_entry_s=(
            diagnostic_array[:, 27]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        stop_below_0p05_s=(
            diagnostic_array[:, 28]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        hold_complete_s=(
            diagnostic_array[:, 29]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        elapsed_s_action_aligned=(
            diagnostic_array[:, 30]
            if is_v2_policy
            else np.empty(0, dtype=np.float64)
        ),
        dynamic_reference_index=diagnostic_array[:, 0].astype(np.int64),
        dynamic_position_error_m=diagnostic_array[:, 7],
        dynamic_yaw_error_rad=diagnostic_array[:, 8],
        dynamic_auxiliary_error_rad=diagnostic_array[:, 9],
        simulation_time_s=_optional_telemetry_array(simulation_times),
        physics_step_index=_optional_telemetry_array(physics_steps, integer=True),
        raw_simulation_time_s=_optional_telemetry_array(raw_simulation_times),
        raw_physics_step_index=_optional_telemetry_array(
            raw_physics_steps, integer=True
        ),
        roll_world_rad=_optional_telemetry_array(rolls),
        diagnostic_stop_hold_passed=diagnostic_stop_hold_passed,
        diagnostic_goal_entry_action_block=diagnostic_goal_entry_action_block,
        diagnostic_settling_blocks=diagnostic_settling_blocks,
        diagnostic_settling_runtime_error=diagnostic_settling_runtime_error,
        diagnostic_settling_mask=np.asarray(settling_mask, dtype=bool),
        diagnostic_terminal_brake_start_time_s=(
            diagnostic_terminal_brake_start_time_s if is_v2_policy else None
        ),
        diagnostic_goal_entry_time_s=(
            diagnostic_goal_entry_time_s if is_v2_policy else None
        ),
        diagnostic_stop_below_speed_time_s=(
            diagnostic_stop_below_speed_time_s if is_v2_policy else None
        ),
        diagnostic_hold_complete_time_s=(
            diagnostic_hold_complete_time_s if is_v2_policy else None
        ),
    )
