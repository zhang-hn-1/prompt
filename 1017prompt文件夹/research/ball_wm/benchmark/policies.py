"""Fair policy adapters for Ball PointGoal Benchmark V1."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

import numpy as np

from benchmark.drive_model import DriveModel
from benchmark.dynamic_lattice_planning import DynamicWaypoint
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.trajectory_tracking import (
    SpatialReferenceProfile,
    blend_tracking_curvature,
    curvature_blend_weight,
    query_delay_aware_reference,
)
from training.action_protocol import ActionProtocol


BENCHMARK_ACTION_PROTOCOL_ID = "ball_action_v2_aux30"
RGB_SHAPE = (480, 640, 3)
PROPRIO_DIM = 17
DEFAULT_DRIVE_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)
DEFAULT_YAW_RATE_FEEDBACK_GAIN_S = 0.15
_DEFAULT_DRIVE_MODEL: DriveModel | None = None


def _default_drive_model() -> DriveModel:
    global _DEFAULT_DRIVE_MODEL
    if _DEFAULT_DRIVE_MODEL is None:
        _DEFAULT_DRIVE_MODEL = DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
    return _DEFAULT_DRIVE_MODEL


def calibrated_auxiliary_angle(
    desired_curvature_1_m: float,
    *,
    main_velocity_rad_s: float,
) -> float:
    """Interpolate the measured curvature-to-auxiliary relationship."""

    return _default_drive_model().auxiliary_for_curvature(
        main_velocity_rad_s, desired_curvature_1_m
    )


def calibrated_stopping_distance(speed_m_s: float) -> float:
    """Interpolate measured stopping distance and linearly extrapolate above it."""

    return _default_drive_model().stopping_distance(abs(float(speed_m_s)), "forward")


def adaptive_lookahead_distance(speed_m_s: float) -> float:
    """Return a speed-dependent path lookahead covering steering response lag."""

    speed = abs(float(speed_m_s))
    if not math.isfinite(speed):
        raise ValueError("speed_m_s must be finite")
    return float(np.clip(0.45 + 0.8 * speed, 0.45, 0.9))


def _validate_yaw_rate_feedback_gain_s(value: object) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(
            "yaw_rate_feedback_gain_s must be finite and non-negative"
        )
    try:
        gain = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "yaw_rate_feedback_gain_s must be finite and non-negative"
        ) from error
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("yaw_rate_feedback_gain_s must be finite and non-negative")
    return gain


def _validate_trajectory_scalar(
    value: object,
    *,
    name: str,
    minimum: float = 0.0,
    strictly_positive: bool = False,
) -> float:
    """Validate one V1 trajectory-policy scalar without bool coercion."""

    if isinstance(value, (bool, np.bool_)):
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}") from error
    valid = result > minimum if strictly_positive else result >= minimum
    if not math.isfinite(result) or not valid:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


def curvature_tracker_action(
    protocol: ActionProtocol,
    target_base_xy: object,
    *,
    current_speed_m_s: float,
    current_yaw_rate_rad_s: float,
    remaining_path_m: float,
) -> np.ndarray:
    """Track a base-frame path target with calibrated steering and braking."""

    target = _pointgoal(target_base_xy)
    speed = abs(float(current_speed_m_s))
    yaw_rate = float(current_yaw_rate_rad_s)
    remaining = float(remaining_path_m)
    if not np.isfinite([speed, yaw_rate, remaining]).all() or remaining < 0.0:
        raise ValueError("tracker state must be finite and remaining path non-negative")
    if remaining <= calibrated_stopping_distance(speed) + 0.1:
        return np.zeros(protocol.action_block_dim, dtype=np.float32)

    distance_squared = float(np.dot(target, target))
    curvature = 0.0 if distance_squared <= 1e-9 else 2.0 * float(target[1]) / distance_squared
    abs_curvature = abs(curvature)
    if abs_curvature >= 0.55:
        main_velocity = 0.5
    elif abs_curvature >= 0.25:
        main_velocity = 1.0
    else:
        main_velocity = min(2.0, float(protocol.maximum[0]))
    if remaining < 2.0:
        main_velocity = min(main_velocity, 1.0)

    auxiliary = calibrated_auxiliary_angle(
        curvature,
        main_velocity_rad_s=main_velocity,
    )
    desired_yaw_rate = curvature * max(speed, 0.2)
    auxiliary -= 0.15 * (desired_yaw_rate - yaw_rate)
    return _repeated_control(
        protocol,
        main_velocity=main_velocity,
        auxiliary_angle=auxiliary,
    )


def _immutable_array(
    value: object,
    *,
    name: str,
    dtype: np.dtype | type,
    shape: tuple[int | None, ...],
    exact_dtype: bool = False,
) -> np.ndarray:
    original = np.asarray(value)
    if exact_dtype and original.dtype != np.dtype(dtype):
        raise ValueError(f"{name} must use {np.dtype(dtype)} values")
    try:
        array = np.asarray(value, dtype=dtype)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} has invalid numeric data") from error
    if array.ndim != len(shape) or any(
        expected is not None and actual != expected
        for actual, expected in zip(array.shape, shape)
    ):
        expected_text = ", ".join("N" if item is None else str(item) for item in shape)
        raise ValueError(f"{name} must have shape ({expected_text})")
    if 0 in array.shape:
        raise ValueError(f"{name} cannot be empty")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    result = array.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class PolicyObservation:
    """Only information available to a fair, map-free online policy."""

    stereo_left_rgb: np.ndarray
    stereo_right_rgb: np.ndarray
    proprio_history: np.ndarray
    action_history: np.ndarray
    position_world_xy: np.ndarray
    yaw_world_rad: float

    def __post_init__(self) -> None:
        left = _immutable_array(
            self.stereo_left_rgb,
            name="stereo_left_rgb",
            dtype=np.uint8,
            shape=RGB_SHAPE,
            exact_dtype=True,
        )
        right = _immutable_array(
            self.stereo_right_rgb,
            name="stereo_right_rgb",
            dtype=np.uint8,
            shape=RGB_SHAPE,
            exact_dtype=True,
        )
        proprio = _immutable_array(
            self.proprio_history,
            name="proprio_history",
            dtype=np.float32,
            shape=(None, PROPRIO_DIM),
        )
        actions = _immutable_array(
            self.action_history,
            name="action_history",
            dtype=np.float32,
            shape=(None, 10),
        )
        if proprio.shape[0] != actions.shape[0]:
            raise ValueError("proprio_history and action_history must have equal lengths")
        position = _immutable_array(
            self.position_world_xy,
            name="position_world_xy",
            dtype=np.float64,
            shape=(2,),
        )
        yaw = float(self.yaw_world_rad)
        if not math.isfinite(yaw):
            raise ValueError("yaw_world_rad must be finite")
        object.__setattr__(self, "stereo_left_rgb", left)
        object.__setattr__(self, "stereo_right_rgb", right)
        object.__setattr__(self, "proprio_history", proprio)
        object.__setattr__(self, "action_history", actions)
        object.__setattr__(self, "position_world_xy", position)
        object.__setattr__(self, "yaw_world_rad", yaw)


@dataclass(frozen=True)
class EpisodeContext:
    """Per-episode policy metadata with an explicit privilege boundary."""

    episode_id: str
    privileged: bool
    oracle_path_xy: np.ndarray | None
    action_protocol_id: str
    kinodynamic_reference: tuple[KinodynamicWaypoint, ...] | None = None
    dynamic_reference: tuple[DynamicWaypoint, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.episode_id, str) or not self.episode_id.strip():
            raise ValueError("episode_id must be a non-empty string")
        if not isinstance(self.privileged, bool):
            raise ValueError("privileged must be a boolean")
        if not isinstance(self.action_protocol_id, str) or not self.action_protocol_id.strip():
            raise ValueError("action_protocol_id must be a non-empty string")
        path = None
        if self.oracle_path_xy is not None:
            path = _immutable_array(
                self.oracle_path_xy,
                name="oracle_path_xy",
                dtype=np.float64,
                shape=(None, 2),
            )
            if path.shape[0] < 2:
                raise ValueError("oracle_path_xy must contain at least two points")
        reference = self.kinodynamic_reference
        if reference is not None:
            if not isinstance(reference, tuple) or len(reference) < 2:
                raise ValueError(
                    "kinodynamic_reference must contain at least two immutable waypoints"
                )
            if not all(isinstance(item, KinodynamicWaypoint) for item in reference):
                raise ValueError(
                    "kinodynamic_reference must contain KinodynamicWaypoint values"
                )
        dynamic_reference = self.dynamic_reference
        if dynamic_reference is not None:
            if not isinstance(dynamic_reference, tuple) or len(dynamic_reference) < 2:
                raise ValueError(
                    "dynamic_reference must contain at least two immutable waypoints"
                )
            if not all(isinstance(item, DynamicWaypoint) for item in dynamic_reference):
                raise ValueError(
                    "dynamic_reference must contain DynamicWaypoint values"
                )
            times = [item.time_from_start_s for item in dynamic_reference]
            if not np.isfinite(times).all() or any(
                later <= earlier for earlier, later in zip(times, times[1:])
            ):
                raise ValueError("dynamic_reference times must be finite and increasing")
        if not self.privileged and (
            path is not None or reference is not None or dynamic_reference is not None
        ):
            raise PermissionError(
                "non-privileged context cannot contain privileged references"
            )
        object.__setattr__(self, "oracle_path_xy", path)
        object.__setattr__(self, "kinodynamic_reference", reference)
        object.__setattr__(self, "dynamic_reference", dynamic_reference)


def _pointgoal(value: object) -> np.ndarray:
    try:
        pointgoal = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("pointgoal_base_xy must be a finite XY vector") from error
    if pointgoal.shape != (2,) or not np.all(np.isfinite(pointgoal)):
        raise ValueError("pointgoal_base_xy must be a finite XY vector")
    return pointgoal


def _repeated_control(
    protocol: ActionProtocol,
    *,
    main_velocity: float,
    auxiliary_angle: float,
) -> np.ndarray:
    component = np.asarray((main_velocity, auxiliary_angle), dtype=np.float32)
    block = np.tile(component, protocol.controls_per_block)
    return protocol.clip_block(block)


def geometric_tracker_action(
    protocol: ActionProtocol,
    target_base_xy: object,
    *,
    slowdown_distance_m: float = 0.6,
    stop_distance_m: float = 0.2,
) -> np.ndarray:
    """Convert one base-frame target into a repeated V2 action block."""

    target = _pointgoal(target_base_xy)
    distance = float(np.linalg.norm(target))
    if distance <= stop_distance_m:
        return np.zeros(protocol.action_block_dim, dtype=np.float32)
    heading_error = math.atan2(float(target[1]), float(target[0]))
    maximum_main = max(0.0, float(protocol.maximum[0]))
    speed_scale = min(1.0, distance / slowdown_distance_m)
    heading_scale = max(0.15, math.cos(heading_error))
    main_velocity = maximum_main * speed_scale * heading_scale
    # The simulator's auxiliary joint rotates the base opposite to the
    # base-frame lateral convention used by the policy (positive joint angle
    # produces a right turn).  Convert the geometric heading error into that
    # actuator convention before applying the protocol bounds.
    auxiliary_angle = float(
        np.clip(-heading_error, protocol.minimum[1], protocol.maximum[1])
    )
    return _repeated_control(
        protocol,
        main_velocity=main_velocity,
        auxiliary_angle=auxiliary_angle,
    )


class _PolicyBase:
    policy_name: ClassVar[str]
    privileged: ClassVar[bool] = False
    uses_shared_tracker: ClassVar[bool] = False

    def __init__(self, protocol: ActionProtocol) -> None:
        if not isinstance(protocol, ActionProtocol):
            raise TypeError("protocol must be an ActionProtocol")
        if (
            protocol.protocol_id != BENCHMARK_ACTION_PROTOCOL_ID
            or protocol.action_block_dim != 10
            or protocol.controls_per_block != 5
        ):
            raise ValueError("Benchmark V1 policies require ball_action_v2_aux30")
        self.protocol = protocol
        self._context: EpisodeContext | None = None

    @property
    def online_input_permissions(self) -> frozenset[str]:
        base = {
            "stereo_rgb",
            "proprio_history",
            "action_history",
            "perfect_position_yaw",
            "pointgoal_base_xy",
        }
        if self.privileged:
            base.add("oracle_path_xy")
            base.add("kinodynamic_reference")
        return frozenset(base)

    def _validate_context_protocol(self, context: EpisodeContext) -> None:
        if not isinstance(context, EpisodeContext):
            raise TypeError("context must be an EpisodeContext")
        if context.action_protocol_id != self.protocol.protocol_id:
            raise ValueError(
                f"episode uses {context.action_protocol_id}, but policy requires "
                f"{self.protocol.protocol_id}"
            )

    def _reset_non_privileged(self, context: EpisodeContext) -> None:
        self._validate_context_protocol(context)
        if (
            context.privileged
            or context.oracle_path_xy is not None
            or context.kinodynamic_reference is not None
            or context.dynamic_reference is not None
        ):
            raise PermissionError("non-privileged policy cannot receive privileged context")
        self._context = context

    def _validate_act_inputs(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        if self._context is None:
            raise RuntimeError("policy.reset() must be called before act()")
        if not isinstance(observation, PolicyObservation):
            raise TypeError("observation must be a PolicyObservation")
        return _pointgoal(pointgoal_base_xy)


class StopPolicy(_PolicyBase):
    policy_name = "stop"

    def reset(self, context: EpisodeContext) -> None:
        self._reset_non_privileged(context)

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        self._validate_act_inputs(observation, pointgoal_base_xy)
        return np.zeros(self.protocol.action_block_dim, dtype=np.float32)


class RandomPolicy(_PolicyBase):
    policy_name = "random"

    def __init__(self, protocol: ActionProtocol, *, seed: int) -> None:
        super().__init__(protocol)
        if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
            raise ValueError("seed must be a non-negative integer")
        self.seed = int(seed)
        self._generator: np.random.Generator | None = None

    def reset(self, context: EpisodeContext) -> None:
        self._reset_non_privileged(context)
        self._generator = np.random.default_rng(self.seed)

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        self._validate_act_inputs(observation, pointgoal_base_xy)
        if self._generator is None:
            raise RuntimeError("policy.reset() must be called before act()")
        return self.protocol.sample_block(self._generator)


class GoalGreedyPolicy(_PolicyBase):
    policy_name = "goal_greedy"
    uses_shared_tracker = True

    def reset(self, context: EpisodeContext) -> None:
        self._reset_non_privileged(context)

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        pointgoal = self._validate_act_inputs(observation, pointgoal_base_xy)
        return geometric_tracker_action(self.protocol, pointgoal)


class OraclePathPolicy(_PolicyBase):
    policy_name = "oracle"
    privileged = True
    uses_shared_tracker = True

    def __init__(
        self,
        protocol: ActionProtocol,
        *,
        lookahead_m: float = 0.5,
        yaw_rate_feedback_gain_s: float = DEFAULT_YAW_RATE_FEEDBACK_GAIN_S,
        drive_model: DriveModel | None = None,
    ) -> None:
        super().__init__(protocol)
        lookahead = float(lookahead_m)
        if not math.isfinite(lookahead) or lookahead <= 0.0:
            raise ValueError("lookahead_m must be finite and positive")
        self.lookahead_m = lookahead
        self.yaw_rate_feedback_gain_s = _validate_yaw_rate_feedback_gain_s(
            yaw_rate_feedback_gain_s
        )
        self.drive_model = _default_drive_model() if drive_model is None else drive_model
        if not isinstance(self.drive_model, DriveModel):
            raise TypeError("drive_model must be a DriveModel")
        self._oracle_path_xy: np.ndarray | None = None
        self._reference: tuple[KinodynamicWaypoint, ...] | None = None
        self._reference_xy: np.ndarray | None = None
        self._path_index = 0
        self._active_drive_direction = "forward"
        self._pending_drive_direction: str | None = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason: str | None = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = "forward"
        self._braking_state = "tracking"

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def diagnostics(self) -> dict[str, object]:
        return {
            "reference_index": self._path_index,
            "active_drive_direction": self._active_drive_direction,
            "pending_drive_direction": self._pending_drive_direction,
            "stopped_cycles": self._stopped_cycles,
            "off_reference_cycles": self._off_reference_cycles,
            "failure_subreason": self._failure_subreason,
            "cross_track_error_m": self._cross_track_error_m,
            "desired_drive_direction": self._desired_drive_direction,
            "braking_state": self._braking_state,
        }

    def reset(self, context: EpisodeContext) -> None:
        self._validate_context_protocol(context)
        if not context.privileged:
            raise PermissionError("OraclePathPolicy requires privileged context")
        if context.dynamic_reference is not None:
            raise ValueError("OraclePathPolicy cannot accept dynamic_reference")
        if context.oracle_path_xy is None:
            raise ValueError("privileged OraclePathPolicy requires oracle_path_xy")
        if context.kinodynamic_reference is None:
            raise ValueError(
                "privileged OraclePathPolicy requires kinodynamic_reference"
            )
        self._context = context
        self._oracle_path_xy = np.asarray(context.oracle_path_xy, dtype=np.float64).copy()
        self._reference = tuple(context.kinodynamic_reference)
        self._reference_xy = np.asarray(
            [(point.x_m, point.y_m) for point in self._reference], dtype=np.float64
        )
        self._path_index = 0
        self._active_drive_direction = self._reference[0].drive_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = self._active_drive_direction
        self._braking_state = "tracking"

    def _require_reference(
        self,
    ) -> tuple[tuple[KinodynamicWaypoint, ...], np.ndarray]:
        if self._reference is None or self._reference_xy is None:
            raise RuntimeError("policy.reset() must be called before act()")
        return self._reference, self._reference_xy

    def _project_to_reference(self, position_world_xy: np.ndarray) -> float:
        reference, reference_xy = self._require_reference()
        last_candidate = len(reference) - 1
        for index in range(self._path_index + 1, len(reference)):
            if reference[index].drive_direction != self._active_drive_direction:
                last_candidate = index
                break
        candidates = reference_xy[self._path_index : last_candidate + 1]
        distances = np.linalg.norm(candidates - position_world_xy, axis=1)
        nearest_offset = int(np.argmin(distances))
        self._path_index += nearest_offset
        return float(distances[nearest_offset])

    def _lookahead_target(self, lookahead_m: float) -> np.ndarray:
        reference, reference_xy = self._require_reference()
        direction = reference[self._path_index].drive_direction
        target_index = self._path_index
        accumulated = 0.0
        while target_index + 1 < len(reference):
            if reference[target_index + 1].drive_direction != direction:
                break
            segment = float(np.linalg.norm(reference_xy[target_index + 1] - reference_xy[target_index]))
            accumulated += segment
            target_index += 1
            if accumulated >= lookahead_m:
                break
        return reference_xy[target_index]

    def _remaining_path_length(self, position_world_xy: np.ndarray) -> float:
        _, reference_xy = self._require_reference()
        distance_to_path = float(
            np.linalg.norm(reference_xy[self._path_index] - position_world_xy)
        )
        if self._path_index + 1 >= reference_xy.shape[0]:
            return distance_to_path
        segments = np.linalg.norm(
            np.diff(reference_xy[self._path_index :], axis=0),
            axis=1,
        )
        return distance_to_path + float(segments.sum())

    def _zero_action(self) -> np.ndarray:
        return np.zeros(self.protocol.action_block_dim, dtype=np.float32)

    def _handle_direction_change(
        self, desired_direction: str, speed_m_s: float
    ) -> np.ndarray | None:
        if desired_direction == self._active_drive_direction:
            self._pending_drive_direction = None
            self._stopped_cycles = 0
            self._braking_state = "tracking"
            return None
        self._pending_drive_direction = desired_direction
        self._braking_state = "direction_change"
        if speed_m_s < 0.05:
            self._stopped_cycles += 1
        else:
            self._stopped_cycles = 0
        if self._stopped_cycles < 5:
            return self._zero_action()
        self._active_drive_direction = desired_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._braking_state = "tracking"
        return None

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        pointgoal = self._validate_act_inputs(observation, pointgoal_base_xy)
        reference, _ = self._require_reference()
        current_speed = float(np.linalg.norm(observation.proprio_history[-1, :2]))
        current_yaw_rate = float(observation.proprio_history[-1, 5])
        cross_track_error = self._project_to_reference(observation.position_world_xy)
        self._cross_track_error_m = cross_track_error
        desired_direction = reference[self._path_index].drive_direction
        self._desired_drive_direction = desired_direction
        if cross_track_error > 1.0:
            self._off_reference_cycles += 1
        else:
            self._off_reference_cycles = 0
        if self._off_reference_cycles >= 10:
            self._failure_subreason = "tracking_infeasible"
        if self._failure_subreason is not None:
            self._braking_state = "tracking_infeasible"
            return self._zero_action()

        if float(np.linalg.norm(pointgoal)) <= 0.2:
            self._braking_state = "goal_reached"
            return self._zero_action()
        direction_action = self._handle_direction_change(
            desired_direction, current_speed
        )
        if direction_action is not None:
            return direction_action

        lookahead = max(
            self.lookahead_m,
            adaptive_lookahead_distance(current_speed),
        )
        target_world = self._lookahead_target(lookahead)
        world_delta = target_world - observation.position_world_xy
        cosine = math.cos(observation.yaw_world_rad)
        sine = math.sin(observation.yaw_world_rad)
        target_base = np.asarray(
            (
                cosine * world_delta[0] + sine * world_delta[1],
                -sine * world_delta[0] + cosine * world_delta[1],
            ),
            dtype=np.float64,
        )
        distance_squared = float(np.dot(target_base, target_base))
        desired_curvature = (
            0.0
            if distance_squared <= 1e-9
            else 2.0 * float(target_base[1]) / distance_squared
        )
        planned_main = float(reference[self._path_index].main_command_rad_s)
        if planned_main == 0.0:
            return self._zero_action()
        sign = 1.0 if desired_direction == "forward" else -1.0
        main_magnitude = abs(planned_main)
        if abs(desired_curvature) >= 0.55:
            main_magnitude = min(main_magnitude, 0.5)
        elif abs(desired_curvature) >= 0.25:
            main_magnitude = min(main_magnitude, 1.0)
        main_command = sign * main_magnitude

        remaining = self._remaining_path_length(observation.position_world_xy)
        if remaining <= self.drive_model.stopping_distance(
            current_speed, desired_direction
        ) + 0.1:
            self._braking_state = "goal_braking"
            return self._zero_action()

        auxiliary = self.drive_model.auxiliary_for_curvature(
            main_command, desired_curvature
        )
        desired_yaw_rate = desired_curvature * sign * max(current_speed, 0.2)
        correction = -sign * self.yaw_rate_feedback_gain_s * (
            desired_yaw_rate - current_yaw_rate
        )
        correction = float(np.clip(correction, math.radians(-5.0), math.radians(5.0)))
        self._braking_state = "tracking"
        return _repeated_control(
            self.protocol,
            main_velocity=main_command,
            auxiliary_angle=auxiliary + correction,
        )


class TrajectoryOraclePolicy(_PolicyBase):
    """Track a spatial kinodynamic reference with curvature feedforward.

    This policy intentionally owns its tracker state.  ``OraclePathPolicy`` is
    a frozen compatibility baseline, so sharing its mutable state or changing
    its implementation would make the baseline sensitive to this V1 policy.
    ``SpatialReferenceProfile`` stores geometric curvature with respect to the
    waypoint-order arc length.  Reverse motion therefore negates that value
    before asking the signed DriveModel for an auxiliary angle: the dynamics
    profile's curvature is defined with respect to the signed body command.
    """

    policy_name = "oracle_trajectory_v1"
    privileged = True
    uses_shared_tracker = True

    def __init__(
        self,
        protocol: ActionProtocol,
        *,
        lookahead_m: float = 1.0,
        smoothing_window_m: float = 0.40,
        curvature_preview_m: float = 0.80,
        lateral_curvature_gain_1_m2: float = 0.20,
        heading_curvature_gain_1_m: float = 0.15,
        max_feedback_curvature_1_m: float = 0.20,
        yaw_rate_feedback_gain_s: float = 0.15,
        latency_preview_s: float = 0.0,
    ) -> None:
        super().__init__(protocol)
        self.lookahead_m = _validate_trajectory_scalar(
            lookahead_m,
            name="lookahead_m",
            strictly_positive=True,
        )
        self.smoothing_window_m = _validate_trajectory_scalar(
            smoothing_window_m,
            name="smoothing_window_m",
        )
        self.curvature_preview_m = _validate_trajectory_scalar(
            curvature_preview_m,
            name="curvature_preview_m",
        )
        self.lateral_curvature_gain_1_m2 = _validate_trajectory_scalar(
            lateral_curvature_gain_1_m2,
            name="lateral_curvature_gain_1_m2",
        )
        self.heading_curvature_gain_1_m = _validate_trajectory_scalar(
            heading_curvature_gain_1_m,
            name="heading_curvature_gain_1_m",
        )
        self.max_feedback_curvature_1_m = _validate_trajectory_scalar(
            max_feedback_curvature_1_m,
            name="max_feedback_curvature_1_m",
        )
        self.yaw_rate_feedback_gain_s = _validate_trajectory_scalar(
            yaw_rate_feedback_gain_s,
            name="yaw_rate_feedback_gain_s",
        )
        latency = _validate_trajectory_scalar(
            latency_preview_s,
            name="latency_preview_s",
        )
        if latency != 0.0:
            raise ValueError("latency_preview_s must be exactly 0.0 in V1")
        self.latency_preview_s = latency
        self.drive_model = _default_drive_model()

        self._reference: tuple[KinodynamicWaypoint, ...] | None = None
        self._profile: SpatialReferenceProfile | None = None
        self._path_index = 0
        self._active_drive_direction = "forward"
        self._pending_drive_direction: str | None = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason: str | None = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = "forward"
        self._braking_state = "tracking"
        self._reference_curvature_1_m = 0.0
        self._feedback_curvature_1_m = 0.0
        self._command_curvature_1_m = 0.0
        self._lateral_error_m = 0.0
        self._heading_error_rad = 0.0
        self._main_cap_rad_s = 0.0
        self._auxiliary_feedforward_rad = 0.0
        self._auxiliary_feedback_rad = 0.0

    @property
    def online_input_permissions(self) -> frozenset[str]:
        # The path remains in EpisodeContext for the frozen runner contract,
        # but this policy consumes only the kinodynamic reference.
        return frozenset(
            permission
            for permission in super().online_input_permissions
            if permission != "oracle_path_xy"
        )

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def diagnostics(self) -> dict[str, object]:
        return {
            "reference_index": self._path_index,
            "active_drive_direction": self._active_drive_direction,
            "pending_drive_direction": self._pending_drive_direction,
            "stopped_cycles": self._stopped_cycles,
            "off_reference_cycles": self._off_reference_cycles,
            "failure_subreason": self._failure_subreason,
            "cross_track_error_m": self._cross_track_error_m,
            "desired_drive_direction": self._desired_drive_direction,
            "braking_state": self._braking_state,
            "reference_curvature_1_m": self._reference_curvature_1_m,
            "feedback_curvature_1_m": self._feedback_curvature_1_m,
            "command_curvature_1_m": self._command_curvature_1_m,
            "lateral_error_m": self._lateral_error_m,
            "heading_error_rad": self._heading_error_rad,
            "main_cap_rad_s": self._main_cap_rad_s,
            "auxiliary_feedforward_rad": self._auxiliary_feedforward_rad,
            "auxiliary_feedback_rad": self._auxiliary_feedback_rad,
        }

    def reset(self, context: EpisodeContext) -> None:
        self._validate_context_protocol(context)
        if not context.privileged:
            raise PermissionError("TrajectoryOraclePolicy requires privileged context")
        if context.dynamic_reference is not None:
            raise ValueError("TrajectoryOraclePolicy cannot accept dynamic_reference")
        if context.kinodynamic_reference is None:
            raise ValueError(
                "privileged TrajectoryOraclePolicy requires kinodynamic_reference"
            )
        self._context = context
        self._reference = tuple(context.kinodynamic_reference)
        self._profile = SpatialReferenceProfile.from_waypoints(
            self._reference,
            smoothing_window_m=self.smoothing_window_m,
        )
        self._path_index = 0
        self._active_drive_direction = self._reference[0].drive_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = self._active_drive_direction
        self._braking_state = "tracking"
        self._reference_curvature_1_m = 0.0
        self._feedback_curvature_1_m = 0.0
        self._command_curvature_1_m = 0.0
        self._lateral_error_m = 0.0
        self._heading_error_rad = 0.0
        self._main_cap_rad_s = 0.0
        self._auxiliary_feedforward_rad = 0.0
        self._auxiliary_feedback_rad = 0.0

    def _require_reference(
        self,
    ) -> tuple[tuple[KinodynamicWaypoint, ...], SpatialReferenceProfile]:
        if self._reference is None or self._profile is None:
            raise RuntimeError("policy.reset() must be called before act()")
        return self._reference, self._profile

    def _project_to_reference(self, position_world_xy: np.ndarray) -> float:
        reference, profile = self._require_reference()
        last_candidate = len(reference) - 1
        for index in range(self._path_index + 1, len(reference)):
            if reference[index].drive_direction != self._active_drive_direction:
                last_candidate = index
                break
        candidates = profile.xy[self._path_index : last_candidate + 1]
        distances = np.linalg.norm(candidates - position_world_xy, axis=1)
        nearest_offset = int(np.argmin(distances))
        self._path_index += nearest_offset
        return float(distances[nearest_offset])

    def _lookahead_index(self, lookahead_m: float) -> int:
        reference, profile = self._require_reference()
        direction = reference[self._path_index].drive_direction
        target_index = self._path_index
        accumulated = 0.0
        while target_index + 1 < len(reference):
            if reference[target_index + 1].drive_direction != direction:
                break
            segment = float(
                np.linalg.norm(profile.xy[target_index + 1] - profile.xy[target_index])
            )
            accumulated += segment
            target_index += 1
            if accumulated >= lookahead_m:
                break
        return target_index

    def _remaining_path_length(self, position_world_xy: np.ndarray) -> float:
        _, profile = self._require_reference()
        distance_to_path = float(
            np.linalg.norm(profile.xy[self._path_index] - position_world_xy)
        )
        if self._path_index + 1 >= profile.xy.shape[0]:
            return distance_to_path
        segments = np.linalg.norm(
            np.diff(profile.xy[self._path_index :], axis=0),
            axis=1,
        )
        return distance_to_path + float(segments.sum())

    def _zero_action(self) -> np.ndarray:
        return np.zeros(self.protocol.action_block_dim, dtype=np.float32)

    def _reset_command_diagnostics(self) -> None:
        self._reference_curvature_1_m = 0.0
        self._feedback_curvature_1_m = 0.0
        self._command_curvature_1_m = 0.0
        self._lateral_error_m = 0.0
        self._heading_error_rad = 0.0
        self._main_cap_rad_s = 0.0
        self._auxiliary_feedforward_rad = 0.0
        self._auxiliary_feedback_rad = 0.0

    def _handle_direction_change(
        self, desired_direction: str, speed_m_s: float
    ) -> np.ndarray | None:
        if desired_direction == self._active_drive_direction:
            self._pending_drive_direction = None
            self._stopped_cycles = 0
            self._braking_state = "tracking"
            return None
        self._pending_drive_direction = desired_direction
        self._braking_state = "direction_change"
        if speed_m_s < 0.05:
            self._stopped_cycles += 1
        else:
            self._stopped_cycles = 0
        if self._stopped_cycles < 5:
            self._reset_command_diagnostics()
            return self._zero_action()
        self._active_drive_direction = desired_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._braking_state = "tracking"
        return None

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        pointgoal = self._validate_act_inputs(observation, pointgoal_base_xy)
        reference, profile = self._require_reference()
        current_speed = float(np.linalg.norm(observation.proprio_history[-1, :2]))
        current_yaw_rate = float(observation.proprio_history[-1, 5])
        cross_track_error = self._project_to_reference(observation.position_world_xy)
        self._cross_track_error_m = cross_track_error
        desired_direction = reference[self._path_index].drive_direction
        self._desired_drive_direction = desired_direction
        if cross_track_error > 1.0:
            self._off_reference_cycles += 1
        else:
            self._off_reference_cycles = 0
        if self._off_reference_cycles >= 10:
            self._failure_subreason = "tracking_infeasible"
        if self._failure_subreason is not None:
            self._braking_state = "tracking_infeasible"
            self._reset_command_diagnostics()
            return self._zero_action()

        if float(np.linalg.norm(pointgoal)) <= 0.2:
            self._braking_state = "goal_reached"
            self._reset_command_diagnostics()
            return self._zero_action()
        direction_action = self._handle_direction_change(
            desired_direction, current_speed
        )
        if direction_action is not None:
            return direction_action

        reference_yaw = float(profile.yaw_unwrapped[self._path_index])
        delta = observation.position_world_xy - profile.xy[self._path_index]
        motion_tangent = reference_yaw + (
            math.pi if desired_direction == "reverse" else 0.0
        )
        lateral_error = float(
            -math.sin(motion_tangent) * delta[0]
            + math.cos(motion_tangent) * delta[1]
        )
        heading_error = self._wrap_angle(
            reference_yaw - observation.yaw_world_rad
        )
        lookahead = max(
            1.0,
            self.lookahead_m,
            adaptive_lookahead_distance(current_speed),
        )
        lookahead_index = self._lookahead_index(lookahead)
        reference_curvature = float(profile.curvature_1_m[lookahead_index])
        feedback_curvature = float(
            np.clip(
                -self.lateral_curvature_gain_1_m2 * lateral_error
                + self.heading_curvature_gain_1_m * math.sin(heading_error),
                -self.max_feedback_curvature_1_m,
                self.max_feedback_curvature_1_m,
            )
        )
        command_curvature = reference_curvature + feedback_curvature
        self._reference_curvature_1_m = reference_curvature
        self._feedback_curvature_1_m = feedback_curvature
        self._command_curvature_1_m = command_curvature
        self._lateral_error_m = lateral_error
        self._heading_error_rad = heading_error

        preview_curvature = profile.preview_max_abs_curvature(
            self._path_index, self.curvature_preview_m
        )
        if preview_curvature < 0.20:
            main_cap = 0.50
        elif preview_curvature < 0.45:
            main_cap = 0.35
        else:
            main_cap = 0.25
        if abs(lateral_error) >= 0.15 or abs(heading_error) >= math.radians(20.0):
            main_cap = min(main_cap, 0.25)
        self._main_cap_rad_s = main_cap

        planned_main = float(reference[self._path_index].main_command_rad_s)
        remaining = self._remaining_path_length(observation.position_world_xy)
        if remaining <= self.drive_model.stopping_distance(
            current_speed, desired_direction
        ) + 0.1:
            self._braking_state = "goal_braking"
            self._reset_command_diagnostics()
            return self._zero_action()
        if planned_main == 0.0:
            self._braking_state = "goal_braking"
            self._reset_command_diagnostics()
            return self._zero_action()

        sign = 1.0 if desired_direction == "forward" else -1.0
        main_magnitude = min(abs(planned_main), main_cap)
        main_command = sign * main_magnitude
        # Spatial profile curvature is d(yaw)/d(waypoint-order arc length).
        # DriveModel uses the signed body command convention, which is the
        # opposite convention for a reverse temporal segment.
        drive_reference_curvature = sign * reference_curvature
        drive_command_curvature = sign * command_curvature
        auxiliary_feedforward = self.drive_model.auxiliary_for_curvature(
            main_command, drive_reference_curvature
        )
        auxiliary_command = self.drive_model.auxiliary_for_curvature(
            main_command, drive_command_curvature
        )
        desired_yaw_rate = (
            drive_command_curvature * sign * max(current_speed, 0.2)
        )
        correction = -sign * self.yaw_rate_feedback_gain_s * (
            desired_yaw_rate - current_yaw_rate
        )
        correction = float(
            np.clip(correction, math.radians(-5.0), math.radians(5.0))
        )
        final_auxiliary = auxiliary_command + correction
        self._auxiliary_feedforward_rad = float(auxiliary_feedforward)
        self._auxiliary_feedback_rad = float(
            final_auxiliary - auxiliary_feedforward
        )
        self._braking_state = "tracking"
        return _repeated_control(
            self.protocol,
            main_velocity=main_command,
            auxiliary_angle=final_auxiliary,
        )


class DelayAwareTrajectoryOraclePolicy(_PolicyBase):
    """Track a spatial reference with a bounded, delay-aware profile lead.

    This is deliberately a separate tracker implementation.  The two
    earlier oracle trackers are compatibility baselines and their mutable
    path/cusp state must not be shared with this experiment.  The profile
    curvature is defined in waypoint-order arc length; for reverse motion it
    is mapped through the signed body command before querying ``DriveModel``.
    """

    policy_name = "oracle_trajectory_v2"
    privileged = True
    uses_shared_tracker = True

    _ACTION_PERIOD_S = 0.1
    _GOAL_TOLERANCE_M = 0.2
    _STOP_SPEED_M_S = 0.05
    _STOP_HOLD_CYCLES = 5
    _CALIBRATED_MAIN_COMMAND_RAD_S = 0.50
    _MAX_DELAY_DISTANCE_M = 0.30
    _LATERAL_CURVATURE_GAIN_1_M2 = 0.20
    _HEADING_CURVATURE_GAIN_1_M = 0.15
    _YAW_RATE_FEEDBACK_GAIN_S = 0.15
    _TERMINAL_BRAKE_BUFFER_M = 0.10

    def __init__(
        self,
        protocol: ActionProtocol,
        *,
        auxiliary_latency_s: float = 1.0,
        maximum_delay_distance_m: float = 0.30,
        profile_window_m: float = 0.30,
        profile_blend_start_1_m: float = 0.05,
        profile_blend_full_1_m: float = 0.20,
        feedback_lookahead_m: float = 1.0,
        maximum_feedback_curvature_1_m: float = 0.20,
        supported_main_command_rad_s: float = 0.50,
        episode_budget_s: float = 35.0,
        terminal_brake_buffer_m: float = _TERMINAL_BRAKE_BUFFER_M,
        drive_model: DriveModel | None = None,
    ) -> None:
        super().__init__(protocol)
        self.auxiliary_latency_s = _validate_trajectory_scalar(
            auxiliary_latency_s,
            name="auxiliary_latency_s",
        )
        self.maximum_delay_distance_m = _validate_trajectory_scalar(
            maximum_delay_distance_m,
            name="maximum_delay_distance_m",
        )
        if self.maximum_delay_distance_m > self._MAX_DELAY_DISTANCE_M:
            raise ValueError("maximum_delay_distance_m must not exceed 0.30 m")
        self.profile_window_m = _validate_trajectory_scalar(
            profile_window_m,
            name="profile_window_m",
        )
        self.profile_blend_start_1_m = _validate_trajectory_scalar(
            profile_blend_start_1_m,
            name="profile_blend_start_1_m",
        )
        self.profile_blend_full_1_m = _validate_trajectory_scalar(
            profile_blend_full_1_m,
            name="profile_blend_full_1_m",
        )
        if self.profile_blend_full_1_m <= self.profile_blend_start_1_m:
            raise ValueError(
                "profile_blend_full_1_m must exceed profile_blend_start_1_m"
            )
        self.feedback_lookahead_m = _validate_trajectory_scalar(
            feedback_lookahead_m,
            name="feedback_lookahead_m",
            strictly_positive=True,
        )
        self.maximum_feedback_curvature_1_m = _validate_trajectory_scalar(
            maximum_feedback_curvature_1_m,
            name="maximum_feedback_curvature_1_m",
        )
        # These gains are intentionally frozen with the V1 Pure Pursuit
        # stabilizer; only the delay/profile parameters above are V2's
        # preregistered experiment knobs.
        self.lateral_curvature_gain_1_m2 = self._LATERAL_CURVATURE_GAIN_1_M2
        self.heading_curvature_gain_1_m = self._HEADING_CURVATURE_GAIN_1_M
        self.yaw_rate_feedback_gain_s = self._YAW_RATE_FEEDBACK_GAIN_S
        self.supported_main_command_rad_s = _validate_trajectory_scalar(
            supported_main_command_rad_s,
            name="supported_main_command_rad_s",
            strictly_positive=True,
        )
        if self.supported_main_command_rad_s != self._CALIBRATED_MAIN_COMMAND_RAD_S:
            raise ValueError(
                "supported_main_command_rad_s must be the calibrated 0.50 rad/s operating point"
            )
        self.episode_budget_s = _validate_trajectory_scalar(
            episode_budget_s,
            name="episode_budget_s",
            strictly_positive=True,
        )
        self.terminal_brake_buffer_m = _validate_trajectory_scalar(
            terminal_brake_buffer_m,
            name="terminal_brake_buffer_m",
        )
        self.drive_model = _default_drive_model() if drive_model is None else drive_model
        if not isinstance(self.drive_model, DriveModel):
            raise TypeError("drive_model must be a DriveModel")

        self._reference: tuple[KinodynamicWaypoint, ...] | None = None
        self._profile: SpatialReferenceProfile | None = None
        self._path_index = 0
        self._active_drive_direction = "forward"
        self._pending_drive_direction: str | None = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason: str | None = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = "forward"
        self._braking_state = "tracking"

        # Curvature diagnostics are intentionally separate, so a test can
        # distinguish the frozen Pure Pursuit term from the local profile and
        # bounded feedback terms.
        self._projection_s_m = 0.0
        self._last_projection_s_m = 0.0
        self._delay_distance_m = 0.0
        self._query_s_m = 0.0
        self._pure_pursuit_curvature_1_m = 0.0
        self._profile_curvature_1_m = 0.0
        self._feedback_curvature_1_m = 0.0
        self._blend_weight = 0.0
        self._command_curvature_1_m = 0.0
        self._lateral_error_m = 0.0
        self._heading_error_rad = 0.0
        self._main_cap_rad_s = 0.0
        self._auxiliary_feedforward_rad = 0.0
        self._auxiliary_feedback_rad = 0.0

        # The policy has no simulator clock in its interface.  These values
        # are action-period timing markers for diagnostics only; they never
        # alter stop, goal, or episode-budget decisions.
        self._action_index = 0
        self._elapsed_s = 0.0
        self._terminal_brake_start_s = -1.0
        self._goal_entry_s = -1.0
        self._stop_below_0p05_s = -1.0
        self._hold_complete_s = -1.0
        self._terminal_hold_cycles = 0
        self._terminal_runway_s = self.episode_budget_s
        self._budget_infeasible = False

    @property
    def online_input_permissions(self) -> frozenset[str]:
        # ``oracle_path_xy`` stays in the episode context for the frozen
        # runner contract, but this policy consumes only kinodynamic data.
        return frozenset(
            permission
            for permission in super().online_input_permissions
            if permission != "oracle_path_xy"
        )

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def diagnostics(self) -> dict[str, object]:
        terminal_timing = {
            "terminal_brake_start_s": float(self._terminal_brake_start_s),
            "goal_entry_s": float(self._goal_entry_s),
            "stop_below_0p05_s": float(self._stop_below_0p05_s),
            "hold_complete_s": float(self._hold_complete_s),
        }
        return {
            "reference_index": self._path_index,
            "active_drive_direction": self._active_drive_direction,
            "pending_drive_direction": self._pending_drive_direction,
            "stopped_cycles": self._stopped_cycles,
            "off_reference_cycles": self._off_reference_cycles,
            "failure_subreason": self._failure_subreason,
            "cross_track_error_m": self._cross_track_error_m,
            "desired_drive_direction": self._desired_drive_direction,
            "braking_state": self._braking_state,
            "projection_s_m": float(self._projection_s_m),
            "delay_distance_m": float(self._delay_distance_m),
            "query_s_m": float(self._query_s_m),
            "pure_pursuit_curvature_1_m": float(
                self._pure_pursuit_curvature_1_m
            ),
            "profile_curvature_1_m": float(self._profile_curvature_1_m),
            "feedback_curvature_1_m": float(self._feedback_curvature_1_m),
            "blend_weight": float(self._blend_weight),
            "command_curvature_1_m": float(self._command_curvature_1_m),
            # Keep the established trajectory diagnostic names available to
            # CPU consumers while exposing the V2-specific names above.
            "reference_curvature_1_m": float(self._profile_curvature_1_m),
            "lateral_error_m": float(self._lateral_error_m),
            "heading_error_rad": float(self._heading_error_rad),
            "main_cap_rad_s": float(self._main_cap_rad_s),
            "auxiliary_feedforward_rad": float(self._auxiliary_feedforward_rad),
            "auxiliary_feedback_rad": float(self._auxiliary_feedback_rad),
            "elapsed_s": float(self._elapsed_s),
            "terminal_runway_s": float(self._terminal_runway_s),
            "budget_infeasible": bool(self._budget_infeasible),
            "terminal_brake_start_s": terminal_timing["terminal_brake_start_s"],
            "goal_entry_s": terminal_timing["goal_entry_s"],
            "stop_below_0p05_s": terminal_timing["stop_below_0p05_s"],
            "hold_complete_s": terminal_timing["hold_complete_s"],
            # Flat aliases keep timing diagnostics easy to serialize for
            # callers that do not support nested event payloads.
            "terminal_brake_start_time_s": terminal_timing[
                "terminal_brake_start_s"
            ],
            "terminal_goal_entry_s": terminal_timing["goal_entry_s"],
            "goal_entry_time_s": terminal_timing["goal_entry_s"],
            "terminal_stop_s": terminal_timing["stop_below_0p05_s"],
            "stop_below_speed_s": terminal_timing["stop_below_0p05_s"],
            "stop_below_speed_time_s": terminal_timing["stop_below_0p05_s"],
            "terminal_hold_complete_s": terminal_timing["hold_complete_s"],
            "hold_complete_time_s": terminal_timing["hold_complete_s"],
        }

    def reset(self, context: EpisodeContext) -> None:
        self._validate_context_protocol(context)
        if not context.privileged:
            raise PermissionError(
                "DelayAwareTrajectoryOraclePolicy requires privileged context"
            )
        if context.dynamic_reference is not None:
            raise ValueError(
                "DelayAwareTrajectoryOraclePolicy cannot accept dynamic_reference"
            )
        if context.kinodynamic_reference is None:
            raise ValueError(
                "privileged DelayAwareTrajectoryOraclePolicy requires kinodynamic_reference"
            )

        # Construct the immutable profile before mutating tracker state.  A
        # malformed reference therefore fails during reset, never half-way
        # through an action cycle.
        reference = tuple(context.kinodynamic_reference)
        profile = SpatialReferenceProfile.from_waypoints(
            reference,
            smoothing_window_m=self.profile_window_m,
        )

        self._context = context
        self._reference = reference
        self._profile = profile
        self._path_index = 0
        self._last_projection_s_m = float(profile.s_m[0])
        self._active_drive_direction = reference[0].drive_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._off_reference_cycles = 0
        self._failure_subreason = None
        self._cross_track_error_m = 0.0
        self._desired_drive_direction = self._active_drive_direction
        self._braking_state = "tracking"
        self._reset_command_diagnostics()
        self._action_index = 0
        self._elapsed_s = 0.0
        self._terminal_brake_start_s = -1.0
        self._goal_entry_s = -1.0
        self._stop_below_0p05_s = -1.0
        self._hold_complete_s = -1.0
        self._terminal_hold_cycles = 0
        self._terminal_runway_s = self.episode_budget_s
        self._budget_infeasible = False

    def _require_reference(
        self,
    ) -> tuple[tuple[KinodynamicWaypoint, ...], SpatialReferenceProfile]:
        if self._reference is None or self._profile is None:
            raise RuntimeError("policy.reset() must be called before act()")
        return self._reference, self._profile

    def _project_to_reference(self, position_world_xy: np.ndarray) -> float:
        reference, profile = self._require_reference()
        if self._path_index < 0 or self._path_index >= len(reference):
            raise RuntimeError("V2 tracker path index is outside reference")
        if reference[self._path_index].drive_direction != self._active_drive_direction:
            raise RuntimeError("V2 tracker direction state is inconsistent")

        segment = int(profile.segment_id[self._path_index])
        segment_indices = np.flatnonzero(profile.segment_id == segment)
        segment_start = int(segment_indices[0])
        segment_end = int(segment_indices[-1])
        start = max(self._path_index, segment_start)

        # Keep the projection causal.  ``path_index`` is a waypoint-facing
        # state, while ``_last_projection_s_m`` carries the continuous
        # position between sparse waypoints.  Both are monotonic within a
        # direction run, including when a measured pose briefly moves back.
        previous_s = float(
            np.clip(
                self._last_projection_s_m,
                float(profile.s_m[start]),
                float(profile.s_m[segment_end]),
            )
        )

        # A segment is valid only when both endpoints belong to the active
        # direction.  This deliberately excludes the first waypoint of the
        # next direction run and the transition edge leading to it.
        candidates: list[tuple[float, float, int, float, np.ndarray]] = []
        for index in range(start, segment_end):
            if (
                reference[index].drive_direction != self._active_drive_direction
                or reference[index + 1].drive_direction != self._active_drive_direction
            ):
                continue
            first = profile.xy[index]
            second = profile.xy[index + 1]
            delta = second - first
            length_squared = float(np.dot(delta, delta))
            if length_squared <= 1e-24:
                continue
            length = math.sqrt(length_squared)
            fraction = float(
                np.clip(
                    np.dot(position_world_xy - first, delta) / length_squared,
                    0.0,
                    1.0,
                )
            )
            projected = first + fraction * delta
            distance = float(np.linalg.norm(position_world_xy - projected))
            projected_s = float(profile.s_m[index] + fraction * length)
            candidates.append(
                (distance, projected_s, index, fraction, projected)
            )

        if candidates:
            best_distance, projected_s, projected_segment, projected_fraction, projected = min(
                candidates,
                key=lambda item: (item[0], item[1]),
            )
            if projected_s < previous_s - 1e-12:
                # Recompute the nearest point at the monotonic arc-length
                # clamp, rather than reporting the unconstrained backward
                # projection's cross-track error.
                projected_s = previous_s
                clamped = False
                for index in range(start, segment_end):
                    if (
                        reference[index].drive_direction
                        != self._active_drive_direction
                        or reference[index + 1].drive_direction
                        != self._active_drive_direction
                    ):
                        continue
                    first = profile.xy[index]
                    second = profile.xy[index + 1]
                    delta = second - first
                    length = float(np.linalg.norm(delta))
                    if length <= 1e-12:
                        continue
                    if projected_s <= float(profile.s_m[index + 1]) + 1e-12:
                        projected_segment = index
                        projected_fraction = float(
                            np.clip(
                                (projected_s - float(profile.s_m[index])) / length,
                                0.0,
                                1.0,
                            )
                        )
                        projected = first + projected_fraction * delta
                        best_distance = float(
                            np.linalg.norm(position_world_xy - projected)
                        )
                        clamped = True
                        break
                if not clamped:
                    # A singleton run or terminal duplicate may have no
                    # positive-length support.  Use its final valid marker.
                    projected_segment = segment_end
                    projected_fraction = 0.0
                    projected = profile.xy[segment_end]
                    best_distance = float(
                        np.linalg.norm(position_world_xy - projected)
                    )
        else:
            # A singleton direction run has no positive-length segment.  Its
            # marker remains the only valid projection point.
            projected_segment = start
            projected_fraction = 0.0
            projected = profile.xy[start]
            projected_s = float(profile.s_m[start])
            best_distance = float(np.linalg.norm(position_world_xy - projected))

        # ``path_index`` names the first waypoint of the active projection
        # segment.  It advances only when the continuous projection reaches
        # that segment's endpoint and never enters the next direction run.
        new_index = max(self._path_index, projected_segment)
        if projected_fraction >= 1.0 - 1e-12:
            new_index = max(new_index, projected_segment + 1)
        self._path_index = min(new_index, segment_end)
        self._last_projection_s_m = float(max(previous_s, projected_s))
        self._projection_s_m = self._last_projection_s_m
        self._query_s_m = self._projection_s_m
        return float(best_distance)

    def _next_segment_start(self, index: int) -> int | None:
        reference, _ = self._require_reference()
        direction = reference[index].drive_direction
        for candidate in range(index + 1, len(reference)):
            if reference[candidate].drive_direction != direction:
                return candidate
        return None

    def _desired_direction_for_current_index(self) -> str:
        reference, profile = self._require_reference()
        current_direction = reference[self._path_index].drive_direction
        if current_direction != self._active_drive_direction:
            return current_direction
        segment = int(profile.segment_id[self._path_index])
        segment_indices = np.flatnonzero(profile.segment_id == segment)
        segment_end = int(segment_indices[-1])
        next_index = self._next_segment_start(segment_end)
        if self._path_index == segment_end and next_index is not None:
            return reference[next_index].drive_direction
        return current_direction

    def _lookahead_index(self, lookahead_m: float) -> int:
        reference, profile = self._require_reference()
        direction = reference[self._path_index].drive_direction
        target_index = self._path_index
        accumulated = 0.0
        while target_index + 1 < len(reference):
            if reference[target_index + 1].drive_direction != direction:
                break
            segment = float(
                np.linalg.norm(profile.xy[target_index + 1] - profile.xy[target_index])
            )
            accumulated += segment
            target_index += 1
            if accumulated >= lookahead_m:
                break
        return target_index

    def _remaining_path_length(self, position_world_xy: np.ndarray) -> float:
        _, profile = self._require_reference()
        distance_to_path = float(
            np.linalg.norm(profile.xy[self._path_index] - position_world_xy)
        )
        if self._path_index + 1 >= profile.xy.shape[0]:
            return distance_to_path
        segments = np.linalg.norm(
            np.diff(profile.xy[self._path_index :], axis=0),
            axis=1,
        )
        return distance_to_path + float(segments.sum())

    def _zero_action(self) -> np.ndarray:
        return np.zeros(self.protocol.action_block_dim, dtype=np.float32)

    def _reset_command_diagnostics(self) -> None:
        profile = self._profile
        if profile is not None and 0 <= self._path_index < profile.s_m.shape[0]:
            projection = float(profile.s_m[self._path_index])
        else:
            projection = 0.0
        self._projection_s_m = projection
        self._delay_distance_m = 0.0
        self._query_s_m = projection
        self._pure_pursuit_curvature_1_m = 0.0
        self._profile_curvature_1_m = 0.0
        self._feedback_curvature_1_m = 0.0
        self._blend_weight = 0.0
        self._command_curvature_1_m = 0.0
        self._lateral_error_m = 0.0
        self._heading_error_rad = 0.0
        self._main_cap_rad_s = 0.0
        self._auxiliary_feedforward_rad = 0.0
        self._auxiliary_feedback_rad = 0.0

    def _handle_direction_change(
        self, desired_direction: str, speed_m_s: float
    ) -> np.ndarray | None:
        if desired_direction == self._active_drive_direction:
            self._pending_drive_direction = None
            self._stopped_cycles = 0
            self._braking_state = "tracking"
            return None
        self._pending_drive_direction = desired_direction
        self._braking_state = "direction_change"
        if speed_m_s < self._STOP_SPEED_M_S:
            self._stopped_cycles += 1
        else:
            self._stopped_cycles = 0
        if self._stopped_cycles < self._STOP_HOLD_CYCLES:
            self._reset_command_diagnostics()
            return self._zero_action()
        self._active_drive_direction = desired_direction
        next_index = self._next_segment_start(self._path_index)
        if next_index is not None:
            reference, profile = self._require_reference()
            if reference[next_index].drive_direction == desired_direction:
                self._path_index = next_index
                self._last_projection_s_m = float(profile.s_m[next_index])
                self._projection_s_m = self._last_projection_s_m
                self._query_s_m = self._projection_s_m
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._braking_state = "tracking"
        return None

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def _record_terminal_brake(self) -> None:
        if self._terminal_brake_start_s < 0.0:
            self._terminal_brake_start_s = self._elapsed_s

    def _record_goal_entry(self, speed_m_s: float) -> None:
        if self._goal_entry_s < 0.0:
            self._goal_entry_s = self._elapsed_s
        if speed_m_s < self._STOP_SPEED_M_S:
            if self._stop_below_0p05_s < 0.0:
                self._stop_below_0p05_s = self._elapsed_s
            self._terminal_hold_cycles = min(
                self._STOP_HOLD_CYCLES,
                self._terminal_hold_cycles + 1,
            )
            if (
                self._terminal_hold_cycles >= self._STOP_HOLD_CYCLES
                and self._hold_complete_s < 0.0
            ):
                self._hold_complete_s = self._elapsed_s

    def _update_budget_diagnostics(
        self,
        remaining_path_m: float,
        current_speed_m_s: float,
        desired_direction: str,
        command_curvature_1_m: float = 0.0,
    ) -> None:
        """Estimate runway for diagnostics without changing the command."""

        if remaining_path_m < 0.0 or not math.isfinite(remaining_path_m):
            raise ValueError("remaining path must be finite and non-negative")
        sign = 1.0 if desired_direction == "forward" else -1.0
        main_command = sign * self.supported_main_command_rad_s
        expected_speed = 0.0
        if desired_direction in ("forward", "reverse"):
            expected_auxiliary = self.drive_model.auxiliary_for_curvature(
                main_command,
                sign * float(command_curvature_1_m),
            )
            expected_speed = float(
                self.drive_model.speed(main_command, expected_auxiliary)
            )
        # A measured speed is a useful fallback at a stop, but keep the
        # estimate finite even when both measured and calibrated speeds are
        # zero.  This is a diagnostic-only conservative runway estimate.
        expected_speed = max(expected_speed, abs(float(current_speed_m_s)), 1e-6)
        estimated_travel_s = remaining_path_m / expected_speed
        runway = self.episode_budget_s - self._elapsed_s - estimated_travel_s
        if not math.isfinite(runway):
            raise ValueError("terminal runway diagnostic is non-finite")
        self._terminal_runway_s = float(runway)
        self._budget_infeasible = bool(runway < 0.0)

    def _validate_v2_diagnostics(self) -> None:
        scalar_names = (
            "projection_s_m",
            "delay_distance_m",
            "query_s_m",
            "pure_pursuit_curvature_1_m",
            "profile_curvature_1_m",
            "feedback_curvature_1_m",
            "blend_weight",
            "command_curvature_1_m",
            "cross_track_error_m",
            "lateral_error_m",
            "heading_error_rad",
            "main_cap_rad_s",
            "auxiliary_feedforward_rad",
            "auxiliary_feedback_rad",
            "elapsed_s",
            "terminal_runway_s",
            "terminal_brake_start_s",
            "goal_entry_s",
            "stop_below_0p05_s",
            "hold_complete_s",
        )
        diagnostics = self.diagnostics
        for name in scalar_names:
            try:
                value = float(diagnostics[name])
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                raise ValueError(
                    f"V2 diagnostic {name} must be a finite real scalar"
                ) from error
            if not math.isfinite(value):
                raise ValueError(
                    f"V2 diagnostic {name} must be a finite real scalar"
                )
        weight = float(diagnostics["blend_weight"])
        if not 0.0 <= weight <= 1.0:
            raise ValueError("V2 diagnostic blend_weight must be in [0, 1]")
        profile = self._profile
        if profile is not None:
            point_index = self._path_index
            if point_index < 0 or point_index >= profile.s_m.shape[0]:
                raise ValueError("V2 diagnostic reference index is outside profile")
            segment = int(profile.segment_id[point_index])
            segment_end = int(np.flatnonzero(profile.segment_id == segment)[-1])
            projection = float(diagnostics["projection_s_m"])
            query = float(diagnostics["query_s_m"])
            if projection < float(profile.s_m[point_index]) - 1e-9:
                raise ValueError("V2 projection arc length is outside current profile")
            if query < projection - 1e-9 or query > float(profile.s_m[segment_end]) + 1e-9:
                raise ValueError("V2 query arc length crosses the current segment")
            delay = float(diagnostics["delay_distance_m"])
            if delay < -1e-9 or delay > self.maximum_delay_distance_m + 1e-9:
                raise ValueError("V2 delay distance exceeds configured bound")

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        pointgoal = self._validate_act_inputs(observation, pointgoal_base_xy)
        reference, profile = self._require_reference()
        self._elapsed_s = self._action_index * self._ACTION_PERIOD_S
        self._action_index += 1

        current_speed = float(np.linalg.norm(observation.proprio_history[-1, :2]))
        current_yaw_rate = float(observation.proprio_history[-1, 5])
        self._reset_command_diagnostics()
        cross_track_error = self._project_to_reference(observation.position_world_xy)
        self._cross_track_error_m = cross_track_error
        desired_direction = self._desired_direction_for_current_index()
        self._desired_drive_direction = desired_direction

        if cross_track_error > 1.0:
            self._off_reference_cycles += 1
        else:
            self._off_reference_cycles = 0
        if self._off_reference_cycles >= 10:
            self._failure_subreason = "tracking_infeasible"
        if self._failure_subreason is not None:
            self._braking_state = "tracking_infeasible"
            self._validate_v2_diagnostics()
            return self._zero_action()

        goal_distance = float(np.linalg.norm(pointgoal))
        if goal_distance <= self._GOAL_TOLERANCE_M:
            self._record_terminal_brake()
            self._record_goal_entry(current_speed)
            self._braking_state = "goal_reached"
            self._update_budget_diagnostics(
                self._remaining_path_length(observation.position_world_xy),
                current_speed,
                desired_direction,
            )
            self._validate_v2_diagnostics()
            return self._zero_action()

        direction_action = self._handle_direction_change(
            desired_direction, current_speed
        )
        if direction_action is not None:
            self._update_budget_diagnostics(
                self._remaining_path_length(observation.position_world_xy),
                current_speed,
                desired_direction,
            )
            self._validate_v2_diagnostics()
            return direction_action

        reference_yaw = float(profile.yaw_unwrapped[self._path_index])
        delta = observation.position_world_xy - profile.xy[self._path_index]
        sign = 1.0 if desired_direction == "forward" else -1.0
        motion_tangent = reference_yaw + (math.pi if sign < 0.0 else 0.0)
        lateral_error = float(
            -math.sin(motion_tangent) * delta[0]
            + math.cos(motion_tangent) * delta[1]
        )
        heading_error = self._wrap_angle(
            reference_yaw - observation.yaw_world_rad
        )
        feedback_curvature = float(
            np.clip(
                -self._LATERAL_CURVATURE_GAIN_1_M2 * lateral_error
                + self._HEADING_CURVATURE_GAIN_1_M * math.sin(heading_error),
                -self.maximum_feedback_curvature_1_m,
                self.maximum_feedback_curvature_1_m,
            )
        )

        # Frozen Pure Pursuit geometry.  The geometric body-frame curvature
        # needs a direction sign to live in the same waypoint-order spatial
        # convention as the profile curvature on a reverse segment.
        lookahead_index = self._lookahead_index(self.feedback_lookahead_m)
        target_world = profile.xy[lookahead_index]
        world_delta = target_world - observation.position_world_xy
        cosine = math.cos(observation.yaw_world_rad)
        sine = math.sin(observation.yaw_world_rad)
        target_base = np.asarray(
            (
                cosine * world_delta[0] + sine * world_delta[1],
                -sine * world_delta[0] + cosine * world_delta[1],
            ),
            dtype=np.float64,
        )
        distance_squared = float(np.dot(target_base, target_base))
        geometric_curvature = (
            0.0
            if distance_squared <= 1e-9
            else 2.0 * float(target_base[1]) / distance_squared
        )
        pure_pursuit_curvature = sign * geometric_curvature

        query = query_delay_aware_reference(
            profile,
            current_index=self._path_index,
            projection_s_m=self._projection_s_m,
            speed_m_s=current_speed,
            auxiliary_latency_s=self.auxiliary_latency_s,
            profile_window_m=self.profile_window_m,
            maximum_delay_distance_m=self.maximum_delay_distance_m,
        )
        profile_curvature = float(query.curvature_1_m)
        blend_weight = curvature_blend_weight(
            profile_curvature,
            blend_start_1_m=self.profile_blend_start_1_m,
            blend_full_1_m=self.profile_blend_full_1_m,
        )
        command_curvature = blend_tracking_curvature(
            pure_pursuit_curvature,
            profile_curvature,
            feedback_curvature,
            weight=blend_weight,
        )
        self._projection_s_m = float(query.projection_s_m)
        self._delay_distance_m = float(query.delay_distance_m)
        self._query_s_m = float(query.query_s_m)
        self._pure_pursuit_curvature_1_m = float(pure_pursuit_curvature)
        self._profile_curvature_1_m = profile_curvature
        self._feedback_curvature_1_m = feedback_curvature
        self._blend_weight = float(blend_weight)
        self._command_curvature_1_m = float(command_curvature)
        self._lateral_error_m = lateral_error
        self._heading_error_rad = heading_error

        planned_main = float(reference[self._path_index].main_command_rad_s)
        remaining = self._remaining_path_length(observation.position_world_xy)
        self._update_budget_diagnostics(
            remaining,
            current_speed,
            desired_direction,
            command_curvature,
        )
        if remaining <= self.drive_model.stopping_distance(
            current_speed, desired_direction
        ) + self.terminal_brake_buffer_m:
            self._record_terminal_brake()
            if current_speed < self._STOP_SPEED_M_S:
                self._stop_below_0p05_s = (
                    self._elapsed_s
                    if self._stop_below_0p05_s < 0.0
                    else self._stop_below_0p05_s
                )
            self._braking_state = "goal_braking"
            self._reset_command_diagnostics()
            self._validate_v2_diagnostics()
            return self._zero_action()
        if planned_main == 0.0:
            self._record_terminal_brake()
            if current_speed < self._STOP_SPEED_M_S:
                self._stop_below_0p05_s = (
                    self._elapsed_s
                    if self._stop_below_0p05_s < 0.0
                    else self._stop_below_0p05_s
                )
            self._braking_state = "goal_braking"
            self._reset_command_diagnostics()
            self._validate_v2_diagnostics()
            return self._zero_action()

        main_command = sign * self.supported_main_command_rad_s
        if not math.isclose(
            abs(main_command),
            self._CALIBRATED_MAIN_COMMAND_RAD_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "moving V2 command is outside the calibrated 0.50 rad/s operating point"
            )
        drive_profile_curvature = sign * profile_curvature
        drive_command_curvature = sign * command_curvature
        auxiliary_feedforward = self.drive_model.auxiliary_for_curvature(
            main_command,
            drive_profile_curvature,
        )
        auxiliary_command = self.drive_model.auxiliary_for_curvature(
            main_command,
            drive_command_curvature,
        )
        desired_yaw_rate = (
            drive_command_curvature * sign * max(current_speed, 0.2)
        )
        correction = -sign * self._YAW_RATE_FEEDBACK_GAIN_S * (
            desired_yaw_rate - current_yaw_rate
        )
        correction = float(
            np.clip(correction, math.radians(-5.0), math.radians(5.0))
        )
        final_auxiliary = auxiliary_command + correction
        self._main_cap_rad_s = self.supported_main_command_rad_s
        self._auxiliary_feedforward_rad = float(auxiliary_feedforward)
        self._auxiliary_feedback_rad = float(
            final_auxiliary - auxiliary_feedforward
        )
        self._braking_state = "tracking"
        self._validate_v2_diagnostics()
        return _repeated_control(
            self.protocol,
            main_velocity=main_command,
            auxiliary_angle=final_auxiliary,
        )


class DynamicOraclePolicy(_PolicyBase):
    """Track a dynamic state-lattice reference using timed feedforward."""

    policy_name = "oracle_v2"
    privileged = True
    uses_shared_tracker = False

    def __init__(self, protocol: ActionProtocol, *, action_period_s: float = 0.1) -> None:
        super().__init__(protocol)
        period = float(action_period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("action_period_s must be finite and positive")
        self.action_period_s = period
        self._reference: tuple[DynamicWaypoint, ...] | None = None
        self._action_index = 0
        self._reference_index = 0
        self._active_drive_direction = "stopped"
        self._pending_drive_direction: str | None = None
        self._stopped_cycles = 0
        self._position_error_m = 0.0
        self._cross_track_error_m = 0.0
        self._yaw_error_rad = 0.0
        self._auxiliary_error_rad = 0.0
        self._desired_drive_direction = "stopped"
        self._braking_state = "tracking"

    @property
    def online_input_permissions(self) -> frozenset[str]:
        return frozenset(
            {
                "stereo_rgb",
                "proprio_history",
                "action_history",
                "perfect_position_yaw",
                "pointgoal_base_xy",
                "dynamic_reference",
            }
        )

    @property
    def diagnostics(self) -> dict[str, object]:
        return {
            "reference_index": self._reference_index,
            "position_error_m": self._position_error_m,
            "cross_track_error_m": self._cross_track_error_m,
            "yaw_error_rad": self._yaw_error_rad,
            "auxiliary_error_rad": self._auxiliary_error_rad,
            "active_drive_direction": self._active_drive_direction,
            "desired_drive_direction": self._desired_drive_direction,
            "pending_drive_direction": self._pending_drive_direction,
            "stopped_cycles": self._stopped_cycles,
            "braking_state": self._braking_state,
        }

    def reset(self, context: EpisodeContext) -> None:
        self._validate_context_protocol(context)
        if not context.privileged:
            raise PermissionError("DynamicOraclePolicy requires privileged context")
        if context.dynamic_reference is None:
            raise ValueError(
                "privileged DynamicOraclePolicy requires dynamic_reference"
            )
        if context.oracle_path_xy is not None or context.kinodynamic_reference is not None:
            raise ValueError("DynamicOraclePolicy accepts only dynamic_reference")
        self._context = context
        self._reference = tuple(context.dynamic_reference)
        self._action_index = 0
        self._reference_index = 0
        initial_direction = self._reference[0].drive_direction
        self._active_drive_direction = initial_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._position_error_m = 0.0
        self._cross_track_error_m = 0.0
        self._yaw_error_rad = 0.0
        self._auxiliary_error_rad = 0.0
        self._desired_drive_direction = initial_direction
        self._braking_state = "tracking"

    @staticmethod
    def _normalized_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def _reference_at(self, elapsed_s: float) -> DynamicWaypoint:
        if self._reference is None:
            raise RuntimeError("policy.reset() must be called before act()")
        times = [waypoint.time_from_start_s for waypoint in self._reference]
        right = int(np.searchsorted(times, elapsed_s, side="right"))
        if right == 0:
            self._reference_index = 0
            return self._reference[0]
        if right >= len(self._reference):
            self._reference_index = len(self._reference) - 1
            return self._reference[-1]
        left = right - 1
        first = self._reference[left]
        second = self._reference[right]
        if math.isclose(elapsed_s, first.time_from_start_s, abs_tol=1e-12):
            self._reference_index = left
            return first
        fraction = (elapsed_s - first.time_from_start_s) / (
            second.time_from_start_s - first.time_from_start_s
        )
        yaw_delta = self._normalized_angle(second.yaw_rad - first.yaw_rad)
        self._reference_index = left
        return DynamicWaypoint(
            x_m=first.x_m + fraction * (second.x_m - first.x_m),
            y_m=first.y_m + fraction * (second.y_m - first.y_m),
            yaw_rad=self._normalized_angle(first.yaw_rad + fraction * yaw_delta),
            main_command_rad_s=first.main_command_rad_s
            + fraction * (second.main_command_rad_s - first.main_command_rad_s),
            auxiliary_angle_rad=first.auxiliary_angle_rad
            + fraction * (second.auxiliary_angle_rad - first.auxiliary_angle_rad),
            speed_m_s=first.speed_m_s + fraction * (second.speed_m_s - first.speed_m_s),
            time_from_start_s=elapsed_s,
            drive_direction=first.drive_direction,
        )

    def _zero_action(self) -> np.ndarray:
        return np.zeros(self.protocol.action_block_dim, dtype=np.float32)

    def _handle_direction_change(
        self, desired_direction: str, speed_m_s: float
    ) -> np.ndarray | None:
        if desired_direction == "stopped":
            self._pending_drive_direction = None
            self._stopped_cycles = self._stopped_cycles + 1 if speed_m_s < 0.05 else 0
            self._braking_state = "goal_braking"
            return self._zero_action()
        if self._active_drive_direction == "stopped":
            self._active_drive_direction = desired_direction
            self._pending_drive_direction = None
            self._stopped_cycles = 0
            self._braking_state = "tracking"
            return None
        if desired_direction == self._active_drive_direction:
            self._pending_drive_direction = None
            self._stopped_cycles = 0
            self._braking_state = "tracking"
            return None
        self._pending_drive_direction = desired_direction
        self._braking_state = "direction_change"
        self._stopped_cycles = self._stopped_cycles + 1 if speed_m_s < 0.05 else 0
        if self._stopped_cycles < 5:
            return self._zero_action()
        self._active_drive_direction = desired_direction
        self._pending_drive_direction = None
        self._stopped_cycles = 0
        self._braking_state = "tracking"
        return None

    def act(
        self,
        observation: PolicyObservation,
        pointgoal_base_xy: object,
    ) -> np.ndarray:
        self._validate_act_inputs(observation, pointgoal_base_xy)
        reference = self._reference_at(self._action_index * self.action_period_s)
        self._action_index += 1

        delta = observation.position_world_xy - np.asarray(
            (reference.x_m, reference.y_m), dtype=np.float64
        )
        lateral = float(
            -math.sin(reference.yaw_rad) * delta[0]
            + math.cos(reference.yaw_rad) * delta[1]
        )
        yaw_error = self._normalized_angle(
            observation.yaw_world_rad - reference.yaw_rad
        )
        actual_auxiliary = float(observation.proprio_history[-1, 10])
        auxiliary_error = self._normalized_angle(
            actual_auxiliary - reference.auxiliary_angle_rad
        )
        self._position_error_m = float(np.linalg.norm(delta))
        self._cross_track_error_m = lateral
        self._yaw_error_rad = yaw_error
        self._auxiliary_error_rad = auxiliary_error
        self._desired_drive_direction = reference.drive_direction

        if self._reference is None:
            raise RuntimeError("policy.reset() must be called before act()")
        if self._reference_index == len(self._reference) - 1:
            self._braking_state = "goal_reached"
            return self._zero_action()

        speed_m_s = float(np.linalg.norm(observation.proprio_history[-1, :2]))
        direction_action = self._handle_direction_change(
            reference.drive_direction, speed_m_s
        )
        if direction_action is not None:
            return direction_action

        main_limit = math.inf
        errors = (
            abs(lateral),
            abs(yaw_error),
            abs(auxiliary_error),
        )
        if any(error >= threshold for error, threshold in zip(
            errors, (0.30, math.radians(30.0), math.radians(30.0))
        )):
            main_limit = 0.0
        elif any(error >= threshold for error, threshold in zip(
            errors, (0.20, math.radians(20.0), math.radians(20.0))
        )):
            main_limit = 0.25
        elif any(error >= threshold for error, threshold in zip(
            errors, (0.10, math.radians(10.0), math.radians(10.0))
        )):
            main_limit = 0.5

        main_command = math.copysign(
            min(abs(reference.main_command_rad_s), main_limit),
            reference.main_command_rad_s,
        ) if reference.main_command_rad_s != 0.0 else 0.0
        correction = float(
            np.clip(
                lateral + 0.5 * yaw_error,
                math.radians(-5.0),
                math.radians(5.0),
            )
        )
        self._braking_state = "tracking"
        return _repeated_control(
            self.protocol,
            main_velocity=main_command,
            auxiliary_angle=reference.auxiliary_angle_rad + correction,
        )
