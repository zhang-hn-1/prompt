"""Explicit production execution chain and the sole Round 7 physical model."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np

from scripts.action_smoothing import ActionSmoother, ActionSmootherSnapshot


PHYSICAL_STATE_NAMES = (
    "main_joint_velocity_rad_s",
    "auxiliary_joint_rad",
    "auxiliary_joint_velocity_rad_s",
    "body_forward_velocity_m_s",
    "body_lateral_velocity_m_s",
    "world_z_angular_velocity_rad_s",
)
M2_SCHEMA = "round7_m2_explicit_runtime_physical_delta_v1"


def _finite_vector(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


@dataclass(frozen=True)
class ExecutionSnapshot:
    current_applied: tuple[float, float]
    last_smoothed_target: tuple[float, float]
    tick_phase: int
    dt_s: float


@dataclass(frozen=True)
class ExecutionStep:
    protocol_request: np.ndarray
    smoothed_target: np.ndarray
    applied_target: np.ndarray
    supervisor_active: bool
    snapshot_after: ExecutionSnapshot


class ExplicitRuntimeChain:
    """Cloneable wrapper around the production ActionSmoother implementation."""

    def __init__(
        self,
        *,
        dt_s: float,
        action_minimum: Sequence[float],
        action_maximum: Sequence[float],
        max_main_acceleration_rad_s2: float,
        max_auxiliary_rate_rad_s: float,
        max_auxiliary_velocity_rad_s: float,
        controls_per_block: int = 5,
    ) -> None:
        self.dt_s = float(dt_s)
        self.minimum = _finite_vector(action_minimum, (2,), "action_minimum")
        self.maximum = _finite_vector(action_maximum, (2,), "action_maximum")
        if np.any(self.minimum > self.maximum):
            raise ValueError("action_minimum must not exceed action_maximum")
        if not isinstance(controls_per_block, int) or controls_per_block <= 0:
            raise ValueError("controls_per_block must be a positive integer")
        self.controls_per_block = controls_per_block
        self._smoother = ActionSmoother(
            dt_s=self.dt_s,
            max_main_acceleration_rad_s2=max_main_acceleration_rad_s2,
            max_auxiliary_rate_rad_s=max_auxiliary_rate_rad_s,
            max_auxiliary_velocity_rad_s=max_auxiliary_velocity_rad_s,
        )
        self._tick_phase = 0

    def reset(
        self,
        *,
        applied_action: Sequence[float] = (0.0, 0.0),
        last_smoothed_target: Sequence[float] | None = None,
        tick_phase: int = 0,
    ) -> None:
        applied = _finite_vector(applied_action, (2,), "applied_action")
        smoothed = applied if last_smoothed_target is None else _finite_vector(
            last_smoothed_target, (2,), "last_smoothed_target"
        )
        if not isinstance(tick_phase, int) or not 0 <= tick_phase < self.controls_per_block:
            raise ValueError("tick_phase lies outside the policy block")
        self._smoother.restore(
            ActionSmootherSnapshot(tuple(map(float, applied)), tuple(map(float, smoothed)))
        )
        self._tick_phase = tick_phase

    def snapshot(self) -> ExecutionSnapshot:
        state = self._smoother.snapshot()
        return ExecutionSnapshot(
            current_applied=state.current_action,
            last_smoothed_target=state.last_smoothed_target,
            tick_phase=self._tick_phase,
            dt_s=self.dt_s,
        )

    def restore(self, snapshot: ExecutionSnapshot) -> None:
        if not isinstance(snapshot, ExecutionSnapshot):
            raise TypeError("snapshot must be ExecutionSnapshot")
        if not math.isclose(snapshot.dt_s, self.dt_s, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("snapshot dt differs from chain dt")
        self.reset(
            applied_action=snapshot.current_applied,
            last_smoothed_target=snapshot.last_smoothed_target,
            tick_phase=snapshot.tick_phase,
        )

    def clone(self) -> "ExplicitRuntimeChain":
        cloned = ExplicitRuntimeChain(
            dt_s=self.dt_s,
            action_minimum=self.minimum,
            action_maximum=self.maximum,
            max_main_acceleration_rad_s2=self._smoother.max_main_acceleration_rad_s2,
            max_auxiliary_rate_rad_s=self._smoother.max_auxiliary_rate_rad_s,
            max_auxiliary_velocity_rad_s=float(
                self._smoother.max_auxiliary_velocity_rad_s
            ),
            controls_per_block=self.controls_per_block,
        )
        cloned.restore(self.snapshot())
        return cloned

    def step(
        self,
        protocol_request: Sequence[float],
        *,
        auxiliary_position_rad: float,
        auxiliary_velocity_rad_s: float,
    ) -> ExecutionStep:
        request = np.clip(
            _finite_vector(protocol_request, (2,), "protocol_request"),
            self.minimum,
            self.maximum,
        )
        applied = self._smoother.step(
            request,
            self.minimum,
            self.maximum,
            auxiliary_position_rad=float(auxiliary_position_rad),
            auxiliary_velocity_rad_s=float(auxiliary_velocity_rad_s),
        ).astype(np.float64)
        smoothed = self._smoother.last_smoothed_target.astype(np.float64)
        active = abs(float(auxiliary_velocity_rad_s)) > float(
            self._smoother.max_auxiliary_velocity_rad_s
        )
        self._tick_phase = (self._tick_phase + 1) % self.controls_per_block
        return ExecutionStep(request, smoothed, applied, active, self.snapshot())


@dataclass
class StopReadyCounter:
    threshold_m_s: float = 0.05
    required_samples: int = 5
    initial_count: int = 0

    def __post_init__(self) -> None:
        if not math.isfinite(self.threshold_m_s) or self.threshold_m_s <= 0.0:
            raise ValueError("threshold_m_s must be finite and positive")
        if self.required_samples <= 0 or not 0 <= self.initial_count < self.required_samples:
            raise ValueError("invalid stop-ready count")
        self.count = int(self.initial_count)

    def update(self, planar_speed_m_s: float) -> bool:
        speed = float(planar_speed_m_s)
        if not math.isfinite(speed):
            raise ValueError("planar speed must be finite")
        self.count = self.count + 1 if speed < self.threshold_m_s else 0
        self.count = min(self.count, self.required_samples)
        return self.count >= self.required_samples


def classify_event_pair(
    *,
    actual_time_s: float | None,
    predicted_time_s: float | None,
    applicable: bool,
    horizon_s: float,
    available_duration_s: float,
) -> dict[str, object]:
    if not applicable:
        return {
            "actual_status": "not_applicable",
            "predicted_status": "not_applicable",
            "comparison": "not_applicable",
            "timing_error_s": None,
        }
    if available_duration_s + 1e-12 < horizon_s:
        return {
            "actual_status": "invalid_measurement",
            "predicted_status": "invalid_measurement",
            "comparison": "invalid_measurement",
            "timing_error_s": None,
        }
    actual_observed = actual_time_s is not None and actual_time_s <= horizon_s + 1e-12
    predicted_observed = predicted_time_s is not None and predicted_time_s <= horizon_s + 1e-12
    actual_status = "observed_within_horizon" if actual_observed else "not_observed_by_horizon"
    predicted_status = "observed_within_horizon" if predicted_observed else "not_observed_by_horizon"
    if actual_observed and predicted_observed:
        comparison = "correct_detection"
        error = float(predicted_time_s - actual_time_s)  # type: ignore[operator]
    elif actual_observed:
        comparison, error = "missed_event", None
    elif predicted_observed:
        comparison, error = "false_positive", None
    else:
        comparison, error = "both_not_observed", None
    return {
        "actual_status": actual_status,
        "predicted_status": predicted_status,
        "comparison": comparison,
        "timing_error_s": error,
    }


def matched_request_prefix_ticks(
    planned_requests: object, actual_requests: object, *, atol: float = 1e-9
) -> int:
    planned = np.asarray(planned_requests, dtype=np.float64)
    actual = np.asarray(actual_requests, dtype=np.float64)
    if planned.ndim != 2 or planned.shape[1:] != (2,) or actual.ndim != 2 or actual.shape[1:] != (2,):
        raise ValueError("request traces must have shape (N, 2)")
    count = min(len(planned), len(actual))
    for index in range(count):
        if not np.allclose(planned[index], actual[index], rtol=0.0, atol=atol):
            return index
    return count


def m2_features(physical_state: np.ndarray, applied_target: np.ndarray) -> np.ndarray:
    state = _finite_vector(physical_state, (6,), "physical_state")
    applied = _finite_vector(applied_target, (2,), "applied_target")
    return np.concatenate(
        (
            [1.0],
            state,
            applied,
            [
                state[0] * state[1],
                state[3] * state[1],
                applied[0] - state[0],
                applied[1] - state[1],
            ],
        )
    )


@dataclass(frozen=True)
class M2Rollout:
    physical_state_trace: np.ndarray
    smoothed_target_trace: np.ndarray
    applied_target_trace: np.ndarray
    supervisor_active_trace: np.ndarray
    local_trajectory_xy_m: np.ndarray
    heading_trace_rad: np.ndarray
    final_execution_snapshot: ExecutionSnapshot


class M2PhysicalPredictor:
    def __init__(self, payload: Mapping[str, object], execution_chain: ExplicitRuntimeChain) -> None:
        if payload.get("schema_version") != M2_SCHEMA:
            raise ValueError("Round 7 M2 schema is required")
        if tuple(payload.get("physical_state_names", ())) != PHYSICAL_STATE_NAMES:
            raise ValueError("M2 physical state order mismatch")
        if int(payload.get("feature_count", -1)) != 13:
            raise ValueError("M2 feature count mismatch")
        coefficients = np.asarray(
            payload["coefficient_feature_by_physical_state"], dtype=np.float64
        )
        if coefficients.shape != (13, 6) or not np.all(np.isfinite(coefficients)):
            raise ValueError("M2 coefficients must have shape (13, 6)")
        causal = payload.get("causal_contract")
        if not isinstance(causal, Mapping) or any(
            causal.get(name) is not False
            for name in (
                "future_measured_state_used",
                "future_applied_action_used",
                "future_supervisor_output_used",
            )
        ):
            raise ValueError("M2 causal contract forbids future truth")
        dt_s = float(payload["dt_s"])
        if not math.isclose(dt_s, execution_chain.dt_s, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("model and execution-chain dt differ")
        self.coefficients = coefficients.copy()
        self.execution_chain = execution_chain.clone()
        self.dt_s = dt_s

    def rollout(
        self,
        *,
        current_physical_state: Sequence[float],
        execution_snapshot: ExecutionSnapshot,
        future_protocol_requests: object,
        start_yaw_rad: float,
    ) -> M2Rollout:
        state = _finite_vector(current_physical_state, (6,), "current_physical_state").copy()
        requests = np.asarray(future_protocol_requests, dtype=np.float64)
        if requests.ndim != 2 or requests.shape[1:] != (2,) or not np.all(np.isfinite(requests)):
            raise ValueError("future_protocol_requests must have shape (N, 2)")
        chain = self.execution_chain.clone()
        chain.restore(execution_snapshot)
        yaw = float(start_yaw_rad)
        if not math.isfinite(yaw):
            raise ValueError("start_yaw_rad must be finite")
        start_yaw = yaw
        position = np.zeros(2, dtype=np.float64)
        states = [state.copy()]
        smoothed, applied, active = [], [], []
        positions, headings = [position.copy()], [yaw]
        for request in requests:
            execution = chain.step(
                request,
                auxiliary_position_rad=float(state[1]),
                auxiliary_velocity_rad_s=float(state[2]),
            )
            state = state + m2_features(state, execution.applied_target) @ self.coefficients
            if not np.all(np.isfinite(state)):
                raise ValueError("M2 recursive rollout became non-finite")
            yaw += float(state[5]) * self.dt_s
            relative_yaw = yaw - start_yaw
            c, s = math.cos(relative_yaw), math.sin(relative_yaw)
            velocity = np.asarray(
                (c * state[3] - s * state[4], s * state[3] + c * state[4])
            )
            position += velocity * self.dt_s
            states.append(state.copy())
            smoothed.append(execution.smoothed_target.copy())
            applied.append(execution.applied_target.copy())
            active.append(execution.supervisor_active)
            positions.append(position.copy())
            headings.append(yaw)
        return M2Rollout(
            physical_state_trace=np.asarray(states),
            smoothed_target_trace=np.asarray(smoothed).reshape((-1, 2)),
            applied_target_trace=np.asarray(applied).reshape((-1, 2)),
            supervisor_active_trace=np.asarray(active, dtype=bool),
            local_trajectory_xy_m=np.asarray(positions),
            heading_trace_rad=np.asarray(headings),
            final_execution_snapshot=chain.snapshot(),
        )
