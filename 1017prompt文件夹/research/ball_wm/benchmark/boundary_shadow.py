"""Causal, read-only reference-boundary shadow validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from training.action_protocol import ActionProtocol

from .action_layers import classify_signed_direction
from .policies import EpisodeContext, OraclePathPolicy, PolicyObservation


STATE_NAMES = (
    "main_joint_velocity_rad_s",
    "auxiliary_joint_rad",
    "auxiliary_joint_velocity_rad_s",
    "body_forward_velocity_m_s",
    "body_lateral_velocity_m_s",
    "world_z_angular_velocity_rad_s",
    "final_main_rad_s",
    "final_auxiliary_rad",
)
REQUEST_NAMES = ("requested_main_rad_s", "requested_auxiliary_rad")


@dataclass(frozen=True)
class BoundaryContext:
    control_block_index: int
    control_time_s: float
    measurement_timestamp_s: float
    reference_index: int
    reference_direction: str
    active_reference_direction: str
    pending_reference_direction: str | None
    reference_progress_s_m: float
    reference_segment_progress_s_m: float
    request_direction: str
    applied_direction: str
    main_joint_direction: str
    body_motion_direction: str
    policy_raw_component: tuple[float, float]
    protocol_request_component: tuple[float, float]
    previous_applied_component: tuple[float, float]
    smoother_current_action_component: tuple[float, float]
    position_world_xy_m: tuple[float, float]
    root_yaw_rad: float
    measured_state: tuple[float, ...]
    braking_state: str
    stopped_cycles: int
    request_source: str
    request_clipped: bool
    request_clip_reason: str | None
    policy_action_block_sha256: str

    @property
    def value_time_classification(self) -> dict[str, str]:
        return {
            "reference_direction": "current_known",
            "active_reference_direction": "current_known",
            "pending_reference_direction": "current_known",
            "reference_progress_s_m": "current_known",
            "reference_segment_progress_s_m": "current_measured_projection",
            "request_direction": "current_known",
            "applied_direction": "current_known_previous_tick",
            "main_joint_direction": "current_measured",
            "body_motion_direction": "current_measured",
            "policy_raw_component": "current_known",
            "protocol_request_component": "current_known",
            "previous_applied_component": "current_known_previous_tick",
            "smoother_current_action_component": "current_known_previous_tick",
            "measured_state": "current_measured",
        }

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["value_time_classification"] = self.value_time_classification
        return payload


@dataclass(frozen=True)
class M1Rollout:
    state_trace: np.ndarray
    local_trajectory_xy_m: np.ndarray
    heading_trace_rad: np.ndarray


@dataclass(frozen=True)
class SupportEnvelope:
    """Frozen state-request fit cloud used only to decide abstention."""

    points: np.ndarray
    center: np.ndarray
    scale: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    maximum_distance: float

    @classmethod
    def from_points(
        cls, points: np.ndarray, *, maximum_distance: float
    ) -> "SupportEnvelope":
        values = np.asarray(points, dtype=np.float64)
        threshold = float(maximum_distance)
        if values.ndim != 2 or values.shape[1] != 10 or values.shape[0] < 2:
            raise ValueError("support points must have shape (N, 10), N >= 2")
        if not np.all(np.isfinite(values)) or not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("support points and maximum distance must be finite")
        center = values.mean(axis=0)
        scale = values.std(axis=0)
        scale[scale < 1e-6] = 1.0
        return cls(
            points=values.copy(),
            center=center,
            scale=scale,
            lower=values.min(axis=0),
            upper=values.max(axis=0),
            maximum_distance=threshold,
        )

    def check(
        self, current_state: Sequence[float], protocol_request: Sequence[float]
    ) -> dict[str, object]:
        state = np.asarray(current_state, dtype=np.float64)
        request = np.asarray(protocol_request, dtype=np.float64)
        if state.shape != (8,) or request.shape != (2,):
            raise ValueError("support query requires eight state and two request values")
        query = np.concatenate((state, request))
        if not np.all(np.isfinite(query)):
            return {
                "supported": False,
                "inside_feature_box": False,
                "standardized_nearest_neighbor_distance": None,
                "reason": "non_finite_query",
            }
        inside = bool(np.all(query >= self.lower - 1e-9) and np.all(query <= self.upper + 1e-9))
        normalized = (self.points - query) / self.scale
        distance = float(np.min(np.linalg.norm(normalized, axis=1)))
        supported = inside and distance <= self.maximum_distance
        return {
            "supported": supported,
            "inside_feature_box": inside,
            "standardized_nearest_neighbor_distance": distance,
            "maximum_distance": self.maximum_distance,
            "reason": "supported" if supported else ("outside_feature_box" if not inside else "nearest_neighbor_too_far"),
        }

    def check_trajectory(
        self, state_trace_before_request: np.ndarray, protocol_requests: np.ndarray
    ) -> dict[str, object]:
        states = np.asarray(state_trace_before_request, dtype=np.float64)
        requests = np.asarray(protocol_requests, dtype=np.float64)
        if states.ndim != 2 or states.shape[1] != 8 or requests.shape != (len(states), 2):
            raise ValueError("trajectory support requires aligned (N, 8) states and (N, 2) requests")
        checks = [self.check(state, request) for state, request in zip(states, requests)]
        unsupported = [index for index, item in enumerate(checks) if not item["supported"]]
        distances = [
            float(item["standardized_nearest_neighbor_distance"])
            for item in checks
            if item["standardized_nearest_neighbor_distance"] is not None
        ]
        return {
            "supported": not unsupported,
            "checked_tick_count": len(checks),
            "first_unsupported_tick": unsupported[0] if unsupported else None,
            "maximum_standardized_nearest_neighbor_distance": max(distances) if distances else None,
            "inside_feature_box_all_ticks": all(bool(item["inside_feature_box"]) for item in checks),
            "initial": checks[0] if checks else None,
            "reason": "supported" if not unsupported else str(checks[unsupported[0]]["reason"]),
        }


class M1TransitionPredictor:
    """Frozen Round 5 recursive model with a deliberately narrow causal API."""

    def __init__(self, payload: Mapping[str, object]) -> None:
        if payload.get("schema_version") != "round5_linear_state_delta_v1":
            raise ValueError("Round 5 M1 schema is required")
        if tuple(payload.get("state_names", ())) != STATE_NAMES:
            raise ValueError("M1 state order mismatch")
        if tuple(payload.get("request_names", ())) != REQUEST_NAMES:
            raise ValueError("M1 request order mismatch")
        causal = payload.get("causal_contract")
        if not isinstance(causal, Mapping):
            raise ValueError("M1 causal contract is required")
        if causal.get("future_measured_state_used") is not False:
            raise ValueError("M1 must not use future measured state")
        if causal.get("future_supervisor_output_used") is not False:
            raise ValueError("M1 must not use future supervisor output")
        coefficients = np.asarray(payload["coefficient_feature_by_state"], dtype=np.float64)
        if coefficients.shape != (15, 8) or not np.all(np.isfinite(coefficients)):
            raise ValueError("M1 coefficients must have shape (15, 8) and be finite")
        dt = float(payload["dt_s"])
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("M1 dt_s must be finite and positive")
        self._coefficients = coefficients.copy()
        self._dt_s = dt

    @classmethod
    def from_json(cls, path: str | Path) -> "M1TransitionPredictor":
        source = Path(path)
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"could not load M1 model: {source}") from error
        if not isinstance(payload, dict):
            raise ValueError("M1 JSON root must be an object")
        return cls(payload)

    @property
    def dt_s(self) -> float:
        return self._dt_s

    @staticmethod
    def _features(state: np.ndarray, request: np.ndarray) -> np.ndarray:
        return np.concatenate(
            (
                [1.0],
                state,
                request,
                [
                    state[0] * state[1],
                    state[3] * state[1],
                    request[0] - state[6],
                    request[1] - state[7],
                ],
            )
        )

    def rollout(
        self,
        current_state: Sequence[float],
        future_protocol_requests: np.ndarray,
        *,
        start_yaw_rad: float,
    ) -> M1Rollout:
        state = np.asarray(current_state, dtype=np.float64).copy()
        requests = np.asarray(future_protocol_requests, dtype=np.float64)
        yaw = float(start_yaw_rad)
        if state.shape != (8,) or not np.all(np.isfinite(state)):
            raise ValueError("current_state must be eight finite values")
        if requests.ndim != 2 or requests.shape[1] != 2 or not np.all(np.isfinite(requests)):
            raise ValueError("future protocol requests must have shape (N, 2)")
        if not math.isfinite(yaw):
            raise ValueError("start_yaw_rad must be finite")
        local_position = np.zeros(2, dtype=np.float64)
        state_trace = [state.copy()]
        trajectory = [local_position.copy()]
        headings = [yaw]
        start_yaw = yaw
        for request in requests:
            state += self._features(state, request) @ self._coefficients
            if not np.all(np.isfinite(state)):
                raise ValueError("M1 recursive rollout became non-finite")
            state[6] = float(np.clip(state[6], -2.0, 2.0))
            state[7] = float(np.clip(state[7], -math.radians(30.0), math.radians(30.0)))
            yaw += float(state[5]) * self._dt_s
            relative_yaw = yaw - start_yaw
            cosine, sine = math.cos(relative_yaw), math.sin(relative_yaw)
            local_velocity = np.asarray(
                (
                    cosine * state[3] - sine * state[4],
                    sine * state[3] + cosine * state[4],
                )
            )
            local_position += local_velocity * self._dt_s
            state_trace.append(state.copy())
            trajectory.append(local_position.copy())
            headings.append(yaw)
        return M1Rollout(
            state_trace=np.asarray(state_trace),
            local_trajectory_xy_m=np.asarray(trajectory),
            heading_trace_rad=np.asarray(headings),
        )


def measured_state_from_observation(observation: PolicyObservation) -> tuple[float, ...]:
    current = np.asarray(observation.proprio_history[-1], dtype=np.float64)
    yaw = float(observation.yaw_world_rad)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world_x, world_y = float(current[0]), float(current[1])
    body_forward = cosine * world_x + sine * world_y
    body_lateral = -sine * world_x + cosine * world_y
    return (
        float(current[12]),
        float(current[10]),
        float(current[13]),
        body_forward,
        body_lateral,
        float(current[5]),
        float(current[15]),
        float(current[16]),
    )


class ReadOnlyBoundaryShadowPolicy:
    """Call B0 exactly once and append observational boundary records."""

    policy_name = "oracle_b0_shadow"
    privileged = True

    def __init__(
        self,
        base_policy: OraclePathPolicy,
        *,
        predictor: M1TransitionPredictor | None,
        support_envelope: SupportEnvelope | None = None,
        model_accepted: bool = False,
        horizon_s: float = 1.5,
        policy_hz: int = 10,
    ) -> None:
        if not isinstance(base_policy, OraclePathPolicy):
            raise TypeError("base_policy must be OraclePathPolicy")
        if not isinstance(policy_hz, int) or policy_hz <= 0:
            raise ValueError("policy_hz must be a positive integer")
        self.base_policy = base_policy
        self.predictor = predictor
        self.support_envelope = support_envelope
        self.model_accepted = bool(model_accepted)
        self.horizon_s = float(horizon_s)
        if not math.isfinite(self.horizon_s) or self.horizon_s <= 0.0:
            raise ValueError("horizon_s must be finite and positive")
        self.policy_hz = policy_hz
        self._contexts: list[BoundaryContext] = []
        self._records: list[dict[str, object]] = []
        self._context: EpisodeContext | None = None

    @property
    def contexts(self) -> tuple[BoundaryContext, ...]:
        return tuple(self._contexts)

    @property
    def records(self) -> tuple[dict[str, object], ...]:
        return tuple(self._records)

    @property
    def diagnostics(self) -> dict[str, object]:
        result = dict(self.base_policy.diagnostics)
        for name in (
            "reference_progress_s_m",
            "reference_heading_rad",
            "reference_curvature_1_m",
            "desired_curvature_1_m",
            "baseline_main_command_rad_s",
            "scheduled_main_command_rad_s",
            "auxiliary_target_angle_rad",
            "auxiliary_velocity_rad_s",
            "scheduler_active",
            "scheduler_trigger_code",
        ):
            result[name] = getattr(self.base_policy, f"_{name}")
        result["shadow_context_count"] = len(self._contexts)
        return result

    @property
    def online_input_permissions(self):
        return self.base_policy.online_input_permissions

    def reset(self, context: EpisodeContext) -> None:
        self.base_policy.reset(context)
        self._context = context
        self._contexts.clear()
        self._records.clear()

    @staticmethod
    def _project(point: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
        if len(reference) == 1:
            return 0.0, float(np.linalg.norm(point - reference[0]))
        cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
        best_progress, best_error = 0.0, math.inf
        for index, (left, right) in enumerate(zip(reference, reference[1:])):
            delta = right - left
            length2 = float(delta @ delta)
            fraction = 0.0 if length2 <= 1e-12 else float(
                np.clip((point - left) @ delta / length2, 0.0, 1.0)
            )
            projected = left + fraction * delta
            error = float(np.linalg.norm(point - projected))
            if error < best_error:
                best_error = error
                best_progress = float(cumulative[index] + fraction * math.sqrt(length2))
        return best_progress, best_error

    def _reference_segment(self, reference_items, index: int) -> tuple[np.ndarray, float]:
        direction = reference_items[index].drive_direction
        left = index
        while left > 0 and reference_items[left - 1].drive_direction == direction:
            left -= 1
        right = index
        while right + 1 < len(reference_items) and reference_items[right + 1].drive_direction == direction:
            right += 1
        full = np.asarray([(item.x_m, item.y_m) for item in reference_items], dtype=np.float64)
        offset = float(np.linalg.norm(np.diff(full[: left + 1], axis=0), axis=1).sum()) if left > 0 else 0.0
        return full[left : right + 1], offset

    @staticmethod
    def _zero_crossing_time(
        values: np.ndarray, *, target: float, dt_s: float
    ) -> float | None:
        if abs(target) <= 1e-9 or values[0] * target >= 0.0:
            return None
        signed = values * math.copysign(1.0, target)
        candidates = np.flatnonzero(signed >= 0.0)
        candidates = candidates[candidates > 0]
        if not candidates.size:
            return None
        index = int(candidates[0])
        before, after = float(signed[index - 1]), float(signed[index])
        fraction = 1.0 if math.isclose(before, after) else float(np.clip(-before / (after - before), 0.0, 1.0))
        return float((index - 1 + fraction) * dt_s)

    def _shadow_record(self, context: BoundaryContext) -> dict[str, object]:
        assert self._context is not None
        base_diagnostics = self.base_policy.diagnostics
        boundary_active = bool(
            base_diagnostics.get("pending_drive_direction") is not None
            or base_diagnostics.get("braking_state") == "direction_change"
            or base_diagnostics.get("desired_drive_direction")
            != base_diagnostics.get("active_drive_direction")
        )
        record: dict[str, object] = {
            "context": context.to_dict(),
            "boundary_active": boundary_active,
            "plans": {},
            "advice": "not_at_boundary",
            "advice_time_classification": "current_computed",
        }
        if not boundary_active:
            return record
        if self.predictor is None:
            record["advice"] = "abstain_no_predictor"
            return record
        reference_items = self._context.kinodynamic_reference
        if reference_items is None or not reference_items:
            record["advice"] = "abstain_no_reference"
            return record
        index = int(np.clip(context.reference_index, 0, len(reference_items) - 1))
        waypoint = reference_items[index]
        handoff_raw = np.asarray(
            (float(waypoint.main_command_rad_s), float(waypoint.auxiliary_angle_rad)),
            dtype=np.float32,
        )
        handoff_block = np.tile(handoff_raw, self.base_policy.protocol.controls_per_block)
        handoff = self.base_policy.protocol.clip_block(handoff_block)[:2].astype(np.float64)
        plan_components = {
            "continue_b0": np.asarray(context.protocol_request_component, dtype=np.float64),
            "handoff_candidate": handoff,
        }
        tick_count = int(round(self.horizon_s / self.predictor.dt_s))
        reference, progress_offset = self._reference_segment(reference_items, index)
        start_position = np.asarray(context.position_world_xy_m, dtype=np.float64)
        start_progress, start_cross_track = self._project(start_position, reference)
        plans: dict[str, object] = {}
        for name, component in plan_components.items():
            requests = np.tile(component, (tick_count, 1))
            rollout = self.predictor.rollout(
                context.measured_state,
                requests,
                start_yaw_rad=context.root_yaw_rad,
            )
            support = (
                self.support_envelope.check_trajectory(rollout.state_trace[:-1], requests)
                if self.support_envelope is not None
                else {
                    "supported": False,
                    "reason": "support_envelope_unavailable",
                    "checked_tick_count": 0,
                    "first_unsupported_tick": 0,
                }
            )
            local_endpoint = rollout.local_trajectory_xy_m[-1]
            cosine, sine = math.cos(context.root_yaw_rad), math.sin(context.root_yaw_rad)
            world_endpoint = start_position + np.asarray(
                (
                    cosine * local_endpoint[0] - sine * local_endpoint[1],
                    sine * local_endpoint[0] + cosine * local_endpoint[1],
                )
            )
            progress, cross_track = self._project(world_endpoint, reference)
            main_crossing = self._zero_crossing_time(
                rollout.state_trace[:, 0], target=float(component[0]), dt_s=self.predictor.dt_s
            )
            auxiliary_crossing = self._zero_crossing_time(
                rollout.state_trace[:, 1], target=float(component[1]), dt_s=self.predictor.dt_s
            )
            plans[name] = {
                "protocol_request_component": component.tolist(),
                "request_tick_count": tick_count,
                "input_time_classification": "future_planned_now",
                "prediction_time_classification": "predicted",
                "request_plan_sha256": hashlib.sha256(requests.astype(np.float64).tobytes()).hexdigest(),
                "support": support,
                "predicted_local_endpoint_xy_m": local_endpoint.tolist(),
                "predicted_world_endpoint_xy_m": world_endpoint.tolist(),
                "predicted_progress_delta_m": float(progress - start_progress),
                "predicted_reference_progress_s_m": float(progress_offset + progress),
                "predicted_cross_track_m": cross_track,
                "start_cross_track_m": start_cross_track,
                "predicted_terminal_state": rollout.state_trace[-1].tolist(),
                "predicted_local_trajectory_xy_m": rollout.local_trajectory_xy_m.tolist(),
                "predicted_heading_trace_rad": rollout.heading_trace_rad.tolist(),
                "predicted_state_trace": rollout.state_trace.tolist(),
                "predicted_main_zero_crossing_s": main_crossing,
                "predicted_auxiliary_zero_crossing_s": auxiliary_crossing,
                "invalid_reason": None if support["supported"] else support["reason"],
            }
        record["plans"] = plans
        if not self.model_accepted:
            record["advice"] = "abstain_model_rejected"
            record["invalid_reason"] = "model_rejected_by_validation_gate"
            return record
        if not all(bool(plan["support"]["supported"]) for plan in plans.values()):
            record["advice"] = "abstain_out_of_support"
            record["invalid_reason"] = "one_or_more_rollouts_leave_support"
            return record
        continue_plan = plans["continue_b0"]
        handoff_plan = plans["handoff_candidate"]
        separation = float(
            np.linalg.norm(
                np.asarray(handoff_plan["predicted_world_endpoint_xy_m"])
                - np.asarray(continue_plan["predicted_world_endpoint_xy_m"])
            )
        )
        progress_gain = float(
            handoff_plan["predicted_progress_delta_m"]
            - continue_plan["predicted_progress_delta_m"]
        )
        cross_track_worsening = float(
            handoff_plan["predicted_cross_track_m"]
            - continue_plan["predicted_cross_track_m"]
        )
        record["comparison"] = {
            "endpoint_separation_m": separation,
            "handoff_progress_gain_m": progress_gain,
            "handoff_cross_track_worsening_m": cross_track_worsening,
        }
        record["advice"] = (
            "handoff_candidate_supported"
            if separation >= 0.15 and progress_gain >= 0.10 and cross_track_worsening <= 0.05
            else "abstain_inconclusive"
        )
        record["invalid_reason"] = None if record["advice"] == "handoff_candidate_supported" else "prediction_difference_below_resolution_gate"
        return record

    def act(self, observation: PolicyObservation, pointgoal_base_xy: object) -> np.ndarray:
        action = self.base_policy.act(observation, pointgoal_base_xy)
        # self.diagnostics adds live reference progress/heading fields that the
        # frozen base diagnostics payload intentionally does not expose.
        diagnostics = self.diagnostics
        request_diagnostics = self.base_policy.request_diagnostics
        component = np.asarray(action[:2], dtype=np.float64)
        raw = (
            float(request_diagnostics["policy_raw_main_command_rad_s"]),
            float(request_diagnostics["policy_raw_auxiliary_target_rad"]),
        )
        current = np.asarray(observation.proprio_history[-1], dtype=np.float64)
        previous_applied = (float(current[15]), float(current[16]))
        state = measured_state_from_observation(observation)
        block_index = len(self._contexts)
        reference_items = None if self._context is None else self._context.kinodynamic_reference
        reference_segment_progress = 0.0
        if reference_items:
            segment, _ = self._reference_segment(
                reference_items,
                int(np.clip(int(diagnostics.get("reference_index", 0)), 0, len(reference_items) - 1)),
            )
            reference_segment_progress, _ = self._project(
                np.asarray(observation.position_world_xy, dtype=np.float64), segment
            )
        context = BoundaryContext(
            control_block_index=block_index,
            control_time_s=block_index / float(self.policy_hz),
            measurement_timestamp_s=block_index / float(self.policy_hz),
            reference_index=int(diagnostics.get("reference_index", -1)),
            reference_direction=str(diagnostics.get("desired_drive_direction", "unknown")),
            active_reference_direction=str(diagnostics.get("active_drive_direction", "unknown")),
            pending_reference_direction=diagnostics.get("pending_drive_direction"),
            reference_progress_s_m=float(diagnostics.get("reference_progress_s_m", 0.0)),
            reference_segment_progress_s_m=reference_segment_progress,
            request_direction=classify_signed_direction(component[0], deadband=0.02),
            applied_direction=classify_signed_direction(previous_applied[0], deadband=0.02),
            main_joint_direction=classify_signed_direction(state[0], deadband=0.05),
            body_motion_direction=classify_signed_direction(state[3], deadband=0.05),
            policy_raw_component=raw,
            protocol_request_component=(float(component[0]), float(component[1])),
            previous_applied_component=previous_applied,
            smoother_current_action_component=previous_applied,
            position_world_xy_m=tuple(map(float, observation.position_world_xy)),
            root_yaw_rad=float(observation.yaw_world_rad),
            measured_state=state,
            braking_state=str(diagnostics.get("braking_state", "unknown")),
            stopped_cycles=int(diagnostics.get("stopped_cycles", 0)),
            request_source=str(request_diagnostics["request_source"]),
            request_clipped=bool(request_diagnostics["policy_request_clipped"]),
            request_clip_reason=request_diagnostics["policy_request_clip_reason"],
            policy_action_block_sha256=hashlib.sha256(
                np.asarray(action, dtype=np.float32).tobytes()
            ).hexdigest(),
        )
        self._contexts.append(context)
        self._records.append(self._shadow_record(context))
        return action
