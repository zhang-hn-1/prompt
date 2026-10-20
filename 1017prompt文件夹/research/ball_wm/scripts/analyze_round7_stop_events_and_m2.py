#!/usr/bin/env python3
"""Evaluate Round 6 event censoring and the sole frozen Round 7 M2 candidate."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.explicit_runtime_model import (  # noqa: E402
    ExplicitRuntimeChain,
    M2PhysicalPredictor,
    PHYSICAL_STATE_NAMES,
    StopReadyCounter,
    classify_event_pair,
)
from benchmark.drive_model import DriveModel  # noqa: E402
try:  # Script execution and package import use different module roots.
    import analyze_round5_motion_transitions as round5  # type: ignore  # noqa: E402
except ModuleNotFoundError:  # pragma: no cover - exercised by test collection
    from scripts import analyze_round5_motion_transitions as round5  # type: ignore  # noqa: E402
from training.action_protocol import load_action_protocol  # noqa: E402


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _chain(action_contract: Path) -> ExplicitRuntimeChain:
    protocol = load_action_protocol(action_contract)
    return ExplicitRuntimeChain(
        dt_s=0.02,
        action_minimum=protocol.minimum,
        action_maximum=protocol.maximum,
        max_main_acceleration_rad_s2=0.5,
        max_auxiliary_rate_rad_s=math.radians(30.0),
        max_auxiliary_velocity_rad_s=0.30,
        controls_per_block=protocol.controls_per_block,
    )


def _physical(sample: Mapping[str, object]) -> np.ndarray:
    return round5._state_from_sample(sample)[:6]


def _trace(manifest_item: Mapping[str, object], root: Path) -> tuple[Path, dict[str, object]]:
    path = root / "traces" / f"{manifest_item['sequence_id']}.json"
    return path, _load(path)


def _initial_stop_count(samples: Sequence[Mapping[str, object]], start_index: int) -> int:
    count = 0
    for index in range(start_index, -1, -5):
        speed = float(np.linalg.norm(samples[index]["linear_velocity_world_xy_m_s"]))
        if speed < 0.05:
            count += 1
            if count == 4:
                break
        else:
            break
    return count


def _event_time(values: np.ndarray, target: float, deadband: float, dt_s: float) -> float | None:
    if abs(target) <= 1e-12:
        return None
    signed = values * math.copysign(1.0, target)
    indices = np.flatnonzero(signed > deadband)
    return None if not indices.size else float(indices[0] * dt_s)


def _raw_zero_crossing(values: np.ndarray, target: float, dt_s: float) -> float | None:
    if abs(target) <= 1e-12 or values[0] * target >= 0.0:
        return None
    signed = values * math.copysign(1.0, target)
    indices = np.flatnonzero(signed >= 0.0)
    indices = indices[indices > 0]
    if not indices.size:
        return None
    index = int(indices[0])
    before, after = float(signed[index - 1]), float(signed[index])
    fraction = 1.0 if math.isclose(after, before) else float(np.clip(-before / (after - before), 0.0, 1.0))
    return float((index - 1 + fraction) * dt_s)


def _stop_ready_time(
    physical_state_trace: np.ndarray, *, initial_count: int, dt_s: float
) -> float | None:
    counter = StopReadyCounter(initial_count=initial_count)
    policy_stride = int(round(0.1 / dt_s))
    for index in range(policy_stride, len(physical_state_trace), policy_stride):
        speed = float(np.linalg.norm(physical_state_trace[index, 3:5]))
        if counter.update(speed):
            return float(index * dt_s)
    return None


def _event_bundle(
    state_trace: np.ndarray,
    *,
    target_main: float,
    target_aux: float,
    initial_count: int,
    dt_s: float,
) -> dict[str, object]:
    main_applicable = abs(target_main) > 1e-12 and state_trace[0, 0] * target_main < 0.0
    aux_applicable = abs(target_aux) > 1e-12 and state_trace[0, 1] * target_aux < 0.0
    return {
        "main_applicable": main_applicable,
        "main_new_direction_time_s": (
            _event_time(state_trace[:, 0], target_main, 0.05, dt_s)
            if main_applicable else None
        ),
        "main_raw_zero_crossing_time_s": (
            _raw_zero_crossing(state_trace[:, 0], target_main, dt_s)
            if main_applicable else None
        ),
        "auxiliary_applicable": aux_applicable,
        "auxiliary_new_direction_time_s": (
            _event_time(state_trace[:, 1], target_aux, math.radians(0.5), dt_s)
            if aux_applicable else None
        ),
        "auxiliary_raw_zero_crossing_time_s": (
            _raw_zero_crossing(state_trace[:, 1], target_aux, dt_s)
            if aux_applicable else None
        ),
        "stop_ready_applicable": True,
        "stop_ready_time_s": _stop_ready_time(
            state_trace, initial_count=initial_count, dt_s=dt_s
        ),
        "initial_stop_count": initial_count,
    }


def _m1_rollout(
    trace: Mapping[str, object], model: Mapping[str, object]
) -> tuple[np.ndarray, np.ndarray, int, list[Mapping[str, object]]]:
    samples = round5._trace_samples(trace)
    start, first = round5._transition_boundary(samples)
    future = samples[first:]
    result = round5._rollout_m1(model, samples[start], future)
    return (
        np.asarray(result["state_trace"], dtype=float),
        np.vstack([round5._state_from_sample(item) for item in [samples[start], *future]]),
        start,
        samples,
    )


def _event_matrix_rows(
    *,
    sequence_id: str,
    actual: Mapping[str, object],
    predicted: Mapping[str, object],
    model_name: str,
    available_duration_s: float,
    horizons: Sequence[float],
) -> list[dict[str, object]]:
    rows = []
    for horizon in horizons:
        for label, time_key, applicable_key in (
            ("main_new_direction", "main_new_direction_time_s", "main_applicable"),
            ("auxiliary_new_direction", "auxiliary_new_direction_time_s", "auxiliary_applicable"),
            ("stop_ready", "stop_ready_time_s", "stop_ready_applicable"),
        ):
            comparison = classify_event_pair(
                actual_time_s=actual[time_key],
                predicted_time_s=predicted[time_key],
                applicable=bool(actual[applicable_key]),
                horizon_s=float(horizon),
                available_duration_s=available_duration_s,
            )
            rows.append(
                {
                    "sequence_id": sequence_id,
                    "model": model_name,
                    "event": label,
                    "horizon_s": float(horizon),
                    "actual_event_time_s": actual[time_key],
                    "predicted_event_time_s": predicted[time_key],
                    **comparison,
                }
            )
    return rows


def _execution_events(
    samples: Sequence[Mapping[str, object]], start: int, first: int
) -> dict[str, object]:
    future = samples[first:]
    target_main = float(future[0]["requested_main_rad_s"])
    target_aux = float(future[0]["requested_auxiliary_rad"])
    dt = float(future[0]["time_s"]) - float(samples[start]["time_s"])
    smoothed_main = np.asarray([samples[start]["smoothed_main_rad_s"], *[x["smoothed_main_rad_s"] for x in future]])
    applied_main = np.asarray([samples[start]["final_main_rad_s"], *[x["final_main_rad_s"] for x in future]])
    actual_state = np.vstack([_physical(x) for x in [samples[start], *future]])

    def target_event(values: np.ndarray, target: float, deadband: float) -> float | None:
        if abs(target) <= 1e-12:
            indices = np.flatnonzero(np.abs(values) <= deadband)
            return None if not indices.size else float(indices[0] * dt)
        return _event_time(values, target, deadband, dt)

    return {
        "main_request_change_time_s": 0.0,
        "main_smoothed_target_event_time_s": target_event(smoothed_main, target_main, 0.02),
        "main_applied_target_event_time_s": target_event(applied_main, target_main, 0.02),
        "main_measured_new_direction_time_s": _event_time(actual_state[:, 0], target_main, 0.05, dt),
        "auxiliary_measured_new_direction_time_s": _event_time(actual_state[:, 1], target_aux, math.radians(0.5), dt),
        "first_planar_speed_below_0p05_time_s": next(
            (
                float(i * dt)
                for i, state in enumerate(actual_state)
                if np.linalg.norm(state[3:5]) < 0.05
            ),
            None,
        ),
        "stop_ready_time_s": _stop_ready_time(
            actual_state, initial_count=_initial_stop_count(samples, start), dt_s=dt
        ),
        "b0_latch_switch": "not_applicable_probe_has_no_b0_state_machine",
        "start_smoothed_main_rad_s": float(smoothed_main[0]),
        "target_zero_time_scale_s": abs(float(smoothed_main[0])) / 0.5,
    }


def _support_check(
    points: np.ndarray,
    maximum_distance: float,
    state_trace: np.ndarray,
    initial_applied: np.ndarray,
    applied_trace: np.ndarray,
    requests: np.ndarray,
) -> dict[str, object]:
    center, scale = points.mean(axis=0), points.std(axis=0)
    scale[scale < 1e-6] = 1.0
    lower, upper = points.min(axis=0), points.max(axis=0)
    checks = []
    for index, (state, request) in enumerate(zip(state_trace[:-1], requests)):
        previous = initial_applied if index == 0 else applied_trace[index - 1]
        query = np.concatenate((state, previous, request))
        inside = bool(np.all(query >= lower - 1e-9) and np.all(query <= upper + 1e-9))
        distance = float(np.min(np.linalg.norm((points - query) / scale, axis=1)))
        checks.append((inside, distance))
    unsupported = [i for i, (inside, distance) in enumerate(checks) if not inside or distance > maximum_distance]
    return {
        "supported": not unsupported,
        "checked_tick_count": len(checks),
        "first_unsupported_tick": unsupported[0] if unsupported else None,
        "all_inside_feature_box": all(item[0] for item in checks),
        "maximum_standardized_nearest_neighbor_distance": max(item[1] for item in checks),
        "threshold": maximum_distance,
    }


def _evaluate_m2_trace(
    *,
    trace_path: Path,
    trace: Mapping[str, object],
    model: Mapping[str, object],
    chain: ExplicitRuntimeChain,
    support_points: np.ndarray,
    support_distance: float,
    horizons: Sequence[float],
    dataset_label: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    samples = round5._trace_samples(trace)
    start, first = round5._transition_boundary(samples)
    future_all = samples[first:]
    dt = float(model["dt_s"])
    available = min(6.0, len(future_all) * dt)
    tick_count = min(len(future_all), int(round(available / dt)))
    future = future_all[:tick_count]
    requests = np.asarray(
        [[item["requested_main_rad_s"], item["requested_auxiliary_rad"]] for item in future],
        dtype=float,
    )
    initial_applied = np.asarray(
        [samples[start]["final_main_rad_s"], samples[start]["final_auxiliary_rad"]], dtype=float
    )
    execution = chain.clone()
    execution.reset(
        applied_action=initial_applied,
        last_smoothed_target=(
            samples[start]["smoothed_main_rad_s"], samples[start]["smoothed_auxiliary_rad"]
        ),
        tick_phase=(start + 1) % 5,
    )
    predictor = M2PhysicalPredictor(model, execution)
    rollout = predictor.rollout(
        current_physical_state=_physical(samples[start]),
        execution_snapshot=execution.snapshot(),
        future_protocol_requests=requests,
        start_yaw_rad=float(samples[start]["navigation_heading_rad"]),
    )
    actual_state = np.vstack([_physical(item) for item in [samples[start], *future]])
    actual_position = np.asarray([item["position_xy_m"] for item in [samples[start], *future]], dtype=float)
    actual_position -= actual_position[0]
    start_yaw = float(samples[start]["navigation_heading_rad"])
    actual_local = np.vstack([round5._rotate_world_to_local(item, start_yaw) for item in actual_position])
    actual_smoothed = np.asarray(
        [[item["smoothed_main_rad_s"], item["smoothed_auxiliary_rad"]] for item in future]
    )
    actual_applied = np.asarray(
        [[item["final_main_rad_s"], item["final_auxiliary_rad"]] for item in future]
    )
    actual_supervisor = np.asarray(
        [item["auxiliary_velocity_supervisor_active"] for item in future], dtype=bool
    )
    initial_count = _initial_stop_count(samples, start)
    target_main, target_aux = requests[-1]
    actual_events = _event_bundle(
        actual_state, target_main=target_main, target_aux=target_aux,
        initial_count=initial_count, dt_s=dt,
    )
    predicted_events = _event_bundle(
        rollout.physical_state_trace, target_main=target_main, target_aux=target_aux,
        initial_count=initial_count, dt_s=dt,
    )
    sequence_id = str(trace["sequence"]["sequence_id"])
    event_rows = _event_matrix_rows(
        sequence_id=sequence_id, actual=actual_events, predicted=predicted_events,
        model_name="M2", available_duration_s=available, horizons=horizons,
    )
    support = _support_check(
        support_points, support_distance, rollout.physical_state_trace,
        initial_applied, rollout.applied_target_trace, requests,
    )
    rows = []
    for horizon in horizons:
        end = min(tick_count, int(round(horizon / dt)))
        trajectory_error = np.linalg.norm(
            rollout.local_trajectory_xy_m[: end + 1] - actual_local[: end + 1], axis=1
        )
        state_error = rollout.physical_state_trace[end] - actual_state[end]
        smoothed_error = rollout.smoothed_target_trace[:end] - actual_smoothed[:end]
        applied_error = rollout.applied_target_trace[:end] - actual_applied[:end]
        rows.append(
            {
                "sequence_id": sequence_id,
                "dataset_label": dataset_label,
                "trace_path": str(trace_path.resolve()),
                "horizon_s": float(horizon),
                "trajectory_rms_error_m": float(np.sqrt(np.mean(np.square(trajectory_error)))),
                "trajectory_peak_error_m": float(np.max(trajectory_error)),
                "terminal_physical_state_error": dict(zip(PHYSICAL_STATE_NAMES, state_error.tolist())),
                "terminal_planar_speed_error_m_s": float(
                    np.linalg.norm(rollout.physical_state_trace[end, 3:5])
                    - np.linalg.norm(actual_state[end, 3:5])
                ),
                "terminal_heading_error_rad": float(
                    round5._angle_wrap(
                        rollout.heading_trace_rad[end]
                        - float([samples[start], *future][end]["navigation_heading_rad"])
                    )
                ),
                "smoothed_target_error_max_abs": np.max(np.abs(smoothed_error), axis=0).tolist(),
                "applied_target_error_max_abs": np.max(np.abs(applied_error), axis=0).tolist(),
                "supervisor_mismatch_count": int(
                    np.count_nonzero(rollout.supervisor_active_trace[:end] != actual_supervisor[:end])
                ),
                "support": support,
                "recursive_prediction_finite": True,
                "output_clipping_used": False,
            }
        )
    return rows, event_rows


def _old_model_event_rows(
    *, manifest: Mapping[str, object], root: Path, model: Mapping[str, object], model_name: str,
    horizons: Sequence[float],
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    events, execution_rows = [], []
    for item in manifest["sequences"]:
        if item["dataset_role"] != "heldout_validation":
            continue
        _, trace = _trace(item, root)
        predicted, actual8, start, samples = _m1_rollout(trace, model)
        first = start + 1
        target_main = float(samples[first]["requested_main_rad_s"])
        target_aux = float(samples[first]["requested_auxiliary_rad"])
        count = _initial_stop_count(samples, start)
        actual = _event_bundle(actual8[:, :6], target_main=target_main, target_aux=target_aux, initial_count=count, dt_s=0.02)
        forecast = _event_bundle(predicted[:, :6], target_main=target_main, target_aux=target_aux, initial_count=count, dt_s=0.02)
        sequence_id = str(item["sequence_id"])
        events.extend(_event_matrix_rows(
            sequence_id=sequence_id, actual=actual, predicted=forecast,
            model_name=model_name, available_duration_s=min(6.0, (len(predicted)-1)*0.02), horizons=horizons,
        ))
        execution_rows.append({
            "sequence_id": sequence_id,
            **_execution_events(samples, start, first),
            "actual_main_raw_zero_crossing_time_s": actual["main_raw_zero_crossing_time_s"],
            "predicted_main_raw_zero_crossing_time_s": forecast["main_raw_zero_crossing_time_s"],
            "actual_auxiliary_raw_zero_crossing_time_s": actual["auxiliary_raw_zero_crossing_time_s"],
            "predicted_auxiliary_raw_zero_crossing_time_s": forecast["auxiliary_raw_zero_crossing_time_s"],
        })
    return events, execution_rows


def _event_gate_summary(
    rows: Sequence[Mapping[str, object]], *, minimum: int, maximum_mae: float
) -> tuple[bool, dict[str, object]]:
    observable = [row for row in rows if row["actual_status"] == "observed_within_horizon"]
    comparable = [row for row in rows if row["comparison"] == "correct_detection"]
    errors = [abs(float(row["timing_error_s"])) for row in comparable]
    mae = float(np.mean(errors)) if errors else None
    passed = (
        len(observable) >= minimum
        and len(comparable) == len(observable)
        and mae is not None
        and mae <= maximum_mae
    )
    return passed, {
        "observable_count": len(observable),
        "comparable_count": len(comparable),
        "false_positive_count": sum(row["comparison"] == "false_positive" for row in rows),
        "missed_count": sum(row["comparison"] == "missed_event" for row in rows),
        "mae_s": mae,
        "maximum_error_s": max(errors) if errors else None,
    }


def _decision(
    fresh_rows: Sequence[Mapping[str, object]],
    fresh_events: Sequence[Mapping[str, object]],
    gates: Mapping[str, object],
) -> dict[str, object]:
    rows_1p5 = [row for row in fresh_rows if math.isclose(float(row["horizon_s"]), 1.5)]
    main_error = [abs(float(row["terminal_physical_state_error"]["main_joint_velocity_rad_s"])) for row in rows_1p5]
    aux_error = [abs(float(row["terminal_physical_state_error"]["auxiliary_joint_rad"])) for row in rows_1p5]
    trajectory = [float(row["trajectory_rms_error_m"]) for row in rows_1p5]
    primary = [row for row in fresh_events if math.isclose(float(row["horizon_s"]), 3.0)]
    stop = [row for row in primary if row["event"] == "stop_ready"]
    stop_positive = [row for row in stop if row["actual_status"] == "observed_within_horizon"]
    stop_negative = [row for row in stop if row["actual_status"] == "not_observed_by_horizon"]
    stop_comparable = [row for row in stop if row["comparison"] == "correct_detection"]
    stop_errors = [abs(float(row["timing_error_s"])) for row in stop_comparable]
    stop_early = [max(0.0, -float(row["timing_error_s"])) for row in stop_comparable]
    false_positive = sum(row["comparison"] == "false_positive" for row in stop)
    longest = [row for row in fresh_events if math.isclose(float(row["horizon_s"]), 6.0)]

    main_pass, main_summary = _event_gate_summary(
        [row for row in longest if row["event"] == "main_new_direction"],
        minimum=int(gates["minimum_observable_main_event_sequences"]),
        maximum_mae=float(gates["maximum_main_event_mae_s"]),
    )
    aux_pass, aux_summary = _event_gate_summary(
        [row for row in longest if row["event"] == "auxiliary_new_direction"],
        minimum=int(gates["minimum_observable_auxiliary_event_sequences"]),
        maximum_mae=float(gates["maximum_auxiliary_event_mae_s"]),
    )
    checks = {
        "minimum_fresh_complete_sequences": len({row["sequence_id"] for row in rows_1p5}) >= int(gates["minimum_fresh_complete_sequences"]),
        "maximum_trajectory_rms_error_m_at_1p5s": max(trajectory) <= float(gates["maximum_trajectory_rms_error_m_at_1p5s"]),
        "maximum_terminal_main_error_rad_s_at_1p5s": max(main_error) <= float(gates["maximum_terminal_main_error_rad_s_at_1p5s"]),
        "maximum_terminal_auxiliary_angle_error_rad_at_1p5s": max(aux_error) <= float(gates["maximum_terminal_auxiliary_angle_error_rad_at_1p5s"]),
        "main_event_gate": main_pass,
        "auxiliary_event_gate": aux_pass,
        "minimum_stop_ready_positive_sequences": len(stop_positive) >= int(gates["minimum_stop_ready_positive_sequences"]),
        "minimum_stop_ready_not_observed_sequences": len(stop_negative) >= int(gates["minimum_stop_ready_not_observed_sequences"]),
        "stop_ready_all_positive_events_detected": len(stop_comparable) == len(stop_positive),
        "stop_ready_maximum_mae_s": bool(stop_errors) and float(np.mean(stop_errors)) <= float(gates["stop_ready_maximum_mae_s"]),
        "stop_ready_maximum_early_error_s": bool(stop_early) and max(stop_early) <= float(gates["stop_ready_maximum_early_error_s"]),
        "stop_ready_no_false_positives": false_positive <= int(gates["stop_ready_false_positives_allowed"]),
        "all_recursive_rollouts_supported": all(bool(row["support"]["supported"]) for row in fresh_rows),
    }
    return {
        "metrics": {
            "trajectory_rms_max_m_at_1p5s": max(trajectory),
            "terminal_main_error_max_rad_s_at_1p5s": max(main_error),
            "terminal_auxiliary_error_max_rad_at_1p5s": max(aux_error),
            "stop_ready_positive_count_at_3s": len(stop_positive),
            "stop_ready_not_observed_count_at_3s": len(stop_negative),
            "stop_ready_comparable_count_at_3s": len(stop_comparable),
            "stop_ready_mae_s": float(np.mean(stop_errors)) if stop_errors else None,
            "stop_ready_maximum_early_error_s": max(stop_early) if stop_early else None,
            "stop_ready_false_positive_count": false_positive,
            "main_event_at_6s": main_summary,
            "auxiliary_event_at_6s": aux_summary,
        },
        "checks": checks,
        "accepted": all(checks.values()),
        "decision": "accepted_local_m2" if all(checks.values()) else "rejected",
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    contract = _load(args.round7_contract)
    horizons = tuple(float(item) for item in contract["events"]["horizons_s"])
    round6_manifest = _load(args.round6_manifest)
    fresh_manifest = _load(args.fresh_manifest)
    old_m1, expanded_m1, m2 = _load(args.old_m1), _load(args.expanded_m1), _load(args.m2)
    old_events, execution_rows = _old_model_event_rows(
        manifest=round6_manifest, root=args.round6_root, model=old_m1,
        model_name="frozen_round5_m1", horizons=horizons,
    )
    expanded_events, _ = _old_model_event_rows(
        manifest=round6_manifest, root=args.round6_root, model=expanded_m1,
        model_name="round6_expanded_m1", horizons=horizons,
    )
    with np.load(args.support_cloud, allow_pickle=False) as archive:
        support_points = np.asarray(archive["state_execution_request_points"], dtype=float)
        support_distance = float(archive["maximum_standardized_nearest_neighbor_distance"])
    chain = _chain(args.action_contract)
    historical_rows, historical_m2_events = [], []
    for item in round6_manifest["sequences"]:
        if item["dataset_role"] != "heldout_validation":
            continue
        path, trace = _trace(item, args.round6_root)
        rows, events = _evaluate_m2_trace(
            trace_path=path, trace=trace, model=m2, chain=chain,
            support_points=support_points, support_distance=support_distance,
            horizons=horizons, dataset_label="round6_seen_historical_regression",
        )
        historical_rows.extend(rows)
        historical_m2_events.extend(events)
    fresh_rows, fresh_events = [], []
    for item in fresh_manifest["sequences"]:
        path, trace = _trace(item, args.fresh_root)
        rows, events = _evaluate_m2_trace(
            trace_path=path, trace=trace, model=m2, chain=chain,
            support_points=support_points, support_distance=support_distance,
            horizons=horizons, dataset_label="round7_fresh_whole_sequence_validation",
        )
        fresh_rows.extend(rows)
        fresh_events.extend(events)
    decision = _decision(fresh_rows, fresh_events, contract["validation_gates"])
    result = {
        "schema_version": "round7_stop_events_and_m2_validation_v1",
        "round6_decision_preserved": "rejected_terminal_state_gates_unchanged",
        "event_definition_version": "round7_observation_censoring_v1",
        "round6_heldout_execution_event_audit": execution_rows,
        "round6_event_matrix": [*old_events, *expanded_events, *historical_m2_events],
        "m2_historical_regression_rows": historical_rows,
        "m2_fresh_validation_rows": fresh_rows,
        "m2_fresh_event_matrix": fresh_events,
        "m2_candidate_decision": decision,
        "model_identity": {
            "path": str(args.m2.resolve()), "sha256": _sha256(args.m2),
            "fit_completed_before_fresh_collection": True,
            "fresh_data_used_for_fit": False,
        },
        "fresh_identity": {
            "manifest_path": str(args.fresh_manifest.resolve()),
            "manifest_sha256": _sha256(args.fresh_manifest),
            "root": str(args.fresh_root.resolve()),
            "complete_sequence_count": len(fresh_manifest["sequences"]),
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    _atomic_json(args.output_dir / "round7_model_and_event_validation.json", result)
    for name, rows in (
        ("round6_event_matrix.csv", result["round6_event_matrix"]),
        ("m2_fresh_event_matrix.csv", fresh_events),
        ("m2_fresh_validation_rows.csv", fresh_rows),
    ):
        with (args.output_dir / name).open("w", encoding="utf-8", newline="") as handle:
            flattened = []
            for row in rows:
                flattened.append({key: json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value for key, value in row.items()})
            writer = csv.DictWriter(handle, fieldnames=list(flattened[0]))
            writer.writeheader()
            writer.writerows(flattened)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-contract", type=Path, required=True)
    parser.add_argument("--round7-contract", type=Path, required=True)
    parser.add_argument("--round6-manifest", type=Path, required=True)
    parser.add_argument("--round6-root", type=Path, required=True)
    parser.add_argument("--fresh-manifest", type=Path, required=True)
    parser.add_argument("--fresh-root", type=Path, required=True)
    parser.add_argument("--old-m1", type=Path, required=True)
    parser.add_argument("--expanded-m1", type=Path, required=True)
    parser.add_argument("--m2", type=Path, required=True)
    parser.add_argument("--support-cloud", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    result = run(_parse_args(argv))
    print(json.dumps(result["m2_candidate_decision"], indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
