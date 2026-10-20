#!/usr/bin/env python3
"""Create the reproducible Round 4 response and frozen-reference audit."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from benchmark.drive_model import DriveModel  # noqa: E402
from benchmark.tracking_acceptance import polyline_cross_track_errors  # noqa: E402
from probe_drive_dynamics import estimate_geometric_curvature  # noqa: E402


def _load_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _first_time(
    samples: Sequence[Mapping[str, object]],
    *,
    start_index: int,
    value_name: str,
    target: float,
    fraction: float,
) -> float | None:
    threshold = abs(target) * fraction
    if threshold <= 1e-12:
        return 0.0
    start_time = float(samples[start_index]["time_s"])
    for sample in samples[start_index:]:
        value = float(sample[value_name])
        if np.sign(value) == np.sign(target) and abs(value) >= threshold:
            return float(sample["time_s"]) - start_time
    return None


def _response_row(
    case: Mapping[str, object], trace: Mapping[str, object], model: DriveModel
) -> dict[str, object]:
    samples = trace["samples"]
    if not isinstance(samples, list) or not samples:
        raise ValueError("trace samples are required")
    analysis = case["round4_analysis"]
    if not isinstance(analysis, Mapping):
        raise ValueError("Round 4 analysis is required")
    main = float(case["main_command_rad_s"])
    auxiliary = float(case["auxiliary_command_rad"])
    steady_aux = float(analysis["steady_auxiliary_joint_rad"])
    steady_speed = float(analysis["steady_speed_m_s"])
    steady_yaw_rate = float(analysis["steady_body_yaw_rate_rad_s"])
    main_90 = _first_time(
        samples,
        start_index=0,
        value_name="main_joint_velocity_rad_s",
        target=main,
        fraction=0.9,
    )
    body_speed_90 = None
    for sample in samples:
        speed_value = float(np.linalg.norm(sample["linear_velocity_world_xy_m_s"]))
        if speed_value >= 0.9 * steady_speed:
            body_speed_90 = float(sample["time_s"]) - float(samples[0]["time_s"])
            break
    steer_indices = [
        index for index, sample in enumerate(samples) if sample["phase"] == "steer"
    ]
    release_indices = [
        index for index, sample in enumerate(samples) if sample["phase"] == "release"
    ]
    aux_90 = None
    turn_onset = None
    yaw_overshoot = None
    release_aux_residual = None
    release_yaw_residual = None
    geometry_2s: dict[str, object] | None = None
    if steer_indices:
        start = steer_indices[0]
        aux_90 = _first_time(
            samples,
            start_index=start,
            value_name="auxiliary_joint_rad",
            target=auxiliary,
            fraction=0.9,
        )
        steady_abs_yaw = abs(steady_yaw_rate)
        if steady_abs_yaw > 1e-5:
            start_time = float(samples[start]["time_s"])
            desired_sign = np.sign(steady_yaw_rate)
            for index in steer_indices:
                value = float(samples[index]["body_yaw_rate_rad_s"])
                if np.sign(value) == desired_sign and abs(value) >= 0.1 * steady_abs_yaw:
                    turn_onset = float(samples[index]["time_s"]) - start_time
                    break
            peak = max(abs(float(samples[index]["body_yaw_rate_rad_s"])) for index in steer_indices)
            yaw_overshoot = peak / steady_abs_yaw - 1.0
        end_time = float(samples[steer_indices[-1]]["time_s"])
        window = [
            samples[index]
            for index in steer_indices
            if float(samples[index]["time_s"]) >= end_time - 2.0
        ]
        geometry_2s = estimate_geometric_curvature(
            [sample["position_xy_m"] for sample in window],
            minimum_path_length_m=0.15,
        )
        geometry_2s["diagnostic_only"] = True
        geometry_2s["window_s"] = 2.0
    if release_indices:
        end_time = float(samples[release_indices[-1]]["time_s"])
        last = [
            samples[index]
            for index in release_indices
            if float(samples[index]["time_s"]) >= end_time - 0.5
        ]
        release_aux_residual = float(
            np.median(np.abs([float(sample["auxiliary_joint_rad"]) for sample in last]))
        )
        release_yaw_residual = float(
            np.median(np.abs([float(sample["body_yaw_rate_rad_s"]) for sample in last]))
        )
    predicted_speed = model.speed(main, steady_aux)
    predicted_curvature = model.curvature(main, steady_aux)
    actual_curvature = analysis["steady_curvature_1_m"]
    return {
        "case_id": case["case_id"],
        "dataset_role": case["dataset_role"],
        "requested_main_rad_s": main,
        "requested_auxiliary_rad": auxiliary,
        "actual_main_rad_s": float(analysis["steady_main_joint_velocity_rad_s"]),
        "actual_auxiliary_rad": steady_aux,
        "actual_auxiliary_error_rad": float(analysis["steady_auxiliary_error_rad"]),
        "actual_auxiliary_p95_abs_velocity_rad_s": float(
            analysis["steady_auxiliary_p95_abs_velocity_rad_s"]
        ),
        "steady_speed_m_s": steady_speed,
        "steady_body_yaw_rate_rad_s": steady_yaw_rate,
        "yaw_rate_over_speed_proxy_1_m": (
            steady_yaw_rate / steady_speed if steady_speed >= 0.05 else None
        ),
        "steady_xy_curvature_1_m": actual_curvature,
        "steady_state_valid": bool(analysis["steady_state_valid"]),
        "invalid_reasons": list(analysis["invalid_reasons"]),
        "xy_curvature_2s_diagnostic": geometry_2s,
        "main_joint_time_to_90_percent_s": main_90,
        "body_speed_time_to_90_percent_s": body_speed_90,
        "auxiliary_time_to_90_percent_s": aux_90,
        "body_turn_onset_10_percent_s": turn_onset,
        "body_yaw_rate_overshoot_fraction": yaw_overshoot,
        "release_auxiliary_residual_rad": release_aux_residual,
        "release_abs_yaw_rate_residual_rad_s": release_yaw_residual,
        "angle_limit_fraction": float(
            np.mean([bool(sample["auxiliary_command_at_angle_limit"]) for sample in samples])
        ),
        "tracking_deficient_fraction": float(
            np.mean([bool(sample["auxiliary_tracking_deficient"]) for sample in samples])
        ),
        "velocity_supervisor_fraction": float(
            np.mean(
                [bool(sample["auxiliary_velocity_supervisor_active"]) for sample in samples]
            )
        ),
        "v1_predicted_speed_m_s": predicted_speed,
        "v1_predicted_curvature_1_m": predicted_curvature,
        "v1_speed_error_m_s": predicted_speed - steady_speed,
        "v1_xy_curvature_error_1_m": (
            None if actual_curvature is None else predicted_curvature - float(actual_curvature)
        ),
        "v1_proxy_curvature_error_1_m": (
            None
            if steady_speed < 0.05
            else predicted_curvature - steady_yaw_rate / steady_speed
        ),
        "v1_main_query_behavior": (
            "speed interpolates from synthetic zero; curvature and inverse clamp to 0.5-rad/s endpoint"
            if main < 0.5
            else "within measured main-command support"
        ),
    }


def _first_true(value: np.ndarray) -> int | None:
    indices = np.flatnonzero(value)
    return int(indices[0]) if indices.size else None


def _reference_audit(
    episode_id: str, roots: Mapping[str, Path], response_rows: Sequence[Mapping[str, object]]
) -> dict[str, object]:
    policies: dict[str, object] = {}
    finite_aux_delays = [
        float(row["auxiliary_time_to_90_percent_s"])
        for row in response_rows
        if row["auxiliary_time_to_90_percent_s"] is not None
        and row["dataset_role"] == "fit_core"
    ]
    measured_response = float(np.median(finite_aux_delays)) if finite_aux_delays else None
    for label, root in roots.items():
        path = root / f"episode_{episode_id}" / "states.npz"
        with np.load(path, allow_pickle=False) as data:
            arrays = {name: np.asarray(data[name]) for name in data.files}
        contact_index = _first_true(arrays["physical_contact"])
        collision_index = _first_true(arrays["collision"])
        action_limit = arrays["policy_reference_index"].shape[0]
        if contact_index is not None:
            action_limit = min(action_limit, max(contact_index, 1))
        reference_index = arrays["policy_reference_index"][:action_limit].astype(int)
        boundaries = [0]
        boundaries.extend(
            index
            for index in range(1, action_limit)
            if reference_index[index] != reference_index[index - 1]
        )
        boundaries.append(action_limit)
        segments: list[dict[str, object]] = []
        reference_pose = arrays["kinodynamic_reference_pose"]
        reference_command = arrays["kinodynamic_reference_command"]
        actual_aux = arrays["joint_position_rad"][1 : action_limit + 1, 1]
        actual_aux_velocity = arrays["joint_velocity_rad_s"][1 : action_limit + 1, 1]
        for start, end in zip(boundaries, boundaries[1:]):
            index = int(reference_index[start])
            dwell_s = 0.1 * (end - start)
            baseline_main = float(
                np.median(arrays["policy_baseline_main_command_rad_s"][start:end])
            )
            scheduled_main = float(
                np.median(arrays["policy_scheduled_main_command_rad_s"][start:end])
            )
            auxiliary_target = float(
                np.median(arrays["policy_auxiliary_target_angle_rad"][start:end])
            )
            segments.append(
                {
                    "reference_index": index,
                    "action_start": start,
                    "action_end_exclusive": end,
                    "dwell_s": dwell_s,
                    "reference_position_xy_m": reference_pose[index, :2].tolist(),
                    "reference_heading_rad": float(reference_pose[index, 2]),
                    "reference_main_command_rad_s": float(reference_command[index, 0]),
                    "reference_auxiliary_angle_rad": float(reference_command[index, 1]),
                    "policy_reference_curvature_1_m": float(
                        np.median(arrays["policy_reference_curvature_1_m"][start:end])
                    ),
                    "baseline_main_command_rad_s": baseline_main,
                    "scheduled_main_command_rad_s": scheduled_main,
                    "auxiliary_target_rad": auxiliary_target,
                    "actual_auxiliary_rad": float(np.median(actual_aux[start:end])),
                    "actual_auxiliary_velocity_rad_s": float(
                        np.median(actual_aux_velocity[start:end])
                    ),
                    "auxiliary_velocity_supervisor_active_fraction": float(
                        np.mean(np.abs(actual_aux_velocity[start:end]) > 0.30)
                    ),
                    "main_query_support": (
                        "below_v1_curvature_support_endpoint_clamped"
                        if 0.0 < abs(scheduled_main) < 0.5
                        else "within_v1_measured_main_support"
                    ),
                    "switch_faster_than_median_measured_aux_response": (
                        None if measured_response is None else dwell_s < measured_response
                    ),
                    "reference_progress_start_m": float(
                        arrays["policy_reference_progress_s_m"][start]
                    ),
                    "reference_progress_end_m": float(
                        arrays["policy_reference_progress_s_m"][end - 1]
                    ),
                }
            )
        policies[label] = {
            "states_path": str(path.resolve()),
            "first_physical_contact_state_index": contact_index,
            "first_formal_collision_state_index": collision_index,
            "precontact_action_count": action_limit,
            "segments": segments,
        }
    return {
        "episode_id": episode_id,
        "reference_progress_mechanism": "causal monotonic spatial projection within active direction segment",
        "median_measured_auxiliary_90_percent_response_s": measured_response,
        "policies": policies,
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    keys = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: json.dumps(value, ensure_ascii=False)
                    if isinstance(value, (dict, list))
                    else value
                    for key, value in row.items()
                }
            )


def _derived_trace(samples: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    positions = np.asarray([sample["position_xy_m"] for sample in samples], dtype=np.float64)
    velocity = np.asarray(
        [sample["linear_velocity_world_xy_m_s"] for sample in samples], dtype=np.float64
    )
    speed = np.linalg.norm(velocity, axis=1)
    curvature: list[float | None] = [None] * len(samples)
    invalid_reason: list[str | None] = ["edge_sample"] * len(samples)
    for index in range(1, len(samples) - 1):
        first = positions[index] - positions[index - 1]
        second = positions[index + 1] - positions[index]
        first_length = float(np.linalg.norm(first))
        second_length = float(np.linalg.norm(second))
        chord = float(np.linalg.norm(first + second))
        if speed[index] < 0.05:
            invalid_reason[index] = "low_planar_speed"
            continue
        denominator = first_length * second_length * chord
        if denominator <= 1e-10:
            invalid_reason[index] = "degenerate_local_geometry"
            continue
        cross = float(first[0] * second[1] - first[1] * second[0])
        value = 2.0 * cross / denominator
        if not math.isfinite(value):
            invalid_reason[index] = "nonfinite_local_geometry"
            continue
        curvature[index] = value
        invalid_reason[index] = None
    return [
        {
            "time_s": float(sample["time_s"]),
            "phase": sample["phase"],
            "planar_speed_m_s": float(speed[index]),
            "trajectory_curvature_1_m": curvature[index],
            "trajectory_curvature_invalid_reason": invalid_reason[index],
            "source": "three_point_xy_geometry_offline_noncausal",
        }
        for index, sample in enumerate(samples)
    ]


def _render_plots(
    output: Path,
    response_rows: Sequence[Mapping[str, object]],
    calibration_root: Path,
    fixed_root: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(12, 9))
    target_angles = (-15.0, 15.0, -30.0, 30.0)
    for axis, angle in zip(axes.flat, target_angles):
        for main, color in ((0.35, "tab:blue"), (0.5, "tab:orange")):
            match = next(
                row
                for row in response_rows
                if row["dataset_role"] == "fit_core"
                and math.isclose(float(row["requested_main_rad_s"]), main)
                and math.isclose(
                    math.degrees(float(row["requested_auxiliary_rad"])), angle
                )
            )
            trace = _load_json(calibration_root / "traces" / f"{match['case_id']}.json")
            samples = [item for item in trace["samples"] if item["phase"] == "steer"]
            xy = np.asarray([item["position_xy_m"] for item in samples])
            axis.plot(xy[:, 0], xy[:, 1], color=color, label=f"main={main:.2f}")
        axis.set_title(f"requested auxiliary {angle:+.0f}°")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=0.25)
        axis.legend()
    figure.suptitle("Round 4 open-plane steering trajectories")
    figure.tight_layout()
    figure.savefig(output / "round4_calibration_trajectories.png", dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    for episode, color in (("hard_0_e1_0002", "tab:blue"), ("hard_0_e1_0004", "tab:green")):
        with np.load(fixed_root / f"episode_{episode}" / "states.npz") as data:
            actual = data["position_world_xy"]
            reference = data["kinodynamic_reference_pose"][:, :2]
        axes[0].plot(reference[:, 0], reference[:, 1], "--", color=color, label=f"{episode} ref")
        axes[0].plot(actual[:, 0], actual[:, 1], color=color, label=f"{episode} actual")
        cross = polyline_cross_track_errors(actual, reference)
        axes[1].plot(np.arange(cross.size) * 0.1, cross, color=color, label=episode)
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title("Frozen reference and actual path")
    axes[0].grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    axes[1].set_title("Cross-track error")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("error (m)")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "round4_fixed_reference_tracking.png", dpi=160)
    plt.close(figure)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-root", type=Path, required=True)
    parser.add_argument("--b0-root", type=Path, required=True)
    parser.add_argument("--r3-root", type=Path, required=True)
    parser.add_argument("--fixed-root", type=Path, required=True)
    parser.add_argument("--drive-profile", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    report = _load_json(args.calibration_root / "drive_dynamics_calibration.json")
    model = DriveModel.from_json(args.drive_profile)
    response_rows = []
    for case in report["cases"]:
        trace = _load_json(args.calibration_root / "traces" / f"{case['case_id']}.json")
        response_rows.append(_response_row(case, trace, model))
        _atomic_json(
            args.out / "derived_traces" / f"{case['case_id']}.json",
            {
                "schema_version": "round4_derived_trace_v1",
                "raw_trace_path": str(
                    (args.calibration_root / "traces" / f"{case['case_id']}.json").resolve()
                ),
                "samples": _derived_trace(trace["samples"]),
            },
        )
    audits = [
        _reference_audit(
            episode,
            {"B0": args.b0_root, "R3": args.r3_root},
            response_rows,
        )
        for episode in ("hard_0_e1_0002", "hard_0_e1_0004")
    ]
    fixed_metrics = {
        episode: _load_json(args.fixed_root / f"episode_{episode}" / "metrics.json")
        for episode in ("hard_0_e1_0002", "hard_0_e1_0004")
    }
    heldout_valid = [
        row
        for row in response_rows
        if row["dataset_role"] == "heldout_validation"
        and row["steady_state_valid"]
    ]
    candidate_decision = {
        "formed_candidate": False,
        "passed_mapping_gate": False,
        "valid_heldout_cases": len(heldout_valid),
        "required_valid_heldout_cases": 3,
        "reason": "insufficient valid held-out geometry; fitting would violate the preregistered split/gate",
        "required_statement": "完成标定，但未形成可用映射。",
    }
    output = {
        "schema_version": "round4_drive_and_reference_analysis_v1",
        "response_rows": response_rows,
        "candidate_decision": candidate_decision,
        "reference_audits": audits,
        "fixed_reference_metrics": fixed_metrics,
    }
    _atomic_json(args.out / "round4_analysis.json", output)
    _write_csv(args.out / "round4_response_table.csv", response_rows)
    for audit in audits:
        rows = []
        for policy, payload in audit["policies"].items():
            rows.extend({"policy": policy, **row} for row in payload["segments"])
        _write_csv(args.out / f"reference_audit_{audit['episode_id']}.csv", rows)
    _render_plots(args.out, response_rows, args.calibration_root, args.fixed_root)
    print(args.out / "round4_analysis.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
