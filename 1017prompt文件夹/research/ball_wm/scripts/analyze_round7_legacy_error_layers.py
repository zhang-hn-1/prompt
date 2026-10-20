#!/usr/bin/env python3
"""Localize frozen M1 errors by transition class without refitting either model."""

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

try:
    import analyze_round5_motion_transitions as round5  # type: ignore
except ModuleNotFoundError:  # pragma: no cover
    from scripts import analyze_round5_motion_transitions as round5  # type: ignore


def _load(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _transition_group(initial: np.ndarray, target: np.ndarray) -> str:
    if abs(float(target[0])) <= 1e-12:
        return "braking_to_zero"
    main_reversal = initial[0] * target[0] < 0.0
    auxiliary_reversal = abs(target[1]) > 1e-12 and initial[1] * target[1] < 0.0
    if main_reversal and auxiliary_reversal:
        return "joint_main_and_auxiliary_reversal"
    if main_reversal:
        return "main_reversal"
    if auxiliary_reversal:
        return "auxiliary_reversal"
    return "same_direction_tracking"


def _row(
    trace_path: Path,
    trace: Mapping[str, object],
    model: Mapping[str, object],
    model_name: str,
    dataset: str,
) -> dict[str, object]:
    samples = round5._trace_samples(trace)
    start, first = round5._transition_boundary(samples)
    count = min(75, len(samples) - first)
    future = samples[first : first + count]
    actual = np.vstack([round5._state_from_sample(item) for item in [samples[start], *future]])
    result = round5._rollout_m1(model, samples[start], future)
    predicted = np.asarray(result["state_trace"], dtype=float)
    initial = actual[0]
    target = np.asarray([future[0]["requested_main_rad_s"], future[0]["requested_auxiliary_rad"]])
    one_step = predicted[1] - actual[1]
    terminal = predicted[-1] - actual[-1]
    actual_position = np.asarray([item["position_xy_m"] for item in [samples[start], *future]], dtype=float)
    actual_position -= actual_position[0]
    start_yaw = float(samples[start]["navigation_heading_rad"])
    actual_local = np.vstack([round5._rotate_world_to_local(point, start_yaw) for point in actual_position])
    predicted_world = np.asarray(result["world_trajectory_delta_xy_m"], dtype=float)
    predicted_local = np.vstack([round5._rotate_world_to_local(point, start_yaw) for point in predicted_world])
    path_error = np.linalg.norm(predicted_local - actual_local, axis=1)
    predicted_trigger = np.abs(predicted[1:, 2]) > 0.30
    actual_trigger = np.asarray([bool(item["auxiliary_velocity_supervisor_active"]) for item in future])
    return {
        "dataset": dataset,
        "sequence_id": trace["sequence"]["sequence_id"],
        "trace_path": str(trace_path.resolve()),
        "model": model_name,
        "transition_group": _transition_group(initial, target),
        "horizon_s": count * float(model["dt_s"]),
        "one_step_main_velocity_error_rad_s": float(one_step[0]),
        "one_step_auxiliary_angle_error_rad": float(one_step[1]),
        "one_step_auxiliary_velocity_error_rad_s": float(one_step[2]),
        "one_step_body_forward_error_m_s": float(one_step[3]),
        "one_step_body_lateral_error_m_s": float(one_step[4]),
        "one_step_world_z_rate_error_rad_s": float(one_step[5]),
        "one_step_final_main_target_error_rad_s": float(one_step[6]),
        "one_step_final_auxiliary_target_error_rad": float(one_step[7]),
        "terminal_main_velocity_error_rad_s": float(terminal[0]),
        "terminal_auxiliary_angle_error_rad": float(terminal[1]),
        "terminal_auxiliary_velocity_error_rad_s": float(terminal[2]),
        "terminal_body_forward_error_m_s": float(terminal[3]),
        "terminal_body_lateral_error_m_s": float(terminal[4]),
        "terminal_world_z_rate_error_rad_s": float(terminal[5]),
        "terminal_final_main_target_error_rad_s": float(terminal[6]),
        "terminal_final_auxiliary_target_error_rad": float(terminal[7]),
        "trajectory_rms_error_m": float(np.sqrt(np.mean(np.square(path_error)))),
        "trajectory_peak_error_m": float(np.max(path_error)),
        "terminal_heading_error_rad": float(round5._angle_wrap(float(result["heading_trace_rad"][-1]) - float(future[-1]["navigation_heading_rad"]))),
        "supervisor_trigger_actual_ticks": int(np.count_nonzero(actual_trigger)),
        "supervisor_trigger_predicted_ticks": int(np.count_nonzero(predicted_trigger)),
        "supervisor_trigger_mismatch_ticks": int(np.count_nonzero(predicted_trigger != actual_trigger)),
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    models = {
        "frozen_round5_m1": _load(args.old_m1),
        "round6_expanded_m1": _load(args.expanded_m1),
    }
    datasets = (
        ("round5_historical_heldout", _load(args.round5_manifest), args.round5_root),
        ("round6_historical_heldout", _load(args.round6_manifest), args.round6_root),
    )
    rows = []
    for dataset, manifest, root in datasets:
        for item in manifest["sequences"]:
            if item["dataset_role"] != "heldout_validation":
                continue
            trace_path = root / "traces" / f"{item['sequence_id']}.json"
            trace = _load(trace_path)
            for model_name, model in models.items():
                rows.append(_row(trace_path, trace, model, model_name, dataset))

    numeric_keys = [
        key for key, value in rows[0].items()
        if isinstance(value, (int, float)) and key != "horizon_s"
    ]
    aggregates = []
    groups = sorted({(row["dataset"], row["model"], row["transition_group"]) for row in rows})
    for dataset, model, group in groups:
        selected = [row for row in rows if (row["dataset"], row["model"], row["transition_group"]) == (dataset, model, group)]
        summary: dict[str, object] = {"dataset": dataset, "model": model, "transition_group": group, "sequence_count": len(selected)}
        for key in numeric_keys:
            values = np.abs([float(row[key]) for row in selected])
            summary[f"{key}_mean_abs"] = float(np.mean(values))
            summary[f"{key}_max_abs"] = float(np.max(values))
        aggregates.append(summary)

    payload = {
        "schema_version": "round7_frozen_legacy_error_layers_v1",
        "models_refit": False,
        "fresh_validation_claimed": False,
        "rows": rows,
        "aggregates": aggregates,
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "round7_legacy_error_layers.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    with (args.output_dir / "round7_legacy_error_layers.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return payload


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--round5-manifest", type=Path, required=True)
    parser.add_argument("--round5-root", type=Path, required=True)
    parser.add_argument("--round6-manifest", type=Path, required=True)
    parser.add_argument("--round6-root", type=Path, required=True)
    parser.add_argument("--old-m1", type=Path, required=True)
    parser.add_argument("--expanded-m1", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    payload = run(args)
    print(json.dumps({"rows": len(payload["rows"]), "aggregate_groups": len(payload["aggregates"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
