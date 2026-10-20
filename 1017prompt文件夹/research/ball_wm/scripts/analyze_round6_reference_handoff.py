#!/usr/bin/env python3
"""Fit/evaluate the sole Round 6 M1 extension and freeze its support cloud."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.drive_model import DriveModel  # noqa: E402
import analyze_round5_motion_transitions as round5  # noqa: E402


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
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def evaluate_local_gate(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    """Apply the predeclared Round 6 1.5 s whole-sequence gates."""

    trajectory = np.asarray(
        [float(row["m1_trajectory_rms_error_m"]) for row in rows], dtype=float
    )
    terminal_main = np.asarray(
        [abs(float(row["m1_terminal_state_error"]["main_joint_velocity_rad_s"])) for row in rows],
        dtype=float,
    )
    terminal_aux = np.asarray(
        [abs(float(row["m1_terminal_state_error"]["auxiliary_joint_rad"])) for row in rows],
        dtype=float,
    )

    def crossing(label: str) -> tuple[list[float], int, int]:
        required = [row for row in rows if bool(row[f"{label}_reversal_required"])]
        errors = [
            abs(float(row[f"m1_{label}_zero_crossing_error_s"]))
            for row in required
            if row[f"m1_{label}_zero_crossing_error_s"] is not None
        ]
        return errors, len(required), len(errors)

    main_crossing, main_required, main_measurable = crossing("main")
    aux_crossing, aux_required, aux_measurable = crossing("auxiliary")
    checks = {
        "minimum_fresh_heldout_sequences": len({str(row["sequence_id"]) for row in rows}) >= 4,
        "maximum_trajectory_rms_error_m_at_1p5s": bool(trajectory.size and np.max(trajectory) <= 0.10),
        "maximum_main_zero_crossing_mae_s": bool(
            main_measurable == main_required
            and (not main_crossing or np.mean(main_crossing) <= 0.25)
        ),
        "maximum_auxiliary_zero_crossing_mae_s": bool(
            aux_measurable == aux_required
            and (not aux_crossing or np.mean(aux_crossing) <= 0.20)
        ),
        "maximum_terminal_main_error_rad_s": bool(terminal_main.size and np.max(terminal_main) <= 0.08),
        "maximum_terminal_auxiliary_error_rad": bool(terminal_aux.size and np.max(terminal_aux) <= 0.05),
    }
    return {
        "fresh_heldout_sequence_count": len({str(row["sequence_id"]) for row in rows}),
        "metrics": {
            "trajectory_rms_error_mean_m_at_1p5s": float(np.mean(trajectory)),
            "trajectory_rms_error_max_m_at_1p5s": float(np.max(trajectory)),
            "terminal_main_error_mae_rad_s": float(np.mean(terminal_main)),
            "terminal_main_error_max_rad_s": float(np.max(terminal_main)),
            "terminal_auxiliary_error_mae_rad": float(np.mean(terminal_aux)),
            "terminal_auxiliary_error_max_rad": float(np.max(terminal_aux)),
            "main_reversal_required_count": main_required,
            "main_crossing_measurable_count": main_measurable,
            "main_zero_crossing_mae_s": float(np.mean(main_crossing)) if main_crossing else None,
            "auxiliary_reversal_required_count": aux_required,
            "auxiliary_crossing_measurable_count": aux_measurable,
            "auxiliary_zero_crossing_mae_s": float(np.mean(aux_crossing)) if aux_crossing else None,
        },
        "checks": checks,
        "accepted": all(checks.values()),
    }


def _core_traces(manifest_path: Path, root: Path):
    manifest = _load(manifest_path)
    result = []
    for item in manifest["sequences"]:
        if item["dataset_role"] != "fit_core":
            continue
        path = root / "traces" / f"{item['sequence_id']}.json"
        result.append((path, _load(path)))
    return result


def _evaluation_rows(
    *,
    manifest_path: Path,
    root: Path,
    protocol: Mapping[str, object],
    drive_model: DriveModel,
    model: Mapping[str, object],
) -> list[dict[str, object]]:
    manifest = _load(manifest_path)
    rows: list[dict[str, object]] = []
    for item in manifest["sequences"]:
        if item["dataset_role"] != "heldout_validation":
            continue
        path = root / "traces" / f"{item['sequence_id']}.json"
        trace_rows = round5._prediction_rows(path, _load(path), protocol, drive_model, model)
        rows.extend(row for row in trace_rows if float(row["horizon_s"]) == 1.5)
    return rows


def _all_evaluation_rows(
    *,
    manifest_path: Path,
    root: Path,
    protocol: Mapping[str, object],
    drive_model: DriveModel,
    model: Mapping[str, object],
) -> list[dict[str, object]]:
    manifest = _load(manifest_path)
    rows: list[dict[str, object]] = []
    for item in manifest["sequences"]:
        path = root / "traces" / f"{item['sequence_id']}.json"
        rows.extend(round5._prediction_rows(path, _load(path), protocol, drive_model, model))
    return rows


def _support_cloud(traces) -> np.ndarray:
    points: list[np.ndarray] = []
    for _, trace in traces:
        samples = round5._trace_samples(trace)
        for current, future in zip(samples, samples[1:]):
            state = round5._state_from_sample(current)
            request = np.asarray(
                [future["requested_main_rad_s"], future["requested_auxiliary_rad"]],
                dtype=np.float64,
            )
            points.append(np.concatenate((state, request)))
    return np.vstack(points)


def evaluate_round5_relative_regression(
    frozen_gate: Mapping[str, object], expanded_gate: Mapping[str, object]
) -> dict[str, object]:
    """Conservatively require no loss on the frozen model's primary old-domain metrics."""

    frozen_m1 = frozen_gate["m1"]
    expanded_m1 = expanded_gate["m1"]
    metric_names = (
        "endpoint_position_mae_m",
        "endpoint_position_p90_m_at_6s",
        "root_heading_change_mae_rad",
    )
    metrics = {}
    checks = {}
    for name in metric_names:
        frozen_value = float(frozen_m1[name])
        expanded_value = float(expanded_m1[name])
        metrics[name] = {
            "frozen_m1": frozen_value,
            "expanded_m1": expanded_value,
            "relative_change_fraction": (
                (expanded_value - frozen_value) / frozen_value
                if frozen_value > 0.0 else None
            ),
        }
        checks[f"expanded_no_worse_{name}"] = expanded_value <= frozen_value
    return {
        "definition": "expanded M1 must be no worse than frozen M1 on every primary frozen Round 5 held-out metric",
        "metrics": metrics,
        "checks": checks,
        "accepted": all(checks.values()),
        "scope_if_rejected": "not_a_unified_replacement; local use still requires every Round 6 local gate",
    }


def combined_candidate_decision(
    round6_local_gate: Mapping[str, object],
    round5_regression_gate: Mapping[str, object],
    round5_relative_regression: Mapping[str, object],
) -> dict[str, object]:
    local_passed = bool(round6_local_gate.get("accepted"))
    regression_passed = bool(round5_regression_gate.get("accepted"))
    relative_passed = bool(round5_relative_regression.get("accepted"))
    accepted = local_passed and regression_passed and relative_passed
    failed = []
    if not local_passed:
        failed.append("round6_local_1p5s_gate")
    if not regression_passed:
        failed.append("frozen_round5_candidate_gate_regression")
    if not relative_passed:
        failed.append("frozen_round5_relative_non_degradation")
    return {
        "accepted": accepted,
        "decision": "accepted" if accepted else "rejected",
        "round6_local_gate_passed": local_passed,
        "round5_regression_gate_passed": regression_passed,
        "round5_relative_non_degradation_passed": relative_passed,
        "failed_gate_groups": failed,
    }


def run_analysis(args: argparse.Namespace) -> dict[str, object]:
    protocol = round5.load_measurement_protocol(args.protocol)
    drive_model = DriveModel.from_json(args.drive_profile)
    old_model = _load(args.old_model)
    old_core = _core_traces(args.round5_manifest, args.round5_root)
    new_core = _core_traces(args.round6_manifest, args.round6_root)
    expanded = round5.fit_m1_model([*old_core, *new_core], protocol)
    expanded.update(
        {
            "candidate_name": "M1_round6_expanded_support_linear_state_delta_v1",
            "round5_model_frozen_and_unmodified": True,
            "round5_model_path": str(args.old_model.resolve()),
            "round5_model_sha256": _sha256(args.old_model),
            "round6_manifest_path": str(args.round6_manifest.resolve()),
            "round6_manifest_sha256": _sha256(args.round6_manifest),
            "heldout_opened_during_fit": False,
        }
    )
    _atomic_json(args.expanded_model, expanded)
    old_rows = _evaluation_rows(
        manifest_path=args.round6_manifest,
        root=args.round6_root,
        protocol=protocol,
        drive_model=drive_model,
        model=old_model,
    )
    expanded_rows = _evaluation_rows(
        manifest_path=args.round6_manifest,
        root=args.round6_root,
        protocol=protocol,
        drive_model=drive_model,
        model=expanded,
    )
    round5_manifest = _load(args.round5_manifest)
    expanded_round5_rows = _all_evaluation_rows(
        manifest_path=args.round5_manifest,
        root=args.round5_root,
        protocol=protocol,
        drive_model=drive_model,
        model=expanded,
    )
    round5_repeatability = round5._repeatability(expanded_round5_rows, round5_manifest)
    round5_regression_gate = round5.evaluate_candidate(
        expanded_round5_rows, protocol, round5_repeatability
    )
    frozen_round5_rows = _all_evaluation_rows(
        manifest_path=args.round5_manifest,
        root=args.round5_root,
        protocol=protocol,
        drive_model=drive_model,
        model=old_model,
    )
    frozen_round5_repeatability = round5._repeatability(frozen_round5_rows, round5_manifest)
    frozen_round5_gate = round5.evaluate_candidate(
        frozen_round5_rows, protocol, frozen_round5_repeatability
    )
    relative_regression = evaluate_round5_relative_regression(
        frozen_round5_gate, round5_regression_gate
    )
    cloud = _support_cloud([*old_core, *new_core])
    args.support_cloud.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.support_cloud,
        state_request_points=cloud,
        state_names=np.asarray(round5.STATE_NAMES),
        request_names=np.asarray(round5.REQUEST_NAMES),
        maximum_standardized_nearest_neighbor_distance=np.asarray(1.75),
    )
    old_gate = evaluate_local_gate(old_rows)
    expanded_gate = evaluate_local_gate(expanded_rows)
    combined_decision = combined_candidate_decision(
        expanded_gate, round5_regression_gate, relative_regression
    )
    result = {
        "schema_version": "round6_local_model_validation_v1",
        "fresh_heldout_was_sealed_before_fit": True,
        "new_complete_sequence_count": 12,
        "new_fit_sequence_count": 8,
        "new_fresh_heldout_sequence_count": 4,
        "old_m1": {
            "path": str(args.old_model.resolve()),
            "sha256": _sha256(args.old_model),
            "gate": old_gate,
            "rows": old_rows,
        },
        "expanded_m1": {
            "path": str(args.expanded_model.resolve()),
            "sha256": _sha256(args.expanded_model),
            "gate": expanded_gate,
            "round5_regression_gate": round5_regression_gate,
            "frozen_round5_gate_for_relative_comparison": frozen_round5_gate,
            "round5_relative_regression": relative_regression,
            "round5_repeatability": round5_repeatability,
            "rows": expanded_rows,
        },
        "support_cloud": {
            "path": str(args.support_cloud.resolve()),
            "sha256": _sha256(args.support_cloud),
            "point_count": int(cloud.shape[0]),
            "feature_order": [*round5.STATE_NAMES, *round5.REQUEST_NAMES],
            "minimum": cloud.min(axis=0).tolist(),
            "maximum": cloud.max(axis=0).tolist(),
            "mean": cloud.mean(axis=0).tolist(),
            "standard_deviation": cloud.std(axis=0).tolist(),
            "maximum_standardized_nearest_neighbor_distance": 1.75,
        },
        "combined_candidate_decision": combined_decision,
        "candidate_decision": combined_decision["decision"],
        "rejection_reason": (
            None
            if combined_decision["accepted"]
            else "candidate must pass both the frozen Round 5 gates and the predeclared Round 6 local 1.5 s gates"
        ),
    }
    _atomic_json(args.output, result)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--drive-profile", type=Path, required=True)
    parser.add_argument("--round5-manifest", type=Path, required=True)
    parser.add_argument("--round5-root", type=Path, required=True)
    parser.add_argument("--round6-manifest", type=Path, required=True)
    parser.add_argument("--round6-root", type=Path, required=True)
    parser.add_argument("--old-model", type=Path, required=True)
    parser.add_argument("--expanded-model", type=Path, required=True)
    parser.add_argument("--support-cloud", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    result = run_analysis(args)
    print(json.dumps({"candidate_decision": result["candidate_decision"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
