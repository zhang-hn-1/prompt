#!/usr/bin/env python3
"""Fit the sole Round 7 M2 without reading any fresh validation trace."""

from __future__ import annotations

import argparse
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
    M2_SCHEMA,
    PHYSICAL_STATE_NAMES,
    m2_features,
)
import analyze_round5_motion_transitions as round5  # noqa: E402
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


def _trace_pairs(trace: Mapping[str, object]):
    samples = round5._trace_samples(trace)
    for index, (current, future) in enumerate(zip(samples, samples[1:])):
        yield index, current, future


def reconstruct_trace(
    trace: Mapping[str, object], *, action_contract: Path
) -> dict[str, object]:
    smoothed_errors: list[np.ndarray] = []
    applied_errors: list[np.ndarray] = []
    supervisor_matches: list[bool] = []
    chain = _chain(action_contract)
    for index, current, future in _trace_pairs(trace):
        chain.reset(
            applied_action=(current["final_main_rad_s"], current["final_auxiliary_rad"]),
            last_smoothed_target=(
                current.get("smoothed_main_rad_s", current["final_main_rad_s"]),
                current.get("smoothed_auxiliary_rad", current["final_auxiliary_rad"]),
            ),
            tick_phase=(index + 1) % 5,
        )
        result = chain.step(
            (future["requested_main_rad_s"], future["requested_auxiliary_rad"]),
            auxiliary_position_rad=float(current["auxiliary_joint_rad"]),
            auxiliary_velocity_rad_s=float(current["auxiliary_joint_velocity_rad_s"]),
        )
        expected_smoothed = np.asarray(
            [future["smoothed_main_rad_s"], future["smoothed_auxiliary_rad"]], dtype=float
        )
        expected_applied = np.asarray(
            [future["final_main_rad_s"], future["final_auxiliary_rad"]], dtype=float
        )
        smoothed_errors.append(result.smoothed_target - expected_smoothed)
        applied_errors.append(result.applied_target - expected_applied)
        supervisor_matches.append(
            result.supervisor_active
            == bool(future["auxiliary_velocity_supervisor_active"])
        )
    smoothed = np.vstack(smoothed_errors)
    applied = np.vstack(applied_errors)
    return {
        "schema_version": "round7_measured_feedback_reconstruction_diagnostic_v1",
        "sample_pair_count": int(len(smoothed)),
        "smoothed_error_max_abs": np.max(np.abs(smoothed), axis=0).tolist(),
        "smoothed_error_rmse": np.sqrt(np.mean(np.square(smoothed), axis=0)).tolist(),
        "applied_error_max_abs": np.max(np.abs(applied), axis=0).tolist(),
        "applied_error_rmse": np.sqrt(np.mean(np.square(applied), axis=0)).tolist(),
        "supervisor_match_count": int(sum(supervisor_matches)),
        "supervisor_mismatch_count": int(len(supervisor_matches) - sum(supervisor_matches)),
        "role": "measured_feedback_reconstruction_diagnostic",
        "is_free_prediction_score": False,
    }


def _fit_traces(manifest_path: Path, root: Path) -> list[tuple[Path, dict[str, object]]]:
    manifest = _load(manifest_path)
    result = []
    for item in manifest["sequences"]:
        if item["dataset_role"] != "fit_core":
            continue
        path = root / "traces" / f"{item['sequence_id']}.json"
        trace = _load(path)
        sequence = trace.get("sequence")
        if not isinstance(sequence, Mapping) or sequence.get("dataset_role") != "fit_core":
            raise ValueError("fit selection contains a non-fit trace")
        result.append((path, trace))
    return result


def run(args: argparse.Namespace) -> dict[str, object]:
    contract = _load(args.round7_contract)
    if contract.get("schema_version") != "round7_explicit_runtime_contract_v1":
        raise ValueError("Round 7 frozen contract is required")
    traces = [
        *_fit_traces(args.round5_manifest, args.round5_root),
        *_fit_traces(args.round6_manifest, args.round6_root),
    ]
    design, targets, support = [], [], []
    reconstruction_rows = []
    identities = []
    for path, trace in traces:
        reconstruction = reconstruct_trace(trace, action_contract=args.action_contract)
        sequence_id = str(trace["sequence"]["sequence_id"])
        reconstruction_rows.append({"sequence_id": sequence_id, **reconstruction})
        chain = _chain(args.action_contract)
        for index, current, future in _trace_pairs(trace):
            chain.reset(
                applied_action=(current["final_main_rad_s"], current["final_auxiliary_rad"]),
                last_smoothed_target=(
                    current.get("smoothed_main_rad_s", current["final_main_rad_s"]),
                    current.get("smoothed_auxiliary_rad", current["final_auxiliary_rad"]),
                ),
                tick_phase=(index + 1) % 5,
            )
            request = np.asarray(
                [future["requested_main_rad_s"], future["requested_auxiliary_rad"]], dtype=float
            )
            state = _physical(current)
            execution_before = np.asarray(chain.snapshot().current_applied, dtype=float)
            execution = chain.step(
                request,
                auxiliary_position_rad=float(state[1]),
                auxiliary_velocity_rad_s=float(state[2]),
            )
            design.append(m2_features(state, execution.applied_target))
            targets.append(_physical(future) - state)
            support.append(
                np.concatenate((state, execution_before, request))
            )
        identities.append(
            {"sequence_id": sequence_id, "path": str(path.resolve()), "sha256": _sha256(path)}
        )
    x, y = np.vstack(design), np.vstack(targets)
    ridge = float(contract["model_identity"]["ridge_lambda"])
    penalty = np.eye(x.shape[1]) * ridge
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(x.T @ x + penalty, x.T @ y)
    residual = y - x @ coefficients
    model = {
        "schema_version": M2_SCHEMA,
        "candidate_name": contract["model_identity"]["name"],
        "candidate_role": contract["model_identity"]["role"],
        "physical_state_names": list(PHYSICAL_STATE_NAMES),
        "feature_count": 13,
        "feature_names": [
            "bias", *PHYSICAL_STATE_NAMES,
            "explicit_applied_main_rad_s", "explicit_applied_auxiliary_rad",
            "main_velocity_times_auxiliary_angle",
            "body_forward_times_auxiliary_angle",
            "applied_main_minus_main_velocity",
            "applied_auxiliary_minus_auxiliary_angle",
        ],
        "coefficient_feature_by_physical_state": coefficients.tolist(),
        "dt_s": 0.02,
        "ridge_lambda": ridge,
        "fit_sequence_count": len(traces),
        "fit_step_count": int(len(x)),
        "fit_trace_identities": identities,
        "fit_residual_physical_state_rmse": np.sqrt(np.mean(np.square(residual), axis=0)).tolist(),
        "fresh_manifest_path": str(args.fresh_manifest.resolve()),
        "fresh_manifest_sha256": _sha256(args.fresh_manifest),
        "fresh_validation_data_opened_during_fit": False,
        "deterministic_execution_outputs_are_not_learning_targets": True,
        "output_clipping": False,
        "causal_contract": {
            "future_measured_state_used": False,
            "future_applied_action_used": False,
            "future_supervisor_output_used": False,
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    _atomic_json(args.output_dir / "m2_explicit_runtime_round7.json", model)
    cloud = np.vstack(support)
    np.savez_compressed(
        args.output_dir / "m2_support_cloud.npz",
        state_execution_request_points=cloud,
        maximum_standardized_nearest_neighbor_distance=np.asarray(
            contract["validation_gates"]["support_maximum_standardized_nearest_neighbor_distance"]
        ),
    )
    aggregate = {
        "schema_version": "round7_execution_reconstruction_summary_v1",
        "role": "measured_feedback_reconstruction_diagnostic",
        "model_fit_completed_before_fresh_collection": True,
        "round7_contract_path": str(args.round7_contract.resolve()),
        "round7_contract_sha256": _sha256(args.round7_contract),
        "fresh_manifest_path": str(args.fresh_manifest.resolve()),
        "fresh_manifest_sha256": _sha256(args.fresh_manifest),
        "trace_rows": reconstruction_rows,
        "maximum_smoothed_error_abs": np.max(
            [row["smoothed_error_max_abs"] for row in reconstruction_rows], axis=0
        ).tolist(),
        "maximum_applied_error_abs": np.max(
            [row["applied_error_max_abs"] for row in reconstruction_rows], axis=0
        ).tolist(),
        "total_supervisor_mismatches": int(
            sum(row["supervisor_mismatch_count"] for row in reconstruction_rows)
        ),
    }
    _atomic_json(args.output_dir / "execution_reconstruction.json", aggregate)
    return {"model": model, "execution_reconstruction": aggregate}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-contract", type=Path, required=True)
    parser.add_argument("--round7-contract", type=Path, required=True)
    parser.add_argument("--fresh-manifest", type=Path, required=True)
    parser.add_argument("--round5-manifest", type=Path, required=True)
    parser.add_argument("--round5-root", type=Path, required=True)
    parser.add_argument("--round6-manifest", type=Path, required=True)
    parser.add_argument("--round6-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    result = run(_parse_args(argv))
    print(json.dumps({
        "fit_sequence_count": result["model"]["fit_sequence_count"],
        "maximum_applied_error_abs": result["execution_reconstruction"]["maximum_applied_error_abs"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
