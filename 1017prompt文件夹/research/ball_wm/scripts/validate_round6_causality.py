#!/usr/bin/env python3
"""Emit executable evidence that the Round 6 shadow is causal and read-only."""

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

from benchmark.boundary_shadow import (  # noqa: E402
    M1TransitionPredictor,
    ReadOnlyBoundaryShadowPolicy,
    SupportEnvelope,
)
from benchmark.kinodynamic_planning import KinodynamicWaypoint  # noqa: E402
from benchmark.policies import EpisodeContext, OraclePathPolicy, PolicyObservation  # noqa: E402
from training.action_protocol import ActionProtocol  # noqa: E402


class CountingOracle(OraclePathPolicy):
    def __init__(self, protocol: ActionProtocol) -> None:
        super().__init__(protocol)
        self.calls = 0

    def act(self, observation: PolicyObservation, pointgoal_base_xy: object) -> np.ndarray:
        self.calls += 1
        return super().act(observation, pointgoal_base_xy)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical(payload: object) -> bytes:
    return json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def _digest(payload: object) -> str:
    return hashlib.sha256(_canonical(payload)).hexdigest()


def _observation(*, position: tuple[float, float], speed: float) -> PolicyObservation:
    proprio = np.zeros((5, 17), dtype=np.float32)
    proprio[-1, 0] = speed
    proprio[-1, 12] = speed
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray(position, dtype=np.float64),
        yaw_world_rad=0.0,
    )


def _episode_context() -> EpisodeContext:
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.4, 0.0, 0.0, "forward", 0.5, math.radians(-35.0)),
        KinodynamicWaypoint(0.6, -0.1, 0.2, "reverse", -0.5, math.radians(35.0)),
        KinodynamicWaypoint(0.9, -0.2, 0.2, "reverse", -0.5, math.radians(35.0)),
    )
    return EpisodeContext(
        episode_id="round6-causality",
        privileged=True,
        oracle_path_xy=np.asarray([(item.x_m, item.y_m) for item in reference]),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference,
    )


def run_validation(*, model_path: Path, support_cloud_path: Path) -> dict[str, object]:
    protocol = ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -math.radians(30.0)),
        maximum=(2.0, math.radians(30.0)),
        controls_per_block=5,
    )
    predictor = M1TransitionPredictor.from_json(model_path)
    with np.load(support_cloud_path, allow_pickle=False) as archive:
        support = SupportEnvelope.from_points(
            np.asarray(archive["state_request_points"], dtype=np.float64),
            maximum_distance=float(archive["maximum_standardized_nearest_neighbor_distance"]),
        )
    bases = [CountingOracle(protocol) for _ in range(3)]
    wrappers = [
        ReadOnlyBoundaryShadowPolicy(
            base,
            predictor=predictor,
            support_envelope=support,
            model_accepted=False,
            horizon_s=1.5,
        )
        for base in bases
    ]
    for wrapper in wrappers:
        wrapper.reset(_episode_context())
    direct = OraclePathPolicy(protocol)
    direct.reset(_episode_context())

    current = _observation(position=(0.6, -0.1), speed=0.2)
    goal = np.asarray((-0.3, -0.1), dtype=np.float64)
    direct_action = direct.act(current, goal)
    returned = [wrapper.act(current, goal).copy() for wrapper in wrappers]
    emitted = [
        {"context": wrapper.contexts[0].to_dict(), "record": wrapper.records[0]}
        for wrapper in wrappers
    ]
    before_hashes = [_digest(item) for item in emitted]

    # Index 0 is a truncated suffix. Indices 1 and 2 receive incompatible future logs.
    wrappers[1].act(
        _observation(position=(0.8, -0.2), speed=-0.1),
        np.asarray((-1.0, 0.2), dtype=np.float64),
    )
    wrappers[2].act(
        _observation(position=(-4.0, 3.0), speed=1.7),
        np.asarray((5.0, -2.0), dtype=np.float64),
    )
    after_hashes = [
        _digest({"context": wrapper.contexts[0].to_dict(), "record": wrapper.records[0]})
        for wrapper in wrappers
    ]
    action_equal = [bool(np.array_equal(action, direct_action)) for action in returned]
    checks = {
        "identical_prefix_emits_identical_context_and_plans": len(set(before_hashes)) == 1,
        "future_suffix_truncation_preserves_emitted_record": after_hashes[0] == before_hashes[0],
        "future_suffix_replacement_preserves_emitted_record_a": after_hashes[1] == before_hashes[1],
        "future_suffix_replacement_preserves_emitted_record_b": after_hashes[2] == before_hashes[2],
        "shadow_returns_exact_direct_b0_action": all(action_equal),
        "base_policy_called_once_at_observed_prefix": [base.calls for base in bases] == [1, 2, 2],
    }
    return {
        "schema_version": "round6_causality_validation_v1",
        "model_path": str(model_path.resolve()),
        "model_sha256": _sha256(model_path),
        "support_cloud_path": str(support_cloud_path.resolve()),
        "support_cloud_sha256": _sha256(support_cloud_path),
        "method": {
            "prefix": "same current observation, policy state, pointgoal, model, support cloud, and frozen reference",
            "truncation": "one wrapper receives no sample after the observed prefix",
            "replacement": "two wrappers receive mutually different measured future observations and pointgoals only after the emitted prefix",
        },
        "emitted_prefix_sha256_before_suffix": before_hashes,
        "emitted_prefix_sha256_after_suffix": after_hashes,
        "base_policy_call_counts_after_suffix": [base.calls for base in bases],
        "checks": checks,
        "passed": all(checks.values()),
    }


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--support-cloud", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    result = run_validation(
        model_path=args.model.resolve(), support_cloud_path=args.support_cloud.resolve()
    )
    _atomic_json(args.output.resolve(), result)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
