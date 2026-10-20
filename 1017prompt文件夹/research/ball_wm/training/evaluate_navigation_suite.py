"""Cross-scene orchestrator for strict, manifest-locked PointGoal evaluation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import sys
from typing import Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.action_protocol import load_action_protocol
    from training.ball_wm_dataset import BallWMDataset
    from training.evaluate_navigation import (
        NavigationContext,
        select_navigation_contexts,
        write_context_manifest,
    )
    from training.navigation_metrics import bootstrap_interval, success_weighted_path_length, summarize_navigation
else:
    from .action_protocol import load_action_protocol
    from .ball_wm_dataset import BallWMDataset
    from .evaluate_navigation import NavigationContext, select_navigation_contexts, write_context_manifest
    from .navigation_metrics import bootstrap_interval, success_weighted_path_length, summarize_navigation


_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


@dataclass(frozen=True)
class PolicyCase:
    """One checkpoint/policy pairing evaluated against the shared manifest."""

    name: str
    checkpoint: Path
    policy: str
    planner_mode: str
    safety_weight: float


def build_local_policy_cases(pose_checkpoint: Path, safety_weight: float) -> tuple[PolicyCase, ...]:
    """Return the manifest-comparable local policies from one pose-enabled checkpoint."""

    if float(safety_weight) < 0.0:
        raise ValueError("safety_weight must be non-negative")
    checkpoint = Path(pose_checkpoint)
    return (
        PolicyCase("pose_only_cem", checkpoint, "cem", "pose_bev", 0.0),
        PolicyCase("pose_bev_cem", checkpoint, "cem", "pose_bev", float(safety_weight)),
        PolicyCase("random", checkpoint, "random", "pose_bev", 0.0),
    )


def scene_seed(master_seed: int, scene_index: int) -> int:
    """Derive a reproducible per-scene seed shared across policy cases."""

    if scene_index < 0:
        raise ValueError("scene_index must be non-negative")
    return int((int(master_seed) + int(scene_index)) % (2**32))


def context_tuples(result: Mapping[str, object]) -> list[tuple[int, int, int, int, str]]:
    """Return an order-independent identity tuple for every evaluated context."""

    episodes = result.get("episodes")
    if not isinstance(episodes, list):
        raise ValueError("evaluation result must contain an episodes list")
    tuples: list[tuple[int, int, int, int, str]] = []
    for episode in episodes:
        if not isinstance(episode, Mapping):
            raise ValueError("evaluation episode must be an object")
        fields = ("episode_index", "context_start", "context_last_frame", "goal_frame", "scene_path")
        if any(field not in episode for field in fields):
            raise ValueError("evaluation episode is missing a context identity field")
        tuples.append(
            (
                int(episode["episode_index"]),
                int(episode["context_start"]),
                int(episode["context_last_frame"]),
                int(episode["goal_frame"]),
                str(episode["scene_path"]),
            )
        )
    return sorted(tuples, key=lambda item: (item[4], item[0], item[1], item[2], item[3]))


def assert_policy_contexts_match(policy_results: Mapping[str, Sequence[Mapping[str, object]]]) -> None:
    """Reject cross-policy summaries unless every raw evaluation context matches."""

    expected: list[tuple[int, int, int, int, str]] | None = None
    expected_name: str | None = None
    for name in sorted(policy_results):
        tuples = sorted(
            tuple_ for result in policy_results[name] for tuple_ in context_tuples(result)
        )
        if expected is None:
            expected = tuples
            expected_name = name
        elif tuples != expected:
            raise ValueError(f"policy {name} does not use identical context tuples as {expected_name}")


def summarize_suite_policy(
    scene_results: Sequence[Mapping[str, object]],
    goal_tolerance_m: float,
    max_mpc_steps: int,
) -> dict[str, object]:
    """Aggregate all raw episode records, never summaries of summaries."""

    episodes: list[Mapping[str, object]] = []
    for result in scene_results:
        result_episodes = result.get("episodes")
        if not isinstance(result_episodes, list):
            raise ValueError("scene evaluation result must contain episodes")
        if any(not isinstance(episode, Mapping) for episode in result_episodes):
            raise ValueError("scene evaluation result contains an invalid episode")
        episodes.extend(result_episodes)
    return summarize_navigation(episodes, goal_tolerance_m, max_mpc_steps)


def _finite_episode_values(episodes: Sequence[Mapping[str, object]], field: str) -> list[float]:
    values: list[float] = []
    for episode in episodes:
        value = episode.get(field)
        if value is None:
            continue
        number = float(value)
        if number == number and number not in (float("inf"), float("-inf")):
            values.append(number)
    return values


def _summary_confidence_intervals(
    episodes: Sequence[Mapping[str, object]],
    *,
    seed: int,
    samples: int,
    confidence: float,
) -> dict[str, dict[str, float | int]]:
    """Bootstrap each strict navigation metric directly from raw episode records."""

    success = [float(episode.get("termination_reason") == "success") for episode in episodes]
    collision = [float(episode.get("termination_reason") == "collision") for episode in episodes]
    spl = [
        success_weighted_path_length(
            episode.get("termination_reason") == "success",
            float(episode.get("path_length_m") or 0.0),
            float(episode.get("reference_length_m") or 0.0),
        )
        for episode in episodes
    ]
    final_distance = _finite_episode_values(episodes, "final_distance_m") or [0.0]
    path_length = _finite_episode_values(episodes, "path_length_m") or [0.0]
    if not success:
        success = [0.0]
        collision = [0.0]
        spl = [0.0]
    return {
        "success_rate": bootstrap_interval(success, np.mean, seed=seed, samples=samples, confidence=confidence),
        "collision_rate": bootstrap_interval(collision, np.mean, seed=seed + 1, samples=samples, confidence=confidence),
        "spl": bootstrap_interval(spl, np.mean, seed=seed + 2, samples=samples, confidence=confidence),
        "final_distance_mean_m": bootstrap_interval(
            final_distance, np.mean, seed=seed + 3, samples=samples, confidence=confidence
        ),
        "path_length_mean_m": bootstrap_interval(
            path_length, np.mean, seed=seed + 4, samples=samples, confidence=confidence
        ),
    }


def _strict_dataset(data_root: Path) -> BallWMDataset:
    """Create a model-independent raw dataset view for shared context selection."""

    action_protocol = load_action_protocol(_CONTRACT_PATH)
    dataset = BallWMDataset(
        data_root,
        history_frames=5,
        num_preds=1,
        rollout_horizon=1,
        image_size=(224, 224),
        action_block=action_protocol.controls_per_block,
        with_bev=False,
        with_visibility=False,
    )
    if dataset.action_protocol_id != action_protocol.protocol_id:
        raise ValueError(
            "navigation-suite dataset action protocol does not match the active contract: "
            f"{dataset.action_protocol_id} != {action_protocol.protocol_id}"
        )
    return dataset


def _scene_groups(
    contexts: Sequence[NavigationContext],
    dataset: BallWMDataset,
) -> list[tuple[str, str]]:
    """Return sorted (scene label, USD path) pairs represented by manifest contexts."""

    groups: dict[str, str] = {}
    for context in contexts:
        scene_label = Path(dataset.episode_paths[context.episode_index]).parent.name
        prior_scene = groups.setdefault(scene_label, context.scene_path)
        if prior_scene != context.scene_path:
            raise ValueError(f"scene label {scene_label} maps to multiple USD paths")
    return sorted(groups.items())


def _read_child_result(output_path: Path, policy_name: str, scene_label: str) -> dict[str, object]:
    """Load an evaluator artifact; no artifact means no defensible aggregate result."""

    if not output_path.is_file():
        raise RuntimeError(f"{policy_name}/{scene_label} exited without writing {output_path}")
    try:
        payload = json.loads(output_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"{policy_name}/{scene_label} wrote invalid JSON") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("episodes"), list):
        raise RuntimeError(f"{policy_name}/{scene_label} JSON lacks episode records")
    return payload


def _write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False))


def run_navigation_suite(
    *,
    pose_checkpoint: Path,
    data_root: Path,
    out_dir: Path,
    seed: int,
    goal_offset: int,
    min_initial_goal_distance_m: float,
    goal_tolerance_m: float,
    max_mpc_steps: int,
    horizon: int,
    samples: int,
    elites: int,
    iterations: int,
    safety_weight: float,
    smoothness_weight: float = 0.0,
    safety_threshold: float = 0.5,
    bev_resolution_m: float = 0.1,
    unknown_weight: float = 1.0,
    min_initial_clearance_m: float = 0.0,
    bootstrap_samples: int = 1000,
    bootstrap_confidence: float = 0.95,
    device_name: str = "cuda",
) -> dict[str, object]:
    """Run one manifest-locked evaluator process per scene and policy case."""

    data_root = Path(data_root).resolve()
    out_dir = Path(out_dir)
    dataset = _strict_dataset(data_root)
    contexts = select_navigation_contexts(
        dataset,
        scene_name=None,
        num_eval=0,
        goal_offset=goal_offset,
        seed=seed,
        min_initial_goal_distance_m=min_initial_goal_distance_m,
        require_initial_clearance=True,
        min_initial_clearance_m=min_initial_clearance_m,
    )
    if not contexts:
        raise RuntimeError("strict protocol selected no valid navigation contexts")
    manifest_path = out_dir / "context_manifest.json"
    manifest = write_context_manifest(
        manifest_path,
        data_root,
        history_size=5,
        goal_offset=goal_offset,
        min_initial_goal_distance_m=min_initial_goal_distance_m,
        require_initial_clearance=True,
        seed=seed,
        contexts=contexts,
        min_initial_clearance_m=min_initial_clearance_m,
    )
    scene_groups = _scene_groups(contexts, dataset)
    cases = build_local_policy_cases(Path(pose_checkpoint), safety_weight)
    evaluator_path = Path(__file__).with_name("evaluate_navigation.py")
    policy_results: dict[str, list[dict[str, object]]] = {case.name: [] for case in cases}
    return_codes: dict[str, list[int]] = {case.name: [] for case in cases}

    for case in cases:
        for scene_index, (scene_label, _scene_path) in enumerate(scene_groups):
            output_path = out_dir / case.name / f"{scene_label}.json"
            command = [
                sys.executable,
                str(evaluator_path),
                "--checkpoint",
                str(case.checkpoint),
                "--data_root",
                str(data_root),
                "--out",
                str(output_path),
                "--scene",
                scene_label,
                "--num_eval",
                "0",
                "--goal_offset",
                str(goal_offset),
                "--context_manifest",
                str(manifest_path),
                "--min_initial_goal_distance",
                str(min_initial_goal_distance_m),
                "--min_initial_clearance",
                str(min_initial_clearance_m),
                "--require_initial_clearance",
                "--goal_tolerance",
                str(goal_tolerance_m),
                "--max_mpc_steps",
                str(max_mpc_steps),
                "--horizon",
                str(horizon),
                "--samples",
                str(samples),
                "--elites",
                str(elites),
                "--iterations",
                str(iterations),
                "--safety_weight",
                str(case.safety_weight),
                "--planner_mode",
                case.planner_mode,
                "--smoothness_weight",
                str(smoothness_weight),
                "--safety_threshold",
                str(safety_threshold),
                "--bev_resolution_m",
                str(bev_resolution_m),
                "--unknown_weight",
                str(unknown_weight),
                "--policy",
                case.policy,
                "--seed",
                str(scene_seed(seed, scene_index)),
                "--device",
                device_name,
            ]
            completed = subprocess.run(command, check=False)
            policy_results[case.name].append(_read_child_result(output_path, case.name, scene_label))
            return_codes[case.name].append(int(completed.returncode))

    assert_policy_contexts_match(policy_results)
    summaries: dict[str, dict[str, object]] = {}
    for case in cases:
        summary = summarize_suite_policy(policy_results[case.name], goal_tolerance_m, max_mpc_steps)
        raw_episodes = [
            episode
            for scene_result in policy_results[case.name]
            for episode in scene_result["episodes"]
            if isinstance(episode, Mapping)
        ]
        summary.update(
            {
                "policy": case.policy,
                "checkpoint": str(case.checkpoint),
                "planner_mode": case.planner_mode,
                "safety_weight": case.safety_weight,
                "confidence_intervals": _summary_confidence_intervals(
                    raw_episodes,
                    seed=seed,
                    samples=bootstrap_samples,
                    confidence=bootstrap_confidence,
                ),
                "scene_result_paths": [
                    str(out_dir / case.name / f"{scene_label}.json") for scene_label, _ in scene_groups
                ],
                "child_return_codes": return_codes[case.name],
            }
        )
        _write_json(out_dir / f"{case.name}_summary.json", summary)
        summaries[case.name] = summary
    exit_code = int(
        any(code != 0 for codes in return_codes.values() for code in codes)
        or any(int(summary["runtime_error_count"]) > 0 for summary in summaries.values())
    )
    result: dict[str, object] = {
        "context_manifest": str(manifest_path),
        "num_contexts": len(contexts),
        "goal_tolerance_m": float(goal_tolerance_m),
        "min_initial_clearance_m": float(min_initial_clearance_m),
        "max_mpc_steps": int(max_mpc_steps),
        "planner_mode": "pose_bev",
        "goal_source": "wheel_odometry_pointgoal",
        "bootstrap_samples": int(bootstrap_samples),
        "bootstrap_confidence": float(bootstrap_confidence),
        "summaries": summaries,
        "exit_code": exit_code,
    }
    _write_json(out_dir / "comparison.json", result)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Strict cross-scene Isaac Sim PointGoal evaluation suite")
    parser.add_argument("--pose_checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--out_dir", type=Path, required=True)
    parser.add_argument("--goal_offset", type=int, default=20)
    parser.add_argument("--min_initial_goal_distance", type=float, default=0.5)
    parser.add_argument("--min_initial_clearance", type=float, default=0.0)
    parser.add_argument("--goal_tolerance", type=float, default=0.2)
    parser.add_argument("--max_mpc_steps", type=int, default=50)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--elites", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--safety_weight", type=float, default=0.1)
    parser.add_argument("--smoothness_weight", type=float, default=0.0)
    parser.add_argument("--safety_threshold", type=float, default=0.5)
    parser.add_argument("--bev_resolution_m", type=float, default=0.1)
    parser.add_argument("--unknown_weight", type=float, default=1.0)
    parser.add_argument("--bootstrap_samples", type=int, default=1000)
    parser.add_argument("--bootstrap_confidence", type=float, default=0.95)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args(argv)
    result = run_navigation_suite(
        pose_checkpoint=args.pose_checkpoint,
        data_root=args.data_root,
        out_dir=args.out_dir,
        seed=args.seed,
        goal_offset=args.goal_offset,
        min_initial_goal_distance_m=args.min_initial_goal_distance,
        min_initial_clearance_m=args.min_initial_clearance,
        goal_tolerance_m=args.goal_tolerance,
        max_mpc_steps=args.max_mpc_steps,
        horizon=args.horizon,
        samples=args.samples,
        elites=args.elites,
        iterations=args.iterations,
        safety_weight=args.safety_weight,
        smoothness_weight=args.smoothness_weight,
        safety_threshold=args.safety_threshold,
        bev_resolution_m=args.bev_resolution_m,
        unknown_weight=args.unknown_weight,
        bootstrap_samples=args.bootstrap_samples,
        bootstrap_confidence=args.bootstrap_confidence,
        device_name=args.device,
    )
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return int(result["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
