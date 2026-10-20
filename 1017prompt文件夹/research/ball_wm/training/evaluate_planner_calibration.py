"""Open-loop Isaac calibration for no-map local PointGoal planner scores."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import torch

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.ball_wm_dataset import BallWMDataset, _load_episode
    from training.action_protocol import ActionProtocol, load_action_protocol
    from training.evaluate_navigation import (
        IsaacNavigationRuntime,
        _recorded_context_sample,
        _recorded_reference_length,
        _state_row,
        build_model_context,
        resolve_navigation_contexts,
        require_checkpoint_protocol,
        sample_random_action_block,
    )
    from training.joint_jepa import JointModelConfig, build_joint_model
    from training.navigation_metrics import spearman_rank_correlation
    from training.planner import plan, score_action_sequences
    from training.pose_utils import pointgoal_in_base
    from training.proprioception import checkpoint_velocity_frame
else:
    from .ball_wm_dataset import BallWMDataset, _load_episode
    from .action_protocol import ActionProtocol, load_action_protocol
    from .evaluate_navigation import (
        IsaacNavigationRuntime,
        _recorded_context_sample,
        _recorded_reference_length,
        _state_row,
        build_model_context,
        resolve_navigation_contexts,
        require_checkpoint_protocol,
        sample_random_action_block,
    )
    from .joint_jepa import JointModelConfig, build_joint_model
    from .navigation_metrics import spearman_rank_correlation
    from .planner import plan, score_action_sequences
    from .pose_utils import pointgoal_in_base
    from .proprioception import checkpoint_velocity_frame


_CANDIDATE_NAMES = ("zero", "aux_positive", "aux_negative", "random", "cem")
_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"
_ACTIVE_RUNTIME: Any | None = None


def manifest_navigation_protocol(path: Path) -> tuple[int, float, float, bool]:
    """Read the frozen PointGoal selection constraints required by the manifest loader."""

    try:
        payload = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read context manifest: {path}") from exc
    if not isinstance(payload, Mapping):
        raise ValueError("context manifest must be an object")
    goal_offset = payload.get("goal_offset")
    min_distance = payload.get("min_initial_goal_distance_m")
    min_clearance = payload.get("min_initial_clearance_m", 0.0)
    require_clearance = payload.get("require_initial_clearance")
    if (
        not isinstance(goal_offset, int)
        or goal_offset < 1
        or not isinstance(min_distance, (int, float))
        or not math.isfinite(float(min_distance))
        or float(min_distance) < 0.0
        or not isinstance(min_clearance, (int, float))
        or not math.isfinite(float(min_clearance))
        or float(min_clearance) < 0.0
        or not isinstance(require_clearance, bool)
    ):
        raise ValueError("context manifest has an invalid navigation protocol")
    return int(goal_offset), float(min_distance), float(min_clearance), require_clearance


def manifest_goal_offset(path: Path) -> int:
    """Read the immutable PointGoal offset selected when the shared manifest was frozen."""

    return manifest_navigation_protocol(path)[0]


def _finite_scalar(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def summarize_calibration_contexts(contexts: Sequence[Mapping[str, object]]) -> dict[str, object]:
    """Summarize whether planner scores rank fixed executed candidates correctly."""

    expected_names: set[str] | None = None
    rank_correlations: list[float] = []
    top1_matches: list[float] = []
    cem_better_than_zero: list[float] = []
    cem_better_than_random: list[float] = []
    runtime_error_contexts = 0

    for context in contexts:
        if context.get("runtime_error") is not None:
            runtime_error_contexts += 1
            continue
        candidates = context.get("candidates")
        if not isinstance(candidates, list) or not candidates:
            raise ValueError("successful calibration context must contain candidates")
        names: list[str] = []
        predicted_costs: list[float] = []
        actual_distances: list[float] = []
        actual_by_name: dict[str, float] = {}
        for candidate in candidates:
            if not isinstance(candidate, Mapping):
                raise ValueError("calibration candidate must be an object")
            name = candidate.get("name")
            if not isinstance(name, str) or not name:
                raise ValueError("calibration candidate name must be a non-empty string")
            names.append(name)
            predicted_cost = _finite_scalar(candidate.get("predicted_total_cost"), "predicted_total_cost")
            actual_distance = _finite_scalar(candidate.get("actual_final_distance_m"), "actual_final_distance_m")
            if actual_distance < 0.0:
                raise ValueError("actual_final_distance_m must be non-negative")
            predicted_costs.append(predicted_cost)
            actual_distances.append(actual_distance)
            actual_by_name[name] = actual_distance
        if len(names) != len(set(names)):
            raise ValueError("calibration context has duplicate candidate names")
        name_set = set(names)
        if expected_names is None:
            expected_names = name_set
        elif name_set != expected_names:
            raise ValueError("calibration contexts must use identical candidate names")
        if "cem" not in name_set or "zero" not in name_set or "random" not in name_set:
            raise ValueError("calibration candidate names must include cem, zero, and random")

        rank_correlations.append(spearman_rank_correlation(predicted_costs, actual_distances))
        predicted_minimum = min(predicted_costs)
        actual_minimum = min(actual_distances)
        predicted_winners = {
            name for name, value in zip(names, predicted_costs) if math.isclose(value, predicted_minimum, abs_tol=1e-9)
        }
        actual_winners = {
            name for name, value in zip(names, actual_distances) if math.isclose(value, actual_minimum, abs_tol=1e-9)
        }
        top1_matches.append(float(bool(predicted_winners & actual_winners)))
        cem_better_than_zero.append(float(actual_by_name["cem"] < actual_by_name["zero"]))
        cem_better_than_random.append(float(actual_by_name["cem"] < actual_by_name["random"]))

    usable_contexts = len(rank_correlations)
    return {
        "num_contexts": len(contexts),
        "usable_contexts": usable_contexts,
        "runtime_error_contexts": runtime_error_contexts,
        "candidate_names": sorted(expected_names or ()),
        "spearman_rho_mean": float(np.mean(rank_correlations)) if rank_correlations else 0.0,
        "top1_match_rate": float(np.mean(top1_matches)) if top1_matches else 0.0,
        "cem_better_than_zero_rate": float(np.mean(cem_better_than_zero)) if cem_better_than_zero else 0.0,
        "cem_better_than_random_rate": float(np.mean(cem_better_than_random)) if cem_better_than_random else 0.0,
    }


def _fixed_action_candidates(
    horizon: int,
    rng: np.random.Generator,
    action_protocol: ActionProtocol,
) -> dict[str, np.ndarray]:
    """Produce the fixed non-CEM candidates used in every calibration context."""

    if horizon < 1:
        raise ValueError("horizon must be positive")
    if action_protocol.controls_per_block != 5 or len(action_protocol.minimum) != 2:
        raise ValueError("calibration requires five 2-D controls per action block")
    zero = np.zeros((horizon, action_protocol.action_block_dim), dtype=np.float32)
    aux_positive = zero.copy()
    aux_positive[:, 1::2] = action_protocol.maximum[1]
    aux_negative = zero.copy()
    aux_negative[:, 1::2] = action_protocol.minimum[1]
    random_actions = np.stack(
        [sample_random_action_block(rng, action_protocol) for _ in range(horizon)]
    ).astype(np.float32)
    return {
        "zero": zero,
        "aux_positive": aux_positive,
        "aux_negative": aux_negative,
        "random": random_actions,
    }


def _execute_open_loop_candidate(
    runtime: IsaacNavigationRuntime,
    state_row: Mapping[str, np.ndarray],
    action_sequence: np.ndarray,
    goal_xy: np.ndarray,
    initial_distance_m: float,
) -> dict[str, object]:
    """Reset exact state then execute all candidate blocks without replanning."""

    runtime.reset_to_context(state_row)
    minimum_clearance = float("inf")
    collision = False
    observation = None
    for action_block in action_sequence:
        observation = runtime.execute_action_block(action_block)
        minimum_clearance = min(minimum_clearance, float(observation.clearance_m))
        collision = collision or bool(observation.collision)
        if collision:
            break
    if observation is None:
        raise RuntimeError("open-loop candidate did not execute an action block")
    final_distance = float(np.linalg.norm(np.asarray(observation.position_xy, dtype=np.float64) - goal_xy))
    if not math.isfinite(final_distance):
        raise RuntimeError("runtime returned a non-finite final distance")
    return {
        "actual_final_distance_m": final_distance,
        "actual_distance_delta_m": final_distance - float(initial_distance_m),
        "actual_collision": collision,
        "actual_minimum_clearance_m": minimum_clearance,
        "actual_final_position_xy": np.asarray(observation.position_xy, dtype=np.float32).tolist(),
    }


def run_planner_calibration(
    *,
    checkpoint_path: Path,
    data_root: Path,
    context_manifest_path: Path,
    output_path: Path,
    scene_name: str | None,
    horizon: int = 5,
    samples: int = 64,
    elites: int = 8,
    iterations: int = 3,
    safety_weight: float = 0.1,
    smoothness_weight: float = 0.0,
    safety_threshold: float = 0.5,
    bev_resolution_m: float = 0.1,
    unknown_weight: float = 1.0,
    seed: int = 3072,
    device_name: str = "cuda",
) -> dict[str, object]:
    """Score five no-map candidates jointly, then calibrate scores against Isaac execution."""

    global _ACTIVE_RUNTIME
    _ACTIVE_RUNTIME = None
    if horizon != 5:
        raise ValueError("calibration horizon is fixed at five action blocks")
    if min(samples, elites, iterations) < 1:
        raise ValueError("samples, elites, and iterations must be positive")
    device = torch.device(device_name if not device_name.startswith("cuda") or torch.cuda.is_available() else "cpu")
    torch.manual_seed(seed)
    random_rng = np.random.default_rng(seed)
    action_protocol = load_action_protocol(_CONTRACT_PATH)
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if not isinstance(checkpoint, Mapping) or "config" not in checkpoint or "model" not in checkpoint:
        raise ValueError("checkpoint must contain config and model")
    require_checkpoint_protocol(checkpoint, action_protocol.protocol_id)
    checkpoint_velocity_frame(checkpoint)
    config = JointModelConfig(**checkpoint["config"])
    if not config.pose_head_enabled:
        raise ValueError("calibration requires a pose-enabled checkpoint")
    if config.rollout_horizon < horizon:
        raise ValueError("checkpoint rollout_horizon cannot cover calibration horizon")
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval()
    dataset = BallWMDataset(
        data_root,
        history_frames=config.history_size,
        num_preds=config.num_preds,
        rollout_horizon=config.rollout_horizon,
        image_size=(config.image_size, config.image_size),
        action_source=config.action_source,
        proprio_velocity_frame=config.proprio_velocity_frame,
        action_block=action_protocol.controls_per_block,
        with_bev=False,
        with_visibility=False,
    )
    if dataset.action_protocol_id != action_protocol.protocol_id:
        raise ValueError(
            "calibration dataset action protocol does not match the active contract: "
            f"{dataset.action_protocol_id} != {action_protocol.protocol_id}"
        )
    (
        manifest_goal_offset_value,
        manifest_min_distance,
        manifest_min_clearance,
        manifest_require_clearance,
    ) = manifest_navigation_protocol(
        context_manifest_path
    )
    contexts = resolve_navigation_contexts(
        dataset,
        scene_name,
        num_eval=0,
        goal_offset=manifest_goal_offset_value,
        seed=seed,
        context_manifest_path=context_manifest_path,
        min_initial_goal_distance_m=manifest_min_distance,
        require_initial_clearance=manifest_require_clearance,
        min_initial_clearance_m=manifest_min_clearance,
    )
    if not contexts:
        raise ValueError("calibration manifest has no matching contexts")
    scene_paths = {context.scene_path for context in contexts}
    if len(scene_paths) != 1:
        raise ValueError("calibration requires contexts from exactly one scene; pass --scene")

    runtime: IsaacNavigationRuntime | None = None
    records: list[dict[str, object]] = []
    try:
        for context in contexts:
            record: dict[str, object] = {
                "episode_index": context.episode_index,
                "context_start": context.context_start,
                "context_last_frame": context.context_last_frame,
                "goal_frame": context.goal_frame,
                "scene_path": context.scene_path,
                "runtime_error": None,
            }
            try:
                episode = _load_episode(str(dataset.episode_paths[context.episode_index]))
                states = episode["states"]
                metadata = episode["metadata"]
                if not isinstance(states, Mapping) or not isinstance(metadata, Mapping):
                    raise RuntimeError("episode data must contain state and metadata mappings")
                state_row = _state_row(states, context.start_state_index)
                goal_xy = np.asarray(states["pos"][context.goal_state_index, :2], dtype=np.float32)
                if goal_xy.shape != (2,) or not np.isfinite(goal_xy).all():
                    raise RuntimeError("recorded PointGoal is invalid")
                model_context = {
                    key: value.to(device)
                    for key, value in build_model_context(
                        _recorded_context_sample(dataset, context.episode_index, context.context_start),
                        config.history_size,
                    ).items()
                }
                if runtime is None:
                    runtime = IsaacNavigationRuntime(
                        scene_path=context.scene_path,
                        image_size=config.image_size,
                        ball_radius_m=float(metadata.get("ball_radius_m", 0.4)),
                        device=str(device),
                        proprio_velocity_frame=config.proprio_velocity_frame,
                    )
                    _ACTIVE_RUNTIME = runtime
                initial_observation = runtime.reset_to_context(state_row)
                goal_relative_xy = torch.from_numpy(
                    pointgoal_in_base(initial_observation.position_xy, initial_observation.yaw, goal_xy)
                ).to(device).reshape(1, 2)
                fixed = _fixed_action_candidates(horizon, random_rng, action_protocol)
                with torch.no_grad():
                    planned = plan(
                        model,
                        dict(model_context),
                        action_protocol=action_protocol,
                        goal_emb=None,
                        planner_mode="pose_bev",
                        goal_relative_xy=goal_relative_xy,
                        horizon=horizon,
                        samples=samples,
                        elites=elites,
                        iterations=iterations,
                        safety_weight=safety_weight,
                        smoothness_weight=smoothness_weight,
                        safety_threshold=safety_threshold,
                        history_size=config.history_size,
                        bev_resolution_m=bev_resolution_m,
                        robot_radius_m=runtime.ball_radius_m,
                        unknown_weight=unknown_weight,
                        device=device,
                    )
                    fixed["cem"] = planned.actions.detach().cpu().numpy().astype(np.float32)
                    action_sequences = torch.from_numpy(
                        np.stack([fixed[name] for name in _CANDIDATE_NAMES])
                    ).to(device).unsqueeze(0)
                    scores = score_action_sequences(
                        model,
                        dict(model_context),
                        action_sequences,
                        planner_mode="pose_bev",
                        goal_emb=None,
                        goal_relative_xy=goal_relative_xy,
                        safety_weight=safety_weight,
                        smoothness_weight=smoothness_weight,
                        history_size=config.history_size,
                        bev_resolution_m=bev_resolution_m,
                        robot_radius_m=runtime.ball_radius_m,
                        unknown_weight=unknown_weight,
                    )
                candidates: list[dict[str, object]] = []
                for candidate_index, name in enumerate(_CANDIDATE_NAMES):
                    predicted_endpoint = None
                    if scores.predicted_relative_pose is not None:
                        predicted_endpoint = scores.predicted_relative_pose[0, candidate_index, -1].detach().cpu().numpy()
                    executed = _execute_open_loop_candidate(
                        runtime,
                        state_row,
                        fixed[name],
                        goal_xy,
                        context.initial_distance_m,
                    )
                    candidates.append(
                        {
                            "name": name,
                            "action_sequence": fixed[name].tolist(),
                            "predicted_total_cost": float(scores.total_costs[0, candidate_index]),
                            "predicted_goal_cost": float(scores.goal_costs[0, candidate_index]),
                            "predicted_safety_cost": float(scores.safety_costs[0, candidate_index]),
                            "predicted_smoothness_cost": float(scores.smoothness_costs[0, candidate_index]),
                            "predicted_relative_endpoint": None if predicted_endpoint is None else predicted_endpoint.tolist(),
                            **executed,
                        }
                    )
                record.update(
                    {
                        "goal_xy": goal_xy.tolist(),
                        "initial_distance_m": float(context.initial_distance_m),
                        "reference_length_m": _recorded_reference_length(
                            states, context.start_state_index, context.goal_state_index
                        ),
                        "candidates": candidates,
                    }
                )
            except Exception as exc:
                record["runtime_error"] = f"{type(exc).__name__}: {exc}"
            records.append(record)
    finally:
        # Defer Kit shutdown to ``main`` so result JSON is written before close().
        _ACTIVE_RUNTIME = runtime

    result: dict[str, object] = {
        "schema_version": "pointgoal_planner_calibration_v1",
        "checkpoint": str(checkpoint_path),
        "data_root": str(data_root),
        "context_manifest": str(context_manifest_path),
        "planner_mode": "pose_bev",
        "goal_source": "wheel_odometry_pointgoal",
        "action_protocol_id": action_protocol.protocol_id,
        "controls_per_block": action_protocol.controls_per_block,
        "horizon": horizon,
        "contexts": records,
        "summary": summarize_calibration_contexts(records),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Isaac open-loop local PointGoal planner calibration")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--context_manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scene", type=str, default=None)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--elites", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--safety_weight", type=float, default=0.1)
    parser.add_argument("--smoothness_weight", type=float, default=0.0)
    parser.add_argument("--safety_threshold", type=float, default=0.5)
    parser.add_argument("--bev_resolution_m", type=float, default=0.1)
    parser.add_argument("--unknown_weight", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args(argv)
    try:
        result = run_planner_calibration(
            checkpoint_path=args.checkpoint,
            data_root=args.data_root,
            context_manifest_path=args.context_manifest,
            output_path=args.out,
            scene_name=args.scene,
            horizon=args.horizon,
            samples=args.samples,
            elites=args.elites,
            iterations=args.iterations,
            safety_weight=args.safety_weight,
            smoothness_weight=args.smoothness_weight,
            safety_threshold=args.safety_threshold,
            bev_resolution_m=args.bev_resolution_m,
            unknown_weight=args.unknown_weight,
            seed=args.seed,
            device_name=args.device,
        )
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)
        return 1 if int(result["summary"]["runtime_error_contexts"]) else 0
    finally:
        if _ACTIVE_RUNTIME is not None:
            _ACTIVE_RUNTIME.close(return_code=0)


if __name__ == "__main__":
    raise SystemExit(main())
