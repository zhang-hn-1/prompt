"""Collect same-context multi-action rollouts for candidate ranking training.

The module-level action sampler is Isaac-free so that the data contract can be
tested on a workstation.  Isaac is imported only after ``collect_candidate_contexts``
starts a simulator runtime.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import json
import math
import sys
from typing import Any, Mapping

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.ball_wm_dataset import BallWMDataset, _load_episode
    from training.action_protocol import ActionProtocol, load_action_protocol
    from training.candidate_rollout_schema import write_candidate_context_atomic
    from training.evaluate_navigation import (
        IsaacNavigationRuntime,
        _finite_vector,
        _state_row,
        _yaw_from_wxyz,
        resolve_navigation_contexts,
    )
    from training.pose_utils import pointgoal_in_base
else:
    from ..training.ball_wm_dataset import BallWMDataset, _load_episode
    from ..training.action_protocol import ActionProtocol, load_action_protocol
    from ..training.candidate_rollout_schema import write_candidate_context_atomic
    from ..training.evaluate_navigation import (
        IsaacNavigationRuntime,
        _finite_vector,
        _state_row,
        _yaw_from_wxyz,
        resolve_navigation_contexts,
    )
    from ..training.pose_utils import pointgoal_in_base


_HORIZON = 5
_ACTION_DIM = 10
_CONTROL_DIM = 2
_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


def _validate_action_bounds(action_min: np.ndarray, action_max: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray(action_min, dtype=np.float32)
    upper = np.asarray(action_max, dtype=np.float32)
    if lower.shape != (2,) or upper.shape != (2,) or not np.isfinite(lower).all() or not np.isfinite(upper).all():
        raise ValueError("action bounds must be finite vectors with shape (2,)")
    if np.any(lower > upper):
        raise ValueError("action_min must not exceed action_max")
    if np.any(lower > 0.0) or np.any(upper < 0.0):
        raise ValueError("action bounds must contain the neutral zero command")
    return lower, upper


def fixed_candidate_action_sequences(
    horizon: int,
    candidate_count: int,
    action_protocol: ActionProtocol,
    seed: int,
) -> np.ndarray:
    """Build deterministic primitive-plus-random action sequences.

    Each candidate contains ``horizon`` model action blocks; every block is a
    flattened sequence of five 50-Hz ``(main, auxiliary)`` controls.  The first
    candidates are fixed primitives so every context has interpretable anchors,
    while remaining candidates provide local coverage for ranking supervision.
    """

    if isinstance(horizon, bool) or int(horizon) != _HORIZON:
        raise ValueError("candidate protocol requires horizon=5")
    if isinstance(candidate_count, bool) or int(candidate_count) < 8:
        raise ValueError("candidate_count must be at least 8")
    if not isinstance(action_protocol, ActionProtocol):
        raise TypeError("action_protocol must be an ActionProtocol")
    if action_protocol.controls_per_block != 5 or len(action_protocol.minimum) != _CONTROL_DIM:
        raise ValueError("candidate rollout protocol requires five 2-D controls per block")
    component_bounds = action_protocol.component_bounds_numpy()
    lower, upper = _validate_action_bounds(component_bounds[:, 0], component_bounds[:, 1])
    if isinstance(seed, bool) or int(seed) < 0:
        raise ValueError("seed must be a non-negative integer")

    count = int(candidate_count)
    rng = np.random.default_rng(int(seed))
    controls = np.zeros((count, _HORIZON, 5, _CONTROL_DIM), dtype=np.float32)
    main_forward = 0.70 * upper[0]
    main_reverse = 0.70 * lower[0]
    aux_left = 0.70 * upper[1]
    aux_right = 0.70 * lower[1]

    primitive_blocks = (
        (0.0, 0.0),
        (main_forward, 0.0),
        (main_reverse, 0.0),
        (0.0, aux_left),
        (0.0, aux_right),
        (main_forward, aux_left),
        (main_forward, aux_right),
    )
    for candidate_index, (main, auxiliary) in enumerate(primitive_blocks):
        controls[candidate_index, ..., 0] = main
        controls[candidate_index, ..., 1] = auxiliary
    # A deliberately changing primitive tests whether a model captures action
    # sequencing rather than only the mean command of a block.
    controls[7, :3, ..., 0] = main_forward
    controls[7, 3:, ..., 0] = main_reverse
    controls[7, :3, ..., 1] = aux_left
    controls[7, 3:, ..., 1] = aux_right

    if count > len(primitive_blocks) + 1:
        controls[len(primitive_blocks) + 1 :] = rng.uniform(
            lower,
            upper,
            size=(count - len(primitive_blocks) - 1, _HORIZON, 5, _CONTROL_DIM),
        ).astype(np.float32)
    return controls.reshape(count, _HORIZON, _ACTION_DIM)


def _relative_pose_from_observation(
    source_position_xy: np.ndarray,
    source_yaw: float,
    position_xy: np.ndarray,
    yaw: float,
) -> np.ndarray:
    source = _finite_vector(source_position_xy, (2,), "source_position_xy")
    position = _finite_vector(position_xy, (2,), "position_xy")
    if not math.isfinite(float(source_yaw)) or not math.isfinite(float(yaw)):
        raise ValueError("source_yaw and yaw must be finite")
    delta = pointgoal_in_base(source, float(source_yaw), position)
    delta_yaw = (float(yaw) - float(source_yaw) + math.pi) % (2.0 * math.pi) - math.pi
    return np.asarray([delta[0], delta[1], math.sin(delta_yaw), math.cos(delta_yaw)], dtype=np.float32)


def _source_episode_relative_path(episode_path: Path, formal_data_root: Path) -> str:
    try:
        relative = episode_path.resolve().relative_to(formal_data_root.resolve())
    except ValueError as exc:
        raise ValueError(f"episode is outside formal_data_root: {episode_path}") from exc
    if not relative.parts:
        raise ValueError("episode path must be a non-empty relative path")
    return relative.as_posix()


def collect_candidate_contexts(
    *,
    formal_data_root: str | Path,
    candidate_root: str | Path,
    context_manifest: str | Path,
    scene_name: str,
    candidate_count: int = 16,
    horizon: int = 5,
    seed: int = 3072,
    device: str = "cuda",
    goal_offset: int = 20,
    min_initial_goal_distance_m: float = 0.5,
    require_initial_clearance: bool = True,
) -> dict[str, object]:
    """Execute fixed candidates in Isaac and publish one artifact per context."""

    if not isinstance(scene_name, str) or not scene_name.strip():
        raise ValueError("scene_name must be a non-empty scene selector")
    if isinstance(seed, bool) or int(seed) < 0:
        raise ValueError("seed must be a non-negative integer")
    data_root = Path(formal_data_root).resolve()
    output_root = Path(candidate_root).resolve()
    action_protocol = load_action_protocol(_CONTRACT_PATH)
    dataset = BallWMDataset(
        data_root,
        history_frames=5,
        num_preds=1,
        rollout_horizon=5,
        image_size=(224, 224),
        action_block=action_protocol.controls_per_block,
        with_bev=False,
        with_visibility=False,
    )
    if dataset.action_protocol_id != action_protocol.protocol_id:
        raise ValueError(
            "formal dataset action protocol does not match the active contract: "
            f"{dataset.action_protocol_id} != {action_protocol.protocol_id}"
        )
    if dataset.controls_per_block != action_protocol.controls_per_block:
        raise ValueError(
            "formal dataset controls_per_block does not match the active contract"
        )
    contexts = resolve_navigation_contexts(
        dataset,
        scene_name,
        num_eval=0,
        goal_offset=int(goal_offset),
        seed=int(seed),
        context_manifest_path=Path(context_manifest),
        min_initial_goal_distance_m=float(min_initial_goal_distance_m),
        require_initial_clearance=bool(require_initial_clearance),
    )
    if not contexts:
        raise ValueError(f"context manifest contains no contexts for scene {scene_name!r}")
    scene_paths = {context.scene_path for context in contexts}
    if len(scene_paths) != 1:
        raise ValueError("candidate collection requires contexts from exactly one scene")

    scene_output = output_root / scene_name
    scene_output.mkdir(parents=True, exist_ok=True)
    runtime: IsaacNavigationRuntime | None = None
    published = 0
    errors: list[dict[str, object]] = []

    def write_summary() -> dict[str, object]:
        summary: dict[str, object] = {
            "schema_version": "candidate_rollout_collection_v1",
            "scene": scene_name,
            "seed": int(seed),
            "candidate_count": int(candidate_count),
            "horizon": int(horizon),
            "action_protocol_id": action_protocol.protocol_id,
            "controls_per_block": action_protocol.controls_per_block,
            "num_contexts": len(contexts),
            "published_contexts": int(published),
            "errors": errors,
        }
        (scene_output / "collection_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
        )
        return summary

    try:
        first_data = _load_episode(str(dataset.episode_paths[contexts[0].episode_index]))
        metadata = first_data.get("metadata")
        if not isinstance(metadata, Mapping):
            raise RuntimeError("episode metadata must be a mapping")
        runtime = IsaacNavigationRuntime(
            scene_path=contexts[0].scene_path,
            image_size=(224, 224),
            ball_radius_m=float(metadata.get("ball_radius_m", 0.4)),
            device=str(device),
        )
        for context_index, context in enumerate(contexts):
            try:
                episode_path = Path(dataset.episode_paths[context.episode_index])
                data = _load_episode(str(episode_path))
                states = data.get("states")
                episode_metadata = data.get("metadata")
                if not isinstance(states, Mapping) or not isinstance(episode_metadata, Mapping):
                    raise RuntimeError("episode data must contain states and metadata mappings")
                source_position = _finite_vector(states["pos"][context.start_state_index, :2], (2,), "source_position_xy")
                source_yaw = _yaw_from_wxyz(states["quat"][context.start_state_index])
                goal_xy = _finite_vector(states["pos"][context.goal_state_index, :2], (2,), "goal_xy")
                goal_relative_xy = pointgoal_in_base(source_position, source_yaw, goal_xy)
                actions = fixed_candidate_action_sequences(
                    horizon,
                    candidate_count,
                    action_protocol,
                    int(seed) + 1009 * context_index,
                )
                pose_labels = np.zeros((actions.shape[0], _HORIZON, 4), dtype=np.float32)
                distance_labels = np.zeros((actions.shape[0],), dtype=np.float32)
                collision_labels = np.zeros((actions.shape[0], _HORIZON), dtype=np.uint8)
                clearance_labels = np.full((actions.shape[0], _HORIZON), np.inf, dtype=np.float32)
                for candidate_index, candidate in enumerate(actions):
                    runtime.reset_to_context(_state_row(states, context.start_state_index))
                    for horizon_index, action_block in enumerate(candidate):
                        observation = runtime.execute_action_block(action_block)
                        pose_labels[candidate_index, horizon_index] = _relative_pose_from_observation(
                            source_position,
                            source_yaw,
                            observation.position_xy,
                            observation.yaw,
                        )
                        distance_labels[candidate_index] = float(np.linalg.norm(goal_xy - observation.position_xy))
                        collision_labels[candidate_index, horizon_index] = int(bool(observation.collision))
                        clearance_labels[candidate_index, horizon_index] = float(observation.clearance_m)
                metadata_out = {
                    "schema_version": "candidate_rollout_v1",
                    "source_episode": _source_episode_relative_path(episode_path, data_root),
                    "context_start": int(context.context_start),
                    "history_frames": 5,
                    "rollout_horizon": 5,
                    "action_block": action_protocol.controls_per_block,
                    "action_protocol_id": action_protocol.protocol_id,
                    "controls_per_block": action_protocol.controls_per_block,
                    "goal_relative_xy": goal_relative_xy.tolist(),
                    "scene": str(episode_metadata.get("scene", context.scene_path)),
                    "seed": int(seed) + 1009 * context_index,
                }
                write_candidate_context_atomic(
                    scene_output / f"context_{context_index:04d}",
                    metadata_out,
                    {
                        "action": actions,
                        "future_relative_pose": pose_labels,
                        "goal_distance_m": distance_labels,
                        "collision": collision_labels,
                        "clearance_m": clearance_labels,
                    },
                )
                published += 1
            except Exception as exc:
                errors.append(
                    {
                        "context_index": int(context_index),
                        "episode_index": int(context.episode_index),
                        "context_start": int(context.context_start),
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
    finally:
        # Isaac Kit may terminate the process from app.close(); publish the
        # audit summary before invoking it so completed contexts remain visible.
        write_summary()
        if runtime is not None:
            runtime.close()
    summary = write_summary()
    if errors:
        raise RuntimeError(f"candidate collection failed for {len(errors)} contexts; see {scene_output / 'collection_summary.json'}")
    return summary


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formal_data_root", type=Path, required=True)
    parser.add_argument("--candidate_root", type=Path, required=True)
    parser.add_argument("--context_manifest", type=Path, required=True)
    parser.add_argument("--scene", required=True, help="scene selector, e.g. scene_easy_8")
    parser.add_argument("--candidate_count", type=int, default=16)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--goal_offset", type=int, default=20)
    parser.add_argument("--min_initial_goal_distance_m", type=float, default=0.5)
    parser.add_argument("--allow_initial_collision", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        summary = collect_candidate_contexts(
            formal_data_root=args.formal_data_root,
            candidate_root=args.candidate_root,
            context_manifest=args.context_manifest,
            scene_name=args.scene,
            candidate_count=args.candidate_count,
            horizon=args.horizon,
            seed=args.seed,
            device=args.device,
            goal_offset=args.goal_offset,
            min_initial_goal_distance_m=args.min_initial_goal_distance_m,
            require_initial_clearance=not args.allow_initial_collision,
        )
    except Exception as exc:
        print(f"candidate collection failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
