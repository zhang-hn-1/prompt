"""Validation and atomic persistence for same-context candidate rollouts."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

import numpy as np

from .action_protocol import LEGACY_ACTION_PROTOCOL_ID, declared_action_protocol_id


SCHEMA_VERSION = "candidate_rollout_v1"
REQUIRED_ARRAYS = frozenset(
    {"action", "future_relative_pose", "goal_distance_m", "collision", "clearance_m"}
)


def _finite_scalar(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) < 1:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def _nonnegative_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return int(value)


@dataclass(frozen=True)
class CandidateContextRef:
    """Metadata identifying one frozen history context in the formal data root."""

    schema_version: str
    source_episode: str
    context_start: int
    history_frames: int
    rollout_horizon: int
    action_block: int
    action_protocol_id: str
    controls_per_block: int
    goal_relative_xy: tuple[float, float]
    scene: str
    seed: int

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "CandidateContextRef":
        if not isinstance(values, Mapping):
            raise ValueError("candidate context metadata must be a mapping")
        if values.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
        source = values.get("source_episode")
        if not isinstance(source, str) or not source or source.startswith(("/", "\\")):
            raise ValueError("source_episode must be a relative path")
        if "\\" in source or re.match(r"^[A-Za-z]:", source):
            raise ValueError("source_episode must be a relative path")
        source_path = Path(source)
        if source_path.is_absolute() or ".." in source_path.parts:
            raise ValueError("source_episode must not contain path traversal")
        context_start = _nonnegative_int(values.get("context_start"), "context_start")
        history_frames = _positive_int(values.get("history_frames"), "history_frames")
        rollout_horizon = _positive_int(values.get("rollout_horizon"), "rollout_horizon")
        action_block = _positive_int(values.get("action_block"), "action_block")
        if history_frames != 5 or rollout_horizon != 5 or action_block != 5:
            raise ValueError("candidate context protocol requires history=5, horizon=5, action_block=5")
        action_protocol_id = declared_action_protocol_id(values)
        raw_controls = values.get("controls_per_block")
        if raw_controls is None:
            if action_protocol_id != LEGACY_ACTION_PROTOCOL_ID:
                raise ValueError(
                    "controls_per_block is required for labelled action protocols"
                )
            controls_per_block = action_block
        else:
            controls_per_block = _positive_int(raw_controls, "controls_per_block")
        if controls_per_block != action_block:
            raise ValueError("controls_per_block must equal action_block")
        goal = np.asarray(values.get("goal_relative_xy"), dtype=np.float64)
        if goal.shape != (2,) or not np.isfinite(goal).all():
            raise ValueError("goal_relative_xy must be finite with shape (2,)")
        scene = values.get("scene")
        if not isinstance(scene, str) or not scene:
            raise ValueError("scene must be a non-empty string")
        seed = _nonnegative_int(values.get("seed"), "seed")
        return cls(
            schema_version=SCHEMA_VERSION,
            source_episode=source,
            context_start=context_start,
            history_frames=history_frames,
            rollout_horizon=rollout_horizon,
            action_block=action_block,
            action_protocol_id=action_protocol_id,
            controls_per_block=controls_per_block,
            goal_relative_xy=(float(goal[0]), float(goal[1])),
            scene=scene,
            seed=seed,
        )

    @classmethod
    def from_json(cls, path: Path) -> "CandidateContextRef":
        try:
            values = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot read candidate context metadata: {path}") from exc
        return cls.from_mapping(values)


def _validate_clearance(values: object, shape: tuple[int, int]) -> np.ndarray:
    raw = np.asarray(values)
    if raw.shape != shape:
        raise ValueError(f"clearance_m must have shape {shape}")
    numeric = raw.astype(np.float64, copy=False)
    if np.isnan(numeric).any() or np.isneginf(numeric).any() or (numeric < 0.0).any():
        raise ValueError("clearance_m must be non-negative and may only use positive infinity")
    return numeric.astype(np.float32, copy=True)


def validate_candidate_arrays(
    arrays: Mapping[str, object],
    *,
    candidate_count: int | None = None,
    horizon: int = 5,
    action_dim: int = 10,
) -> dict[str, np.ndarray]:
    """Validate and copy the fixed-shape candidate rollout arrays."""

    if not isinstance(arrays, Mapping):
        raise ValueError("candidate arrays must be a mapping")
    missing = REQUIRED_ARRAYS.difference(arrays)
    if missing:
        raise ValueError(f"candidate arrays are missing: {sorted(missing)}")
    unknown = set(arrays).difference(REQUIRED_ARRAYS)
    if unknown:
        raise ValueError(f"candidate arrays contain unknown keys: {sorted(unknown)}")
    if isinstance(horizon, bool) or int(horizon) != 5 or int(action_dim) != 10:
        raise ValueError("candidate protocol requires horizon=5 and action_dim=10")

    action = np.asarray(arrays["action"], dtype=np.float64)
    if action.ndim != 3 or action.shape[1:] != (5, 10):
        raise ValueError("action must have shape (K, 5, 10)")
    k = int(action.shape[0])
    if k < 8 or (candidate_count is not None and k != int(candidate_count)):
        raise ValueError("candidate count must be at least 8 and match candidate_count when provided")
    if not np.isfinite(action).all():
        raise ValueError("action must contain finite values")

    pose = np.asarray(arrays["future_relative_pose"], dtype=np.float64)
    if pose.shape != (k, 5, 4) or not np.isfinite(pose).all():
        raise ValueError("future_relative_pose must be finite with shape (K, 5, 4)")
    yaw_norm = np.linalg.norm(pose[..., 2:4], axis=-1)
    if np.any(np.abs(yaw_norm - 1.0) > 1e-3):
        raise ValueError("future_relative_pose yaw sin/cos must have unit norm")

    distance = np.asarray(arrays["goal_distance_m"], dtype=np.float64)
    if distance.shape != (k,) or not np.isfinite(distance).all() or (distance < 0.0).any():
        raise ValueError("goal_distance_m must be finite, non-negative, and shaped (K,)")

    collision_raw = np.asarray(arrays["collision"])
    if collision_raw.shape != (k, 5) or not np.isin(collision_raw, (0, 1)).all():
        raise ValueError("collision must be binary with shape (K, 5)")

    clearance = _validate_clearance(arrays["clearance_m"], (k, 5))
    return {
        "action": action.astype(np.float32, copy=True),
        "future_relative_pose": pose.astype(np.float32, copy=True),
        "goal_distance_m": distance.astype(np.float32, copy=True),
        "collision": collision_raw.astype(np.uint8, copy=True),
        "clearance_m": clearance,
    }


def load_candidate_context(
    path: str | Path,
    formal_data_root: str | Path,
) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    """Load one finalized context and resolve its source episode safely."""

    context_dir = Path(path)
    if not context_dir.is_dir() or context_dir.name.endswith(".partial"):
        raise ValueError(f"candidate context is not finalized: {context_dir}")
    metadata_path = context_dir / "context.json"
    arrays_path = context_dir / "candidates.npz"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    ref = CandidateContextRef.from_mapping(metadata)
    root = Path(formal_data_root).resolve()
    source_episode = (root / ref.source_episode).resolve()
    try:
        source_episode.relative_to(root)
    except ValueError as exc:
        raise ValueError("source_episode escapes formal_data_root") from exc
    if not (source_episode / "states.npz").is_file() or not (source_episode / "metadata.json").is_file():
        raise FileNotFoundError(f"source episode does not exist: {source_episode}")
    with np.load(arrays_path, allow_pickle=False) as loaded:
        arrays = validate_candidate_arrays({name: loaded[name] for name in loaded.files})
    return dict(metadata), arrays


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"value is not JSON serializable: {type(value).__name__}")


def write_candidate_context_atomic(
    output_dir: str | Path,
    metadata: Mapping[str, object],
    arrays: Mapping[str, object],
) -> Path:
    """Write one candidate context to a partial directory and atomically publish it."""

    final_dir = Path(output_dir)
    if final_dir.exists():
        raise FileExistsError(f"candidate context already exists: {final_dir}")
    CandidateContextRef.from_mapping(metadata)
    validated = validate_candidate_arrays(arrays)
    partial_dir = final_dir.with_name(final_dir.name + ".partial")
    if partial_dir.exists():
        raise FileExistsError(f"partial candidate context already exists: {partial_dir}")
    partial_dir.parent.mkdir(parents=True, exist_ok=True)
    partial_dir.mkdir()
    try:
        (partial_dir / "context.json").write_text(
            json.dumps(dict(metadata), indent=2, sort_keys=True, default=_json_default) + "\n",
            encoding="utf-8",
        )
        np.savez_compressed(partial_dir / "candidates.npz", **validated)
        loaded_metadata = json.loads((partial_dir / "context.json").read_text(encoding="utf-8"))
        CandidateContextRef.from_mapping(loaded_metadata)
        with np.load(partial_dir / "candidates.npz", allow_pickle=False) as loaded:
            validate_candidate_arrays({name: loaded[name] for name in loaded.files})
        partial_dir.replace(final_dir)
    except Exception:
        # Keep the partial directory and its error context for audit/debugging.
        raise
    return final_dir
