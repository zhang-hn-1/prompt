"""Offline integrity checks for finalized BEV collection episodes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable

import numpy as np
from PIL import Image

from collection_schema import (
    PHYSICS_DT_S,
    STEREO_DT_S,
    STEREO_STEP_INTERVAL,
    STEREO_SYNC_TOLERANCE_S,
    TIME_TOLERANCE_S,
    load_contract,
)

_BALL_WM_ROOT = Path(__file__).resolve().parents[1]
if str(_BALL_WM_ROOT) not in sys.path:
    sys.path.insert(0, str(_BALL_WM_ROOT))
from training.action_protocol import (
    ActionProtocol,
    declared_action_protocol_id,
)


_STATE_SCALARS = {
    "physics_step": np.dtype(np.int64),
    "sim_time": np.dtype(np.float64),
    "state_timestamp": np.dtype(np.float64),
    "action_command_timestamp": np.dtype(np.float64),
    "action_applied_timestamp": np.dtype(np.float64),
}
_STATE_VECTORS = {
    "pos": (3,),
    "quat": (4,),
    "lin_vel": (3,),
    "ang_vel": (3,),
    "joint_pos": (3,),
    "joint_vel": (3,),
    "commanded_action": (2,),
    "applied_action": (2,),
}
_STEREO_SCALARS = {
    "stereo_frame_id": np.dtype(np.int64),
    "stereo_physics_step": np.dtype(np.int64),
    "left_timestamp": np.dtype(np.float64),
    "right_timestamp": np.dtype(np.float64),
    "collision": np.dtype(bool),
    "minimum_obstacle_distance": np.dtype(np.float32),
}
_STEREO_VECTORS = {
    "cam_left_pose": (4, 4),
    "cam_right_pose": (4, 4),
}
_FILENAMES = ("left_filename", "right_filename")
_BEV_SHAPE = (80, 80)
_RGB_SHAPE = (480, 640)
_CONTRACT_PATH = _BALL_WM_ROOT / "configs" / "data_contract.yaml"
_HIGH_ANGLE_THRESHOLD_DEG = 16.1
_ACTIVE_ACTION_PROTOCOL = ActionProtocol.from_contract(load_contract(_CONTRACT_PATH))
_AUXILIARY_BIN_WIDTH_DEG = 5.0
_AUXILIARY_MINIMUM_DEG = math.floor(
    (math.degrees(_ACTIVE_ACTION_PROTOCOL.minimum[1]) + 1e-6)
    / _AUXILIARY_BIN_WIDTH_DEG
) * _AUXILIARY_BIN_WIDTH_DEG
_AUXILIARY_MAXIMUM_DEG = math.ceil(
    (math.degrees(_ACTIVE_ACTION_PROTOCOL.maximum[1]) - 1e-6)
    / _AUXILIARY_BIN_WIDTH_DEG
) * _AUXILIARY_BIN_WIDTH_DEG
_AUXILIARY_HISTOGRAM_EDGES_DEG = np.arange(
    _AUXILIARY_MINIMUM_DEG,
    _AUXILIARY_MAXIMUM_DEG + _AUXILIARY_BIN_WIDTH_DEG,
    _AUXILIARY_BIN_WIDTH_DEG,
    dtype=np.float64,
)


def _error(errors: list[str], message: str) -> None:
    errors.append(message)


def _validate_npz_fields(
    arrays: dict[str, np.ndarray], errors: list[str]
) -> tuple[int | None, int | None, set[str]]:
    """Check fields and return only names safe for dependent array operations."""

    required = set(_STATE_SCALARS) | set(_STATE_VECTORS) | set(_STEREO_SCALARS) | set(_STEREO_VECTORS) | set(_FILENAMES)
    for field in sorted(required - arrays.keys()):
        _error(errors, f"missing required NPZ field {field}")

    n: int | None = None
    valid: set[str] = set()
    for field, dtype in _STATE_SCALARS.items():
        array = arrays.get(field)
        if array is None:
            continue
        if array.ndim != 1:
            _error(errors, f"{field} must have shape (N,)")
            continue
        if array.dtype != dtype:
            _error(errors, f"{field} must have dtype {dtype.name}, got {array.dtype}")
            continue
        n = array.shape[0] if n is None else n
        if n != array.shape[0]:
            _error(errors, f"{field} must have {n} rows")
            continue
        if field != "physics_step" and not np.all(np.isfinite(array)):
            _error(errors, f"{field} must contain only finite values")
            continue
        valid.add(field)
    for field, trailing_shape in _STATE_VECTORS.items():
        array = arrays.get(field)
        if array is None:
            continue
        if array.ndim != len(trailing_shape) + 1 or array.shape[1:] != trailing_shape:
            _error(errors, f"{field} must have shape (N, {', '.join(map(str, trailing_shape))})")
            continue
        if array.dtype != np.dtype(np.float32):
            _error(errors, f"{field} must have dtype float32, got {array.dtype}")
            continue
        n = array.shape[0] if n is None else n
        if n != array.shape[0]:
            _error(errors, f"{field} must have {n} rows")
            continue
        if not np.all(np.isfinite(array)):
            _error(errors, f"{field} must contain only finite values")
            continue
        valid.add(field)

    m: int | None = None
    for field, dtype in _STEREO_SCALARS.items():
        array = arrays.get(field)
        if array is None:
            continue
        if array.ndim != 1:
            _error(errors, f"{field} must have shape (M,)")
            continue
        if array.dtype != dtype:
            _error(errors, f"{field} must have dtype {dtype.name}, got {array.dtype}")
            continue
        m = array.shape[0] if m is None else m
        if m != array.shape[0]:
            _error(errors, f"{field} must have {m} rows")
            continue
        if field in ("left_timestamp", "right_timestamp") and not np.all(np.isfinite(array)):
            _error(errors, f"{field} must contain only finite values")
            continue
        if field == "minimum_obstacle_distance" and (
            np.any(np.isnan(array)) or np.any(np.isneginf(array))
        ):
            _error(
                errors,
                "minimum_obstacle_distance must contain only finite values or positive infinity",
            )
            continue
        valid.add(field)
    for field, trailing_shape in _STEREO_VECTORS.items():
        array = arrays.get(field)
        if array is None:
            continue
        if array.ndim != len(trailing_shape) + 1 or array.shape[1:] != trailing_shape:
            _error(errors, f"{field} must have shape (M, 4, 4)")
            continue
        if array.dtype != np.dtype(np.float32):
            _error(errors, f"{field} must have dtype float32, got {array.dtype}")
            continue
        m = array.shape[0] if m is None else m
        if m != array.shape[0]:
            _error(errors, f"{field} must have {m} rows")
            continue
        if not np.all(np.isfinite(array)):
            _error(errors, f"{field} must contain only finite values")
            continue
        valid.add(field)
    for field in _FILENAMES:
        array = arrays.get(field)
        if array is None:
            continue
        if array.ndim != 1:
            _error(errors, f"{field} must have shape (M,)")
            continue
        if array.dtype.kind not in "SU":
            _error(errors, f"{field} must contain strings")
            continue
        m = array.shape[0] if m is None else m
        if m != array.shape[0]:
            _error(errors, f"{field} must have {m} rows")
            continue
        valid.add(field)
    if {"collision", "minimum_obstacle_distance"}.issubset(valid):
        expected_collision = arrays["minimum_obstacle_distance"] <= 0.0
        if not np.array_equal(arrays["collision"], expected_collision):
            _error(errors, "collision must equal (minimum_obstacle_distance <= 0)")
    return n, m, valid


def _strict_fixed_interval(values: np.ndarray, interval: float, name: str, errors: list[str]) -> None:
    if values.size > 1 and not np.allclose(np.diff(values), interval, atol=TIME_TOLERANCE_S, rtol=0.0):
        _error(errors, f"{name} must advance at exactly {interval:g} s")


def _audit_cadence(
    arrays: dict[str, np.ndarray], n: int | None, m: int | None, valid: set[str], errors: list[str]
) -> None:
    state_required = {"physics_step", "state_timestamp", "sim_time"}
    if n is None or not state_required.issubset(valid):
        return
    state_steps = arrays.get("physics_step")
    state_times = arrays.get("state_timestamp")
    sim_times = arrays.get("sim_time")
    if any(value is None for value in (state_steps, state_times, sim_times)):
        return
    if n == 0:
        _error(errors, "episode must contain at least one state sample")
        return
    if not np.all(np.diff(state_steps) == 1):
        _error(errors, "physics_step must advance by one at 50 Hz")
    if not np.all(np.diff(state_times) > 0):
        _error(errors, "state_timestamp must be strictly increasing")
    if not np.all(np.diff(sim_times) > 0):
        _error(errors, "sim_time must be strictly increasing")
    _strict_fixed_interval(state_times, PHYSICS_DT_S, "state_timestamp", errors)
    _strict_fixed_interval(sim_times, PHYSICS_DT_S, "sim_time", errors)
    if not np.allclose(state_times, sim_times, atol=TIME_TOLERANCE_S, rtol=0.0):
        _error(errors, "state_timestamp must align with sim_time")
    for field in ("action_command_timestamp", "action_applied_timestamp"):
        if field not in valid:
            continue
        timestamps = arrays[field]
        if not np.all(np.diff(timestamps) > 0):
            _error(errors, f"{field} must be strictly increasing")
        _strict_fixed_interval(timestamps, PHYSICS_DT_S, field, errors)
        if not (
            np.allclose(timestamps, state_times, atol=TIME_TOLERANCE_S, rtol=0.0)
            and np.allclose(timestamps, sim_times, atol=TIME_TOLERANCE_S, rtol=0.0)
        ):
            _error(errors, f"{field} must align with state_timestamp and sim_time")
    stereo_required = {
        "stereo_frame_id",
        "stereo_physics_step",
        "left_timestamp",
        "right_timestamp",
    }
    if m is None or not stereo_required.issubset(valid):
        return
    stereo_steps = arrays["stereo_physics_step"]
    frame_ids = arrays["stereo_frame_id"]
    left_times = arrays["left_timestamp"]
    right_times = arrays["right_timestamp"]
    if m == 0:
        expected_steps = state_steps[state_steps % STEREO_STEP_INTERVAL == 0]
        if expected_steps.size:
            _error(errors, "missing 10 Hz stereo samples for state cadence")
        return
    if not np.all(np.diff(frame_ids) == 1):
        _error(errors, "stereo_frame_id must be consecutive")
    if np.any(stereo_steps % STEREO_STEP_INTERVAL):
        _error(errors, "stereo_physics_step must lie on 10 Hz boundaries")
    if not np.all(np.diff(stereo_steps) == STEREO_STEP_INTERVAL):
        _error(errors, "stereo_physics_step must advance exactly 5 physics steps")
    if not np.allclose(left_times, right_times, atol=STEREO_SYNC_TOLERANCE_S, rtol=0.0):
        _error(errors, "left_timestamp and right_timestamp must be synchronized")
    _strict_fixed_interval(left_times, STEREO_DT_S, "left_timestamp", errors)
    _strict_fixed_interval(right_times, STEREO_DT_S, "right_timestamp", errors)

    expected_steps = state_steps[state_steps % STEREO_STEP_INTERVAL == 0]
    if not np.array_equal(stereo_steps, expected_steps):
        _error(errors, "stereo samples must match every 10 Hz state boundary (except only the final state tail)")
    time_by_step = dict(zip(state_steps.tolist(), state_times.tolist()))
    for step, timestamp in zip(stereo_steps.tolist(), left_times.tolist()):
        state_timestamp = time_by_step.get(step)
        if state_timestamp is None or abs(timestamp - state_timestamp) > 0.005:
            _error(errors, "stereo timestamps must align with their state physics steps")
            break


def _audit_png(path: Path, expected_mode: str, errors: list[str]) -> np.ndarray | None:
    if not path.is_file():
        _error(errors, f"missing {path.name}")
        return None
    try:
        with Image.open(path) as image:
            image.load()
            if image.format != "PNG":
                _error(errors, f"{path.name} must be a PNG")
            if image.mode != expected_mode:
                _error(errors, f"{path.name} must have {expected_mode} pixels")
            return np.asarray(image)
    except (OSError, ValueError) as exc:
        _error(errors, f"cannot read {path.name}: {exc}")
        return None


def _image_shape_from_metadata(
    metadata: dict[str, object] | None, errors: list[str]
) -> tuple[int, int] | None:
    if metadata is None:
        return None
    height = metadata.get("image_height_px")
    width = metadata.get("image_width_px")
    if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
        _error(errors, "metadata.json image_height_px must be a positive integer")
        return None
    if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
        _error(errors, "metadata.json image_width_px must be a positive integer")
        return None
    shape = (height, width)
    if shape != _RGB_SHAPE:
        _error(errors, f"metadata.json image dimensions must equal schema {_RGB_SHAPE}")
    return shape


def _depth_enabled(metadata: dict[str, object] | None, errors: list[str]) -> bool:
    if metadata is None:
        return False
    enabled = metadata.get("save_depth")
    if not isinstance(enabled, bool):
        _error(errors, "metadata.json save_depth must be a boolean")
        return False
    return enabled


def _audit_depth(path: Path, expected_shape: tuple[int, int], errors: list[str]) -> None:
    if not path.is_file():
        _error(errors, f"missing {path.name}")
        return
    try:
        payload = np.load(path, allow_pickle=False)
    except (OSError, ValueError, EOFError) as exc:
        _error(errors, f"cannot read {path.name}: {exc}")
        return
    if not isinstance(payload, np.ndarray):
        try:
            if isinstance(payload, np.lib.npyio.NpzFile):
                payload.close()
        finally:
            _error(errors, f"{path.name} must contain a NumPy ndarray, not an NPZ container")
        return
    depth = payload
    if depth.dtype != np.dtype(np.float32):
        _error(errors, f"{path.name} must have dtype float32")
    if depth.ndim != 2 or depth.shape != expected_shape:
        _error(errors, f"{path.name} must have shape {expected_shape}")


_ASSET_PATTERNS = (
    re.compile(r"left_\d{6}\.png"),
    re.compile(r"right_\d{6}\.png"),
    re.compile(r"geometry_bev_\d{6}\.png"),
    re.compile(r"current_visible_\d{6}\.png"),
    re.compile(r"history_observed_\d{6}\.png"),
    re.compile(r"left_depth_\d{6}\.npy"),
    re.compile(r"right_depth_\d{6}\.npy"),
    re.compile(r"height_min_bev_\d{6}\.npy"),
    re.compile(r"height_max_bev_\d{6}\.npy"),
    re.compile(r"height_valid_bev_\d{6}\.png"),
)


def _audit_orphan_assets(episode: Path, expected_names: set[str], errors: list[str]) -> None:
    for path in episode.iterdir():
        if any(pattern.fullmatch(path.name) for pattern in _ASSET_PATTERNS) and path.name not in expected_names:
            _error(errors, f"orphan {path.name}")


def _audit_visual_assets(
    episode: Path,
    arrays: dict[str, np.ndarray],
    m: int | None,
    valid: set[str],
    metadata: dict[str, object] | None,
    errors: list[str],
) -> None:
    if m is None or "stereo_frame_id" not in valid:
        return
    frame_ids = arrays["stereo_frame_id"]
    left_names = arrays.get("left_filename") if "left_filename" in valid else None
    right_names = arrays.get("right_filename") if "right_filename" in valid else None
    image_shape = _image_shape_from_metadata(metadata, errors)
    depth_enabled = _depth_enabled(metadata, errors)
    elevation_enabled = _elevation_enabled(metadata, errors)
    expected_names: set[str] = set()
    for index, frame_id_raw in enumerate(frame_ids.tolist()):
        frame_id = int(frame_id_raw)
        left_name = f"left_{frame_id:06d}.png"
        right_name = f"right_{frame_id:06d}.png"
        expected_names.update((left_name, right_name))
        if left_names is not None and index < len(left_names) and str(left_names[index]) != left_name:
            _error(errors, f"left_filename[{index}] must be {left_name}")
        if right_names is not None and index < len(right_names) and str(right_names[index]) != right_name:
            _error(errors, f"right_filename[{index}] must be {right_name}")
        left = _audit_png(episode / left_name, "RGB", errors)
        right = _audit_png(episode / right_name, "RGB", errors)
        if left is not None and right is not None and left.shape != right.shape:
            _error(errors, f"{left_name} and {right_name} must have the same shape")
        decoded_rgb_shape: tuple[int, int] | None = None
        for name, image in ((left_name, left), (right_name, right)):
            if image is None or image.ndim != 3 or image.shape[2] != 3:
                continue
            decoded_rgb_shape = image.shape[:2] if decoded_rgb_shape is None else decoded_rgb_shape
            if image_shape is not None and image.shape[:2] != image_shape:
                _error(errors, f"metadata.json image dimensions must match {name}")
        for prefix, allowed in (("geometry_bev", {0, 1, 2}), ("current_visible", {0, 1}), ("history_observed", {0, 1})):
            name = f"{prefix}_{frame_id:06d}.png"
            expected_names.add(name)
            image = _audit_png(episode / name, "L", errors)
            if image is None:
                continue
            if image.shape != _BEV_SHAPE:
                _error(errors, f"{name} must have shape {_BEV_SHAPE}")
            elif not set(np.unique(image).tolist()).issubset(allowed):
                _error(errors, f"{name} has values outside {sorted(allowed)}")
        if depth_enabled:
            for prefix in ("left_depth", "right_depth"):
                name = f"{prefix}_{frame_id:06d}.npy"
                expected_names.add(name)
                expected_depth_shape = decoded_rgb_shape or image_shape
                if expected_depth_shape is not None:
                    _audit_depth(episode / name, expected_depth_shape, errors)
        if elevation_enabled:
            min_name = f"height_min_bev_{frame_id:06d}.npy"
            max_name = f"height_max_bev_{frame_id:06d}.npy"
            valid_name = f"height_valid_bev_{frame_id:06d}.png"
            expected_names.update((min_name, max_name, valid_name))
            minimum = _audit_elevation_array(episode / min_name, errors)
            maximum = _audit_elevation_array(episode / max_name, errors)
            valid_image = _audit_png(episode / valid_name, "L", errors)
            valid: np.ndarray | None = None
            if valid_image is not None:
                if valid_image.shape != _BEV_SHAPE:
                    _error(errors, f"{valid_name} must have shape {_BEV_SHAPE}")
                elif not np.isin(valid_image, (0, 1)).all():
                    _error(errors, f"{valid_name} must contain only 0 and 1 values")
                else:
                    valid = valid_image
            if minimum is not None and maximum is not None and valid is not None:
                valid_cells = valid == 1
                if np.any(minimum[valid_cells] > maximum[valid_cells]):
                    _error(errors, f"{min_name} and {max_name} have reversed valid bounds")
                invalid_cells = ~valid_cells
                if np.any(minimum[invalid_cells] != 0.0):
                    _error(errors, f"{min_name} must equal 0.0 at invalid cells")
                if np.any(maximum[invalid_cells] != 0.0):
                    _error(errors, f"{max_name} must equal 0.0 at invalid cells")
    _audit_orphan_assets(episode, expected_names, errors)


def _elevation_enabled(metadata: dict[str, object] | None, errors: list[str]) -> bool:
    """Read the optional elevation switch; absent metadata is legacy-disabled."""
    if metadata is None or "elevation_enabled" not in metadata:
        return False
    enabled = metadata["elevation_enabled"]
    if not isinstance(enabled, bool):
        _error(errors, "metadata.json elevation_enabled must be a boolean")
        return False
    return enabled


def _audit_elevation_array(path: Path, errors: list[str]) -> np.ndarray | None:
    if not path.is_file():
        _error(errors, f"missing {path.name}")
        return None
    try:
        payload = np.load(path, allow_pickle=False)
    except (OSError, ValueError, EOFError) as exc:
        _error(errors, f"cannot read {path.name}: {exc}")
        return None
    if not isinstance(payload, np.ndarray):
        try:
            if isinstance(payload, np.lib.npyio.NpzFile):
                payload.close()
        finally:
            _error(errors, f"{path.name} must contain a NumPy ndarray, not an NPZ container")
        return None
    if payload.dtype != np.dtype(np.float32):
        _error(errors, f"{path.name} must have dtype float32")
    if payload.shape != _BEV_SHAPE:
        _error(errors, f"{path.name} must have shape {_BEV_SHAPE}")
    if not np.all(np.isfinite(payload)):
        _error(errors, f"{path.name} must contain only finite values")
    if payload.dtype != np.dtype(np.float32) or payload.shape != _BEV_SHAPE or not np.all(np.isfinite(payload)):
        return None
    return payload


def auxiliary_action_statistics(applied_action: np.ndarray) -> dict[str, object]:
    """Summarize signed auxiliary commands with stable five-degree bins."""

    actions = np.asarray(applied_action)
    if actions.ndim != 2 or actions.shape[1] != 2 or not np.all(np.isfinite(actions)):
        raise ValueError("applied_action must be a finite array with shape (N, 2)")
    auxiliary_deg = np.rad2deg(actions[:, 1].astype(np.float64))
    histogram_values = np.clip(
        auxiliary_deg,
        _AUXILIARY_HISTOGRAM_EDGES_DEG[0],
        _AUXILIARY_HISTOGRAM_EDGES_DEG[-1],
    )
    counts, edges = np.histogram(
        histogram_values, bins=_AUXILIARY_HISTOGRAM_EDGES_DEG
    )
    high_angle = (
        (np.abs(auxiliary_deg) > _HIGH_ANGLE_THRESHOLD_DEG)
        & (np.abs(auxiliary_deg) <= edges[-1] + 1e-4)
    )
    sample_count = int(actions.shape[0])
    high_count = int(np.count_nonzero(high_angle))
    return {
        "histogram_bin_edges_deg": [int(value) for value in edges.tolist()],
        "histogram_counts": [int(value) for value in counts.tolist()],
        "sample_count": sample_count,
        "high_angle_sample_count": high_count,
        "high_angle_fraction": high_count / sample_count if sample_count else 0.0,
    }


def _audit_action_protocol(
    metadata: dict[str, object] | None,
    arrays: dict[str, np.ndarray],
    valid: set[str],
    errors: list[str],
) -> tuple[str, dict[str, object] | None]:
    metadata_record = metadata or {}
    try:
        protocol_id = declared_action_protocol_id(metadata_record)
    except ValueError as exc:
        _error(errors, f"metadata.json {exc}")
        protocol_id = "invalid"

    statistics = None
    if "applied_action" in valid:
        statistics = auxiliary_action_statistics(arrays["applied_action"])

    active_protocol = _ACTIVE_ACTION_PROTOCOL
    if protocol_id == active_protocol.protocol_id:
        controls = metadata_record.get("controls_per_block")
        if controls != active_protocol.controls_per_block:
            _error(
                errors,
                "metadata.json controls_per_block must match the active V2 contract",
            )
        lower = np.asarray(active_protocol.minimum, dtype=np.float32)
        upper = np.asarray(active_protocol.maximum, dtype=np.float32)
        for field in ("commanded_action", "applied_action"):
            if field in valid and (
                np.any(arrays[field] < lower) or np.any(arrays[field] > upper)
            ):
                _error(errors, f"{field} contains values outside the active action protocol")
        if statistics is not None and statistics["high_angle_sample_count"] == 0:
            _error(
                errors,
                "V2 formal collection has zero high-angle auxiliary samples "
                f"(|auxiliary| > {_HIGH_ANGLE_THRESHOLD_DEG:g} deg)",
            )
    return protocol_id, statistics


def audit_episode_report(path: str | Path) -> dict[str, object]:
    """Return integrity errors and action-coverage evidence for one episode."""

    episode = Path(path)
    errors: list[str] = []
    metadata: dict[str, object] | None = None
    if not episode.is_dir():
        return {
            "episode": str(episode),
            "action_protocol_id": "unknown",
            "auxiliary_actions": None,
            "errors": [f"episode directory does not exist: {episode}"],
        }
    metadata_path = episode / "metadata.json"
    if not metadata_path.is_file():
        _error(errors, "missing metadata.json")
    else:
        try:
            with metadata_path.open(encoding="utf-8") as handle:
                loaded_metadata = json.load(handle)
                if not isinstance(loaded_metadata, dict):
                    _error(errors, "metadata.json must contain a JSON object")
                else:
                    metadata = loaded_metadata
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            _error(errors, f"cannot read metadata.json: {exc}")
    states_path = episode / "states.npz"
    if not states_path.is_file():
        protocol_id, statistics = _audit_action_protocol(metadata, {}, set(), errors)
        return {
            "episode": str(episode),
            "action_protocol_id": protocol_id,
            "auxiliary_actions": statistics,
            "errors": errors + ["missing states.npz"],
        }
    try:
        with np.load(states_path, allow_pickle=False) as data:
            arrays = {name: data[name] for name in data.files}
    except (OSError, ValueError) as exc:
        protocol_id, statistics = _audit_action_protocol(metadata, {}, set(), errors)
        return {
            "episode": str(episode),
            "action_protocol_id": protocol_id,
            "auxiliary_actions": statistics,
            "errors": errors + [f"cannot read states.npz: {exc}"],
        }
    n, m, valid = _validate_npz_fields(arrays, errors)
    _audit_cadence(arrays, n, m, valid, errors)
    _audit_visual_assets(episode, arrays, m, valid, metadata, errors)
    protocol_id, statistics = _audit_action_protocol(metadata, arrays, valid, errors)
    return {
        "episode": str(episode),
        "action_protocol_id": protocol_id,
        "auxiliary_actions": statistics,
        "errors": errors,
    }


def audit_episode(path: str | Path) -> list[str]:
    """Return all contract violations found in one finalized episode directory."""

    return list(audit_episode_report(path)["errors"])


def _episode_paths(path: Path) -> Iterable[Path]:
    if (path / "states.npz").is_file():
        return (path,)
    episodes: list[Path] = []
    for child in sorted(path.iterdir()):
        if child.is_dir() and not child.name.endswith(".partial"):
            episodes.extend(_episode_paths(child))
    return tuple(episodes)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit finalized BEV collection episodes.")
    parser.add_argument("path", type=Path, help="an episode directory or directory containing episodes")
    args = parser.parse_args(argv)
    if not args.path.is_dir():
        print(f"error: directory does not exist: {args.path}")
        return 1
    episodes = tuple(_episode_paths(args.path))
    if not episodes:
        print(f"error: no finalized episodes found in {args.path}")
        return 1
    failed = False
    reports = [audit_episode_report(episode) for episode in episodes]
    protocol_ids = {str(report["action_protocol_id"]) for report in reports}
    if len(protocol_ids) != 1:
        failed = True
        print(f"{args.path}: mixed action protocols: {sorted(protocol_ids)}")
    for episode, report in zip(episodes, reports):
        print(json.dumps({key: value for key, value in report.items() if key != "errors"}, sort_keys=True))
        errors = report["errors"]
        if errors:
            failed = True
            for error in errors:
                print(f"{episode}: {error}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
