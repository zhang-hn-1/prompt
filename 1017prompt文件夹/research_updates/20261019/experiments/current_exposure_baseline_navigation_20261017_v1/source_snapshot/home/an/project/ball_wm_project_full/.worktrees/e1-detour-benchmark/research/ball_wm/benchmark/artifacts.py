"""Atomic, self-checking audit bundles for Benchmark V1 episodes."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image


REQUIRED_AUDIT_FILES = {
    "manifest.json",
    "environment.json",
    "task_card.png",
    "third_person.mp4",
    "stereo_view.mp4",
    "trajectory_overlay.png",
    "actions.npz",
    "states.npz",
    "metrics.json",
    "human_review.json",
}
REQUIRED_TRAJECTORY_LAYERS = {
    "obstacles",
    "inflated",
    "oracle_path",
    "kinodynamic_reference",
    "actual_path",
    "start",
    "goal",
    "end",
    "collisions",
}
REQUIRED_CERTIFICATION_FIELDS = {
    "planning_status",
    "expanded_nodes",
    "direction_changes",
    "minimum_clearance_m",
    "maximum_cross_track_error_m",
    "mean_cross_track_error_m",
    "failure_subreason",
}
REQUIRED_DYNAMIC_CERTIFICATION_FIELDS = {
    "planner_version",
    "planning_wall_time_s",
    "predicted_duration_s",
    "maximum_main_rate_rad_s2",
    "maximum_auxiliary_rate_rad_s",
}
REQUIRED_DYNAMIC_PLANNER_FIELDS = {
    "planner_version",
    "drive_profile_path",
    "primitive_duration_s",
    "integration_dt_s",
    "main_acceleration_limit_rad_s2",
    "auxiliary_rate_limit_rad_s",
    "maximum_predicted_duration_s",
    "node_budget",
    "planning_timeout_s",
}
REQUIRED_EXECUTION_STATE_FIELDS = {
    "yaw_world_rad",
    "linear_velocity_world_m_s",
    "angular_velocity_world_rad_s",
    "joint_position_rad",
    "joint_velocity_rad_s",
    "applied_action",
}
REQUIRED_POLICY_DIAGNOSTIC_FIELDS = {
    "policy_reference_index",
    "policy_cross_track_error_m",
    "policy_active_drive_direction",
    "policy_desired_drive_direction",
    "policy_pending_drive_direction",
    "policy_stopped_cycles",
    "policy_braking_state_code",
}
ALLOWED_BEHAVIOR_STATUSES = {"approved", "needs_investigation"}
ALLOWED_BEHAVIOR_LABELS = {
    "goal_response",
    "active_avoidance",
    "contact",
    "stuck",
    "oscillation",
    "termination_mismatch",
    "technically_invalid",
}


def artifact_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_rgb_video(
    output_path: str | Path,
    frames: Sequence[np.ndarray],
    *,
    fps: float = 10.0,
    cv2_module: Any | None = None,
) -> Path:
    """Encode equal-sized RGB uint8 frames and fail if MP4 cannot be opened."""

    rate = float(fps)
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("video fps must be finite and positive")
    normalized: list[np.ndarray] = []
    expected_shape: tuple[int, int, int] | None = None
    for index, value in enumerate(frames):
        frame = np.asarray(value)
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"video frame {index} must be RGB uint8 with shape (H, W, 3)")
        if expected_shape is None:
            expected_shape = frame.shape
        elif frame.shape != expected_shape:
            raise ValueError("all video frames must have the same shape")
        normalized.append(frame)
    if not normalized or expected_shape is None:
        raise ValueError("video requires at least one frame")

    if cv2_module is None:
        import cv2 as cv2_module  # type: ignore[no-redef]

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    height, width, _ = expected_shape
    writer = cv2_module.VideoWriter(
        str(destination),
        cv2_module.VideoWriter_fourcc(*"mp4v"),
        rate,
        (width, height),
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f"video encoder could not open output: {destination}")
    try:
        for frame in normalized:
            writer.write(cv2_module.cvtColor(frame, cv2_module.COLOR_RGB2BGR))
    finally:
        writer.release()
    if not destination.is_file() or destination.stat().st_size <= 0:
        raise RuntimeError(f"video encoder produced an empty output: {destination}")
    return destination


def _require_json_finite(value: object, field: str = "JSON") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{field} must contain only finite numeric values")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _require_json_finite(item, f"{field}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _require_json_finite(item, f"{field}[{index}]")


def _reject_json_constant(token: str) -> object:
    raise ValueError(f"JSON must contain finite values, got {token}")


def _load_strict_json(path: Path) -> object:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream, parse_constant=_reject_json_constant)
    _require_json_finite(payload, path.name)
    return payload


def _safe_bundle_name(name: str) -> str:
    if not isinstance(name, str) or not name or Path(name).name != name:
        raise ValueError("artifact name must be one plain filename")
    return name


class AuditBundleWriter:
    """Write into ``.partial`` and publish only a complete valid directory."""

    def __init__(self, run_root: str | Path, episode_id: str) -> None:
        if not isinstance(episode_id, str) or not episode_id.strip() or "/" in episode_id:
            raise ValueError("episode_id must be a portable non-empty identifier")
        self.run_root = Path(run_root)
        self.episode_id = episode_id
        self.partial_dir = self.run_root / f"episode_{episode_id}.partial"
        self.output_dir = self.run_root / f"episode_{episode_id}"

    def begin(self) -> Path:
        self.run_root.mkdir(parents=True, exist_ok=True)
        if self.output_dir.exists():
            raise FileExistsError(f"audit bundle already exists: {self.output_dir}")
        self.partial_dir.mkdir(parents=False, exist_ok=False)
        return self.partial_dir

    def path(self, name: str) -> Path:
        if not self.partial_dir.is_dir():
            raise RuntimeError("begin() must be called before writing artifacts")
        return self.partial_dir / _safe_bundle_name(name)

    def write_json(self, name: str, payload: object) -> Path:
        _require_json_finite(payload)
        destination = self.path(name)
        with destination.open("w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        return destination

    def write_npz(self, name: str, **arrays: object) -> Path:
        if not arrays:
            raise ValueError("NPZ artifact must contain at least one array")
        destination = self.path(name)
        normalized: dict[str, np.ndarray] = {}
        for key, value in arrays.items():
            array = np.asarray(value)
            if array.dtype == object or not (
                np.issubdtype(array.dtype, np.number) or np.issubdtype(array.dtype, np.bool_)
            ):
                raise ValueError(f"NPZ array {key!r} must be numeric")
            normalized[key] = array
        np.savez(destination, **normalized)
        return destination

    def _validate_required_files(self) -> None:
        missing = sorted(
            name
            for name in REQUIRED_AUDIT_FILES
            if not (self.partial_dir / name).is_file()
        )
        if missing:
            raise ValueError(f"missing audit files: {', '.join(missing)}")

    def _validate_json(self) -> None:
        for name in ("manifest.json", "environment.json", "metrics.json", "human_review.json"):
            _load_strict_json(self.partial_dir / name)
        environment = _load_strict_json(self.partial_dir / "environment.json")
        if not isinstance(environment, Mapping):
            raise ValueError("environment.json must contain an object")
        if environment.get("policy") == "oracle_v2" and not bool(
            environment.get("privileged", False)
        ):
            raise ValueError("oracle_v2 environment must be privileged")
        layers = environment.get("trajectory_overlay_layers")
        if not isinstance(layers, list) or not REQUIRED_TRAJECTORY_LAYERS.issubset(layers):
            missing = sorted(REQUIRED_TRAJECTORY_LAYERS.difference(layers or ()))
            raise ValueError(
                "trajectory overlay layers are incomplete; missing: "
                + ", ".join(missing)
            )
        if bool(environment.get("privileged", False)):
            metrics = _load_strict_json(self.partial_dir / "metrics.json")
            certification = (
                metrics.get("certification") if isinstance(metrics, Mapping) else None
            )
            if not isinstance(certification, Mapping):
                raise ValueError("privileged metrics require a certification object")
            missing = sorted(REQUIRED_CERTIFICATION_FIELDS.difference(certification))
            if missing:
                raise ValueError(
                    "certification fields are incomplete; missing: " + ", ".join(missing)
                )
            if environment.get("policy") == "oracle_v2":
                dynamic_missing = sorted(
                    REQUIRED_DYNAMIC_CERTIFICATION_FIELDS.difference(certification)
                )
                if dynamic_missing:
                    raise ValueError(
                        "dynamic certification fields are incomplete; missing: "
                        + ", ".join(dynamic_missing)
                    )
                planner = environment.get("dynamic_planner")
                if not isinstance(planner, Mapping):
                    raise ValueError("oracle_v2 environment requires dynamic_planner parameters")
                planner_missing = sorted(REQUIRED_DYNAMIC_PLANNER_FIELDS.difference(planner))
                if planner_missing:
                    raise ValueError(
                        "dynamic_planner parameters are incomplete; missing: "
                        + ", ".join(planner_missing)
                    )
                if certification.get("planner_version") != "dynamic_lattice_v2":
                    raise ValueError("oracle_v2 certification has an invalid planner_version")
                if planner.get("planner_version") != "dynamic_lattice_v2":
                    raise ValueError("oracle_v2 environment has an invalid planner_version")
                if not isinstance(planner.get("drive_profile_path"), str) or not planner[
                    "drive_profile_path"
                ]:
                    raise ValueError("dynamic_planner drive_profile_path must be non-empty")
                frozen_parameters = {
                    "primitive_duration_s": 1.0,
                    "integration_dt_s": 0.02,
                    "main_acceleration_limit_rad_s2": 0.5,
                    "auxiliary_rate_limit_rad_s": math.radians(30.0),
                    "maximum_predicted_duration_s": 32.0,
                    "node_budget": 500_000,
                    "planning_timeout_s": 60.0,
                }
                for name, expected in frozen_parameters.items():
                    try:
                        actual = float(planner[name])
                    except (TypeError, ValueError) as error:
                        raise ValueError(
                            f"dynamic_planner {name} must match the frozen value"
                        ) from error
                    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
                        raise ValueError(
                            f"dynamic_planner {name} does not match the frozen value"
                        )

    def _validate_npz(self) -> None:
        for name in ("actions.npz", "states.npz"):
            with np.load(self.partial_dir / name, allow_pickle=False) as archive:
                if not archive.files:
                    raise ValueError(f"{name} must contain at least one array")
                for key in archive.files:
                    array = np.asarray(archive[key])
                    if array.dtype == object or not (
                        np.issubdtype(array.dtype, np.number)
                        or np.issubdtype(array.dtype, np.bool_)
                    ):
                        raise ValueError(f"{name}:{key} must be numeric")
                    if not np.all(np.isfinite(array)):
                        raise ValueError(f"{name}:{key} must contain finite values")
        environment = _load_strict_json(self.partial_dir / "environment.json")
        if not isinstance(environment, Mapping) or not bool(
            environment.get("privileged", False)
        ):
            return
        with np.load(self.partial_dir / "states.npz", allow_pickle=False) as archive:
            dynamic_v2 = environment.get("policy") == "oracle_v2"
            if dynamic_v2:
                required = {
                    "dynamic_reference_time_s",
                    "dynamic_reference_pose",
                    "dynamic_reference_speed_m_s",
                    "dynamic_reference_command",
                    "dynamic_reference_drive_direction",
                    "dynamic_reference_index",
                    "dynamic_position_error_m",
                    "dynamic_yaw_error_rad",
                    "dynamic_auxiliary_error_rad",
                }
                missing = sorted(required.difference(archive.files))
                if missing:
                    raise ValueError(
                        "dynamic_reference arrays are incomplete; missing: "
                        + ", ".join(missing)
                    )
                time_s = np.asarray(archive["dynamic_reference_time_s"])
                pose = np.asarray(archive["dynamic_reference_pose"])
                speed = np.asarray(archive["dynamic_reference_speed_m_s"])
                command = np.asarray(archive["dynamic_reference_command"])
                direction = np.asarray(archive["dynamic_reference_drive_direction"])
                if time_s.ndim != 1 or time_s.shape[0] < 2:
                    raise ValueError("dynamic_reference_time_s must have shape (N,), N >= 2")
                if pose.shape != (time_s.shape[0], 3):
                    raise ValueError("dynamic_reference_pose must have shape (N, 3)")
                if speed.shape != time_s.shape or direction.shape != time_s.shape:
                    raise ValueError("dynamic_reference arrays must have the same length")
                if command.shape != (time_s.shape[0], 2):
                    raise ValueError("dynamic_reference_command must have shape (N, 2)")
                intervals = np.diff(time_s)
                if not np.all(intervals > 0.0):
                    raise ValueError("dynamic_reference time must be strictly increasing")
                if not np.all(np.isin(direction, (-1, 0, 1))):
                    raise ValueError(
                        "dynamic_reference drive direction must use -1, 0, or 1"
                    )
                rates = np.abs(np.diff(command, axis=0)) / intervals[:, None]
                maximum_main_rate = float(np.max(rates[:, 0]))
                maximum_auxiliary_rate = float(np.max(rates[:, 1]))
                if maximum_main_rate > 0.5 + 1e-9:
                    raise ValueError("dynamic reference exceeds the frozen main rate")
                if maximum_auxiliary_rate > math.radians(30.0) + 1e-9:
                    raise ValueError("dynamic reference exceeds the frozen auxiliary rate")
                metrics = _load_strict_json(self.partial_dir / "metrics.json")
                certification = metrics.get("certification")
                for name, actual, limit in (
                    ("maximum_main_rate_rad_s2", maximum_main_rate, 0.5),
                    (
                        "maximum_auxiliary_rate_rad_s",
                        maximum_auxiliary_rate,
                        math.radians(30.0),
                    ),
                ):
                    reported = float(certification[name])
                    if reported > limit + 1e-9 or not math.isclose(
                        reported, actual, rel_tol=0.0, abs_tol=1e-9
                    ):
                        raise ValueError(f"dynamic certification {name} does not match reference")
                for name in ("planning_wall_time_s", "predicted_duration_s"):
                    value = float(certification[name])
                    if not math.isfinite(value) or value < 0.0:
                        raise ValueError(f"dynamic certification {name} must be non-negative")
                if float(certification["predicted_duration_s"]) > 32.0 + 1e-9:
                    raise ValueError("dynamic certification predicted_duration_s exceeds 32 s")
                reference_duration_s = float(time_s[-1] - time_s[0])
                if not math.isclose(
                    float(certification["predicted_duration_s"]),
                    reference_duration_s,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                ):
                    raise ValueError(
                        "dynamic certification predicted_duration_s does not match reference"
                    )
            else:
                required = {
                    "kinodynamic_reference_pose",
                    "kinodynamic_reference_drive_direction",
                    "kinodynamic_reference_command",
                }
                missing = sorted(required.difference(archive.files))
                if missing:
                    raise ValueError(
                        "privileged states require kinodynamic_reference arrays; missing: "
                        + ", ".join(missing)
                    )
                pose = np.asarray(archive["kinodynamic_reference_pose"])
                direction = np.asarray(archive["kinodynamic_reference_drive_direction"])
                command = np.asarray(archive["kinodynamic_reference_command"])
                if pose.ndim != 2 or pose.shape[1] != 3 or pose.shape[0] < 2:
                    raise ValueError(
                        "kinodynamic_reference_pose must have shape (N, 3), N >= 2"
                    )
                if direction.ndim != 1 or command.ndim != 2 or command.shape[1] != 2:
                    raise ValueError("kinodynamic_reference arrays have invalid shapes")
                if not (pose.shape[0] == direction.shape[0] == command.shape[0]):
                    raise ValueError("kinodynamic_reference arrays must have the same length")
                if not np.all(np.isin(direction, (-1, 1))):
                    raise ValueError("kinodynamic_reference drive direction must use -1 or 1")
            audit_required = (
                REQUIRED_EXECUTION_STATE_FIELDS | REQUIRED_POLICY_DIAGNOSTIC_FIELDS
            )
            audit_missing = sorted(audit_required.difference(archive.files))
            if audit_missing:
                raise ValueError(
                    "privileged states require execution audit arrays; missing: "
                    + ", ".join(audit_missing)
                )
            state_count = int(np.asarray(archive["position_world_xy"]).shape[0])
            state_shapes = {
                "yaw_world_rad": (state_count,),
                "linear_velocity_world_m_s": (state_count, 3),
                "angular_velocity_world_rad_s": (state_count, 3),
                "joint_position_rad": (state_count, 3),
                "joint_velocity_rad_s": (state_count, 3),
                "applied_action": (state_count, 2),
            }
            for name, expected_shape in state_shapes.items():
                if np.asarray(archive[name]).shape != expected_shape:
                    raise ValueError(
                        f"execution audit array {name} must have shape {expected_shape}"
                    )
            with np.load(self.partial_dir / "actions.npz", allow_pickle=False) as actions:
                action_name = "action_blocks" if "action_blocks" in actions.files else actions.files[0]
                action_count = int(np.asarray(actions[action_name]).shape[0])
            if state_count != action_count + 1:
                raise ValueError("execution audit states must contain one more row than actions")
            for name in REQUIRED_POLICY_DIAGNOSTIC_FIELDS:
                if np.asarray(archive[name]).shape != (action_count,):
                    raise ValueError(
                        f"policy diagnostic array {name} must have shape ({action_count},)"
                    )
            if dynamic_v2:
                for name in (
                    "dynamic_reference_index",
                    "dynamic_position_error_m",
                    "dynamic_yaw_error_rad",
                    "dynamic_auxiliary_error_rad",
                ):
                    if np.asarray(archive[name]).shape != (action_count,):
                        raise ValueError(
                            f"dynamic execution diagnostic {name} must have shape "
                            f"({action_count},)"
                        )
            for name in (
                "policy_active_drive_direction",
                "policy_desired_drive_direction",
                "policy_pending_drive_direction",
            ):
                if not np.all(np.isin(np.asarray(archive[name]), (-1, 0, 1))):
                    raise ValueError(f"{name} must use only -1, 0, or 1")
            if not np.all(np.isin(np.asarray(archive["policy_braking_state_code"]), range(6))):
                raise ValueError("policy_braking_state_code must be between 0 and 5")

    def _validate_media(self) -> None:
        for name in ("third_person.mp4", "stereo_view.mp4"):
            if (self.partial_dir / name).stat().st_size <= 0:
                raise ValueError(f"video artifact is empty: {name}")
        for name in ("task_card.png", "trajectory_overlay.png"):
            try:
                with Image.open(self.partial_dir / name) as image:
                    image.verify()
            except Exception as error:
                raise ValueError(f"invalid PNG artifact: {name}") from error

    def validate(self) -> dict[str, str]:
        if not self.partial_dir.is_dir():
            raise FileNotFoundError(f"partial audit directory does not exist: {self.partial_dir}")
        self._validate_required_files()
        self._validate_json()
        self._validate_npz()
        self._validate_media()
        return {
            name: artifact_sha256(self.partial_dir / name)
            for name in sorted(REQUIRED_AUDIT_FILES)
        }

    def publish(self) -> Path:
        self.validate()
        if self.output_dir.exists():
            raise FileExistsError(f"refusing to replace audit bundle: {self.output_dir}")
        self.partial_dir.rename(self.output_dir)
        return self.output_dir


def review_episode_bundle(
    episode_dir: str | Path,
    *,
    status: str,
    reviewer: str,
    labels: Sequence[str],
    notes: str,
) -> Path:
    """Atomically replace only human_review.json and verify all other bytes."""

    directory = Path(episode_dir)
    if status not in ALLOWED_BEHAVIOR_STATUSES:
        raise ValueError(f"unsupported behavior-review status: {status!r}")
    if not isinstance(reviewer, str) or not reviewer.strip():
        raise ValueError("reviewer must be a non-empty string")
    if not isinstance(notes, str):
        raise ValueError("notes must be a string")
    normalized_labels = tuple(labels)
    if len(set(normalized_labels)) != len(normalized_labels):
        raise ValueError("behavior-review labels cannot contain duplicates")
    unknown = sorted(set(normalized_labels).difference(ALLOWED_BEHAVIOR_LABELS))
    if unknown:
        raise ValueError(f"unknown behavior-review label: {', '.join(unknown)}")
    missing = sorted(name for name in REQUIRED_AUDIT_FILES if not (directory / name).is_file())
    if missing:
        raise ValueError(f"missing audit files: {', '.join(missing)}")

    protected_names = REQUIRED_AUDIT_FILES - {"human_review.json"}
    before = {
        name: artifact_sha256(directory / name)
        for name in sorted(protected_names)
    }
    review_path = directory / "human_review.json"
    temporary = review_path.with_suffix(review_path.suffix + ".partial")
    payload = {
        "status": status,
        "reviewer": reviewer,
        "labels": list(normalized_labels),
        "notes": notes,
    }
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(review_path)
    after = {
        name: artifact_sha256(directory / name)
        for name in sorted(protected_names)
    }
    if before != after:
        raise RuntimeError("behavior review modified protected audit artifacts")
    return review_path
