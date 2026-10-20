"""Pure-Python records, timing, and durable episode writing for collection."""

from dataclasses import dataclass, field
import ctypes
import errno
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping

import numpy as np
from PIL import Image
import yaml

from bev_ground_truth import BEVConfig


PHYSICS_HZ = 50
CONTROL_HZ = 50
STATE_HZ = 50
STEREO_HZ = 10
PHYSICS_DT_S = 1.0 / PHYSICS_HZ
STEREO_DT_S = 1.0 / STEREO_HZ
STEREO_STEP_INTERVAL = PHYSICS_HZ // STEREO_HZ
TIME_TOLERANCE_S = 1e-6
STEREO_SYNC_TOLERANCE_S = 0.002


@dataclass(frozen=True)
class RateSchedule:
    """Integer-rate schedule over physics steps."""

    physics_hz: int = PHYSICS_HZ
    control_hz: int = CONTROL_HZ
    stereo_hz: int = STEREO_HZ
    state_hz: int = STATE_HZ

    def __post_init__(self) -> None:
        if (self.physics_hz, self.control_hz, self.state_hz, self.stereo_hz) != (
            PHYSICS_HZ,
            CONTROL_HZ,
            STATE_HZ,
            STEREO_HZ,
        ):
            raise ValueError("rates must be exactly 50/50/50/10 Hz (physics/control/state/stereo)")

    @staticmethod
    def _validate_step(step: int) -> int:
        if isinstance(step, bool) or not isinstance(step, (int, np.integer)) or step < 0:
            raise ValueError("step must be a non-negative integer")
        return int(step)

    def is_control_step(self, step: int) -> bool:
        step = self._validate_step(step)
        return step % (self.physics_hz // self.control_hz) == 0

    def is_stereo_step(self, step: int) -> bool:
        step = self._validate_step(step)
        return step % (self.physics_hz // self.stereo_hz) == 0


@dataclass(frozen=True)
class CollectionContract:
    """Runtime objects and observable schema fields parsed from the YAML contract."""

    schema_version: str
    schedule: RateSchedule
    bev: BEVConfig
    action_protocol_id: str
    controls_per_block: int
    action_names: tuple[str, ...]
    action_minimum: tuple[float, ...]
    action_maximum: tuple[float, ...]
    image_width_px: int
    image_height_px: int
    visual_history_frames: int
    intrinsics_matrix_convention: str
    extrinsic_matrix_convention: str
    elevation_enabled_default: bool
    elevation_dtype: str
    elevation_invalid_value: float
    elevation_teacher_modes: tuple[str, ...]


@dataclass(frozen=True)
class StereoExtrinsics:
    """Column-vector transforms whose field names state source and target frames."""

    left_optical_to_base_extrinsic: np.ndarray
    right_optical_to_base_extrinsic: np.ndarray
    base_to_left_extrinsic: np.ndarray
    base_to_right_extrinsic: np.ndarray
    left_to_right_extrinsic: np.ndarray


def _extrinsic_matrix(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError(f"{name} must be invertible")
    return matrix.copy()


def derive_stereo_extrinsics(
    left_optical_to_base: Any,
    right_optical_to_base: Any,
) -> StereoExtrinsics:
    """Derive required source-to-target stereo calibration matrices."""

    left_to_base = _extrinsic_matrix(left_optical_to_base, "left_optical_to_base")
    right_to_base = _extrinsic_matrix(right_optical_to_base, "right_optical_to_base")
    base_to_left = np.linalg.inv(left_to_base)
    base_to_right = np.linalg.inv(right_to_base)
    return StereoExtrinsics(
        left_optical_to_base_extrinsic=left_to_base,
        right_optical_to_base_extrinsic=right_to_base,
        base_to_left_extrinsic=base_to_left,
        base_to_right_extrinsic=base_to_right,
        left_to_right_extrinsic=base_to_right @ left_to_base,
    )


@dataclass(frozen=True)
class CollectionStep:
    """One pre-step control decision and its post-step sampling boundary."""

    physics_step: int
    sim_time: float
    is_control: bool
    is_stereo: bool


def collection_step(loop_index: int, schedule: RateSchedule) -> CollectionStep:
    """Translate a zero-based loop iteration to a completed physics step."""

    index = schedule._validate_step(loop_index)
    completed_step = index + 1
    return CollectionStep(
        physics_step=completed_step,
        sim_time=completed_step / schedule.physics_hz,
        is_control=schedule.is_control_step(index),
        is_stereo=schedule.is_stereo_step(completed_step),
    )


def load_contract(path: str | Path) -> CollectionContract:
    """Parse the formal YAML contract into validated runtime configuration."""

    contract_path = Path(path)
    with contract_path.open(encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, Mapping):
        raise ValueError("contract must contain a YAML mapping")

    rates = raw.get("rates_hz")
    action = raw.get("action")
    bev = raw.get("bev")
    stereo = raw.get("stereo")
    sample = raw.get("sample")
    if not all(isinstance(section, Mapping) for section in (rates, action, bev, stereo, sample)):
        raise ValueError("contract must define rates_hz, action, stereo, bev, and sample mappings")

    schedule = RateSchedule(
        physics_hz=rates.get("physics"),
        control_hz=rates.get("control"),
        stereo_hz=rates.get("stereo"),
        state_hz=rates.get("state"),
    )
    action_protocol_id = action.get("protocol_id")
    if not isinstance(action_protocol_id, str) or not action_protocol_id.strip():
        raise ValueError("action protocol_id must be a non-empty string")
    controls_per_block = action.get("controls_per_block")
    if (
        isinstance(controls_per_block, bool)
        or not isinstance(controls_per_block, int)
        or controls_per_block <= 0
    ):
        raise ValueError("action controls_per_block must be a positive integer")
    fields = action.get("fields")
    if not isinstance(fields, list) or len(fields) != action.get("dimension"):
        raise ValueError("action fields must match the declared action dimension")
    try:
        ordered_fields = sorted(fields, key=lambda field: field["index"])
        action_names = tuple(str(field["name"]) for field in ordered_fields)
    except (KeyError, TypeError) as exc:
        raise ValueError("every action field must define an index and name") from exc
    if [field["index"] for field in ordered_fields] != list(range(len(ordered_fields))):
        raise ValueError("action field indices must be consecutive from zero")
    try:
        action_minimum = tuple(float(field["minimum"]) for field in ordered_fields)
        action_maximum = tuple(float(field["maximum"]) for field in ordered_fields)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("action field bounds must be finite ordered numbers") from exc
    if any(
        not np.isfinite(minimum)
        or not np.isfinite(maximum)
        or minimum >= maximum
        for minimum, maximum in zip(action_minimum, action_maximum)
    ):
        raise ValueError("action field bounds must be finite with minimum below maximum")

    extent = bev.get("extent_m")
    if (
        not isinstance(extent, list)
        or len(extent) != 2
        or float(extent[0]) != float(extent[1])
    ):
        raise ValueError("bev extent_m must contain equal X/Y extents")
    class_codes = bev.get("class_codes")
    if not isinstance(class_codes, Mapping):
        raise ValueError("bev class_codes must be a mapping")
    bev_config = BEVConfig(
        extent_m=float(extent[0]),
        resolution_m=float(bev.get("resolution_m")),
        free=class_codes.get("free"),
        occupied=class_codes.get("occupied"),
        unknown=class_codes.get("unknown"),
    )
    declared_shape = bev.get("shape")
    if list(bev_config.shape) != declared_shape:
        raise ValueError(
            f"bev shape {declared_shape} does not match extent/resolution {bev_config.shape}"
        )
    elevation = raw.get("elevation", bev.get("elevation"))
    if not isinstance(elevation, Mapping):
        raise ValueError("bev elevation must be a mapping")
    if elevation.get("shape") != list(bev_config.shape):
        raise ValueError("bev elevation shape must match BEV shape")
    elevation_enabled_default = elevation.get("enabled_by_default")
    if not isinstance(elevation_enabled_default, bool):
        raise ValueError("bev elevation enabled_by_default must be boolean")
    elevation_dtype = elevation.get("dtype")
    if elevation_dtype != "float32":
        raise ValueError("bev elevation dtype must be float32")
    elevation_invalid_value = float(elevation.get("invalid_value"))
    if not np.isfinite(elevation_invalid_value) or elevation_invalid_value != 0.0:
        raise ValueError("bev elevation invalid_value must be 0.0")
    elevation_teacher_modes_raw = elevation.get("teacher_modes")
    supported_modes = ("auto", "analytic", "raycast")
    if (
        not isinstance(elevation_teacher_modes_raw, list)
        or tuple(elevation_teacher_modes_raw) != supported_modes
    ):
        raise ValueError("bev elevation teacher_modes must be auto, analytic, raycast")
    visual_history_frames = sample.get("visual_history_frames")
    if (
        isinstance(visual_history_frames, bool)
        or not isinstance(visual_history_frames, int)
        or visual_history_frames <= 0
    ):
        raise ValueError("sample visual_history_frames must be a positive integer")
    extrinsic_matrix_convention = stereo.get("extrinsic_matrix_convention")
    if not isinstance(extrinsic_matrix_convention, str) or not extrinsic_matrix_convention:
        raise ValueError("stereo extrinsic_matrix_convention must be a non-empty string")
    intrinsics_matrix_convention = stereo.get("intrinsics_matrix_convention")
    if not isinstance(intrinsics_matrix_convention, str) or not intrinsics_matrix_convention:
        raise ValueError("stereo intrinsics_matrix_convention must be a non-empty string")
    return CollectionContract(
        schema_version=str(raw.get("schema_version")),
        schedule=schedule,
        bev=bev_config,
        action_protocol_id=action_protocol_id,
        controls_per_block=controls_per_block,
        action_names=action_names,
        action_minimum=action_minimum,
        action_maximum=action_maximum,
        image_width_px=int(stereo["width_px"]),
        image_height_px=int(stereo["height_px"]),
        visual_history_frames=visual_history_frames,
        intrinsics_matrix_convention=intrinsics_matrix_convention,
        extrinsic_matrix_convention=extrinsic_matrix_convention,
        elevation_enabled_default=elevation_enabled_default,
        elevation_dtype=elevation_dtype,
        elevation_invalid_value=elevation_invalid_value,
        elevation_teacher_modes=supported_modes,
    )


def validate_action(action: Any) -> np.ndarray:
    """Normalize an action to an independent float32 two-element array."""

    result = np.asarray(action, dtype=np.float32)
    if result.shape != (2,):
        raise ValueError(f"action must have exactly 2 elements, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("action must contain only finite values")
    return result.copy()


def _vector(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result.copy()


def _finite_scalar(value: Any, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _minimum_obstacle_distance(value: Any) -> float:
    result = float(value)
    if np.isnan(result) or np.isneginf(result):
        raise ValueError("minimum_obstacle_distance must be finite or positive infinity")
    return result


@dataclass
class StateRecord:
    physics_step: int
    sim_time: float
    state_timestamp: float
    pos: Any
    quat: Any
    lin_vel: Any
    ang_vel: Any
    joint_pos: Any
    joint_vel: Any
    commanded_action: Any
    applied_action: Any
    action_command_timestamp: float
    action_applied_timestamp: float

    def __post_init__(self) -> None:
        self.physics_step = int(self.physics_step)
        self.sim_time = _finite_scalar(self.sim_time, "sim_time")
        self.state_timestamp = _finite_scalar(self.state_timestamp, "state_timestamp")
        self.action_command_timestamp = _finite_scalar(
            self.action_command_timestamp, "action_command_timestamp"
        )
        self.action_applied_timestamp = _finite_scalar(
            self.action_applied_timestamp, "action_applied_timestamp"
        )
        self.pos = _vector(self.pos, (3,), "pos")
        self.quat = _vector(self.quat, (4,), "quat")
        self.lin_vel = _vector(self.lin_vel, (3,), "lin_vel")
        self.ang_vel = _vector(self.ang_vel, (3,), "ang_vel")
        self.joint_pos = _vector(self.joint_pos, (3,), "joint_pos")
        self.joint_vel = _vector(self.joint_vel, (3,), "joint_vel")
        self.commanded_action = validate_action(self.commanded_action)
        self.applied_action = validate_action(self.applied_action)


@dataclass
class StereoRecord:
    frame_id: int
    physics_step: int
    left_timestamp: float
    right_timestamp: float
    cam_left_pose: Any
    cam_right_pose: Any
    left_filename: str = ""
    right_filename: str = ""
    collision: bool = False
    minimum_obstacle_distance: float = np.inf

    def __post_init__(self) -> None:
        self.frame_id = int(self.frame_id)
        self.physics_step = int(self.physics_step)
        self.left_timestamp = _finite_scalar(self.left_timestamp, "left_timestamp")
        self.right_timestamp = _finite_scalar(self.right_timestamp, "right_timestamp")
        self.cam_left_pose = _vector(self.cam_left_pose, (4, 4), "cam_left_pose")
        self.cam_right_pose = _vector(self.cam_right_pose, (4, 4), "cam_right_pose")
        self.left_filename = str(self.left_filename)
        self.right_filename = str(self.right_filename)
        self.collision = bool(self.collision)
        self.minimum_obstacle_distance = _minimum_obstacle_distance(
            self.minimum_obstacle_distance
        )
        if self.collision != (self.minimum_obstacle_distance <= 0.0):
            raise ValueError("collision must equal (minimum_obstacle_distance <= 0)")


@dataclass
class EpisodeBuffer:
    """In-memory state/stereo rows with contract-preserving array conversion."""

    _states: List[StateRecord] = field(default_factory=list, init=False, repr=False)
    _stereo: List[StereoRecord] = field(default_factory=list, init=False, repr=False)

    def append_state(self, record: StateRecord) -> None:
        if not isinstance(record, StateRecord):
            raise TypeError("record must be a StateRecord")
        if record.physics_step < 0:
            raise ValueError("physics_step must be non-negative")
        if self._states:
            previous = self._states[-1]
            if record.physics_step <= previous.physics_step:
                raise ValueError("physics_step must be strictly increasing")
            if record.physics_step != previous.physics_step + 1:
                raise ValueError("state progression must advance exactly one physics step")
            if record.sim_time <= previous.sim_time:
                raise ValueError("sim_time must be strictly increasing")
            if abs((record.sim_time - previous.sim_time) - PHYSICS_DT_S) > TIME_TOLERANCE_S:
                raise ValueError("state progression requires a 0.02 s sim_time step")
            if record.state_timestamp <= previous.state_timestamp:
                raise ValueError("state_timestamp must be strictly increasing")
            if (
                abs((record.state_timestamp - previous.state_timestamp) - PHYSICS_DT_S)
                > TIME_TOLERANCE_S
            ):
                raise ValueError("state progression requires a 0.02 s state_timestamp step")
        self._states.append(record)

    def append_stereo(self, record: StereoRecord) -> None:
        if not isinstance(record, StereoRecord):
            raise TypeError("record must be a StereoRecord")
        if record.frame_id < 0:
            raise ValueError("stereo_frame_id must be non-negative")
        if record.physics_step < 0 or record.physics_step % STEREO_STEP_INTERVAL:
            raise ValueError("stereo physics_step must be a non-negative 10 Hz boundary")
        if abs(record.left_timestamp - record.right_timestamp) > STEREO_SYNC_TOLERANCE_S:
            raise ValueError("stereo timestamps must be synchronized")
        if self._stereo:
            previous = self._stereo[-1]
            if record.frame_id != previous.frame_id + 1:
                raise ValueError("stereo_frame_id must be consecutive")
            if record.physics_step <= previous.physics_step:
                raise ValueError("stereo_physics_step must be strictly increasing")
            if record.physics_step - previous.physics_step != STEREO_STEP_INTERVAL:
                raise ValueError("stereo physics_step must advance exactly 5 physics steps")
            if record.left_timestamp <= previous.left_timestamp:
                raise ValueError("stereo timestamps must be strictly increasing")
            if record.right_timestamp <= previous.right_timestamp:
                raise ValueError("stereo timestamps must be strictly increasing")
            if abs((record.left_timestamp - previous.left_timestamp) - STEREO_DT_S) > TIME_TOLERANCE_S:
                raise ValueError("stereo left_timestamp requires a 0.1 s step")
            if abs((record.right_timestamp - previous.right_timestamp) - STEREO_DT_S) > TIME_TOLERANCE_S:
                raise ValueError("stereo right_timestamp requires a 0.1 s step")
        self._stereo.append(record)

    def as_arrays(self) -> Dict[str, np.ndarray]:
        states = self._states
        stereo = self._stereo

        def stacked(rows: list, name: str, shape: tuple[int, ...], dtype: Any) -> np.ndarray:
            if rows:
                return np.stack([getattr(row, name) for row in rows], axis=0)
            return np.empty((0, *shape), dtype=dtype)

        return {
            "physics_step": np.asarray([row.physics_step for row in states], dtype=np.int64),
            "sim_time": np.asarray([row.sim_time for row in states], dtype=np.float64),
            "state_timestamp": np.asarray([row.state_timestamp for row in states], dtype=np.float64),
            "pos": stacked(states, "pos", (3,), np.float32),
            "quat": stacked(states, "quat", (4,), np.float32),
            "lin_vel": stacked(states, "lin_vel", (3,), np.float32),
            "ang_vel": stacked(states, "ang_vel", (3,), np.float32),
            "joint_pos": stacked(states, "joint_pos", (3,), np.float32),
            "joint_vel": stacked(states, "joint_vel", (3,), np.float32),
            "commanded_action": stacked(states, "commanded_action", (2,), np.float32),
            "applied_action": stacked(states, "applied_action", (2,), np.float32),
            "action_command_timestamp": np.asarray(
                [row.action_command_timestamp for row in states], dtype=np.float64
            ),
            "action_applied_timestamp": np.asarray(
                [row.action_applied_timestamp for row in states], dtype=np.float64
            ),
            "stereo_frame_id": np.asarray([row.frame_id for row in stereo], dtype=np.int64),
            "stereo_physics_step": np.asarray([row.physics_step for row in stereo], dtype=np.int64),
            "left_timestamp": np.asarray([row.left_timestamp for row in stereo], dtype=np.float64),
            "right_timestamp": np.asarray([row.right_timestamp for row in stereo], dtype=np.float64),
            "cam_left_pose": stacked(stereo, "cam_left_pose", (4, 4), np.float32),
            "cam_right_pose": stacked(stereo, "cam_right_pose", (4, 4), np.float32),
            "left_filename": np.asarray([row.left_filename for row in stereo], dtype=str),
            "right_filename": np.asarray([row.right_filename for row in stereo], dtype=str),
            "collision": np.asarray([row.collision for row in stereo], dtype=bool),
            "minimum_obstacle_distance": np.asarray(
                [row.minimum_obstacle_distance for row in stereo], dtype=np.float32
            ),
        }


def _uint8_image(value: Any, name: str, shape: tuple[int, ...] | None = None) -> np.ndarray:
    """Validate a contract image without silently clipping or converting it."""

    image = np.asarray(value)
    if image.dtype != np.uint8:
        raise ValueError(f"{name} must have dtype uint8, got {image.dtype}")
    if shape is not None and image.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {image.shape}")
    return image.copy()


def _depth_image(value: Any, name: str) -> np.ndarray:
    depth = np.asarray(value)
    if depth.ndim != 2 or depth.dtype not in (np.dtype(np.float32), np.dtype(np.float64)):
        raise ValueError(f"{name} must be a float32 or float64 two-dimensional array")
    if not np.all(np.isfinite(depth)):
        raise ValueError(f"{name} must contain only finite values")
    return np.asarray(depth, dtype=np.float32).copy()


def _elevation_image(value: Any, name: str) -> np.ndarray:
    image = np.asarray(value)
    if image.shape != (80, 80):
        raise ValueError(f"{name} must have shape (80, 80), got {image.shape}")
    if image.dtype != np.dtype(np.float32):
        raise ValueError(f"{name} must have dtype float32, got {image.dtype}")
    if not np.all(np.isfinite(image)):
        raise ValueError(f"{name} must contain only finite values")
    return image.copy()


def _elevation_valid(value: Any) -> np.ndarray:
    image = np.asarray(value)
    if image.shape != (80, 80):
        raise ValueError(f"height_valid_bev must have shape (80, 80), got {image.shape}")
    if image.dtype != np.dtype(np.uint8):
        raise ValueError(f"height_valid_bev must have dtype uint8, got {image.dtype}")
    if not set(np.unique(image).tolist()).issubset({0, 1}):
        raise ValueError("height_valid_bev must be binary")
    return image.copy()


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"cannot serialize {type(value).__name__} in episode metadata")


def _publish_noreplace(partial_dir: Path, output_dir: Path) -> None:
    """Atomically publish a staged directory on Linux without replacing a target.

    ``renameat2(..., RENAME_NOREPLACE)`` is the only publication primitive used
    because ``Path.replace`` can clobber a directory created after a separate
    ``exists()`` check.  On platforms without this Linux primitive we fail
    closed and preserve the partial directory rather than risking an overwrite.
    """

    if not sys.platform.startswith("linux"):
        raise NotImplementedError("atomic no-clobber directory publication requires Linux renameat2")
    libc = ctypes.CDLL(None, use_errno=True)
    try:
        renameat2 = libc.renameat2
    except AttributeError as exc:
        raise NotImplementedError("Linux libc does not expose renameat2; partial episode retained") from exc
    renameat2.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    renameat2.restype = ctypes.c_int
    result = renameat2(
        -100, os.fsencode(partial_dir), -100, os.fsencode(output_dir), 1  # AT_FDCWD, RENAME_NOREPLACE
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number == errno.EEXIST:
        raise FileExistsError(errno.EEXIST, "refusing to overwrite final episode", output_dir)
    if error_number in (errno.ENOSYS, errno.EINVAL):
        raise NotImplementedError("Linux kernel does not support RENAME_NOREPLACE; partial episode retained")
    raise OSError(error_number, os.strerror(error_number), output_dir)


class EpisodeWriter:
    """Stage an episode beside its final path, then publish it atomically.

    ``output_dir`` names the final episode directory (normally ``ep0000``).
    No final directory is ever replaced: all outputs are first written to the
    sibling ``<output_dir>.partial`` directory and the directory rename occurs
    only after states and audit-ready JSON metadata have been written.
    """

    _BEV_SHAPE = (80, 80)
    _BEV_VALUES = {0, 1, 2}
    _MASK_VALUES = {0, 1}

    def __init__(
        self,
        output_dir: str | Path,
        metadata: Mapping[str, Any],
        save_depth: bool,
        save_preview: bool,
        save_elevation: bool = False,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.partial_dir = self.output_dir.with_name(f"{self.output_dir.name}.partial")
        if self.output_dir.exists():
            raise FileExistsError(f"refusing to overwrite final episode {self.output_dir}")
        if self.partial_dir.exists():
            raise FileExistsError(f"partial episode already exists: {self.partial_dir}")
        if not isinstance(metadata, Mapping):
            raise TypeError("metadata must be a mapping")
        self.metadata = dict(metadata)
        height = self.metadata.get("image_height_px")
        width = self.metadata.get("image_width_px")
        if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
            raise ValueError("metadata image_height_px must be a positive integer")
        if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
            raise ValueError("metadata image_width_px must be a positive integer")
        self.save_depth = bool(save_depth)
        self.save_preview = bool(save_preview)
        requested_elevation = self.metadata.get("elevation_enabled")
        if requested_elevation is not None and requested_elevation != bool(save_elevation):
            raise ValueError("metadata elevation_enabled contradicts save_elevation")
        self.save_elevation = bool(save_elevation)
        self.metadata["elevation_enabled"] = self.save_elevation
        self.partial_dir.parent.mkdir(parents=True, exist_ok=True)
        self.partial_dir.mkdir()
        self._visual_frame_ids: list[int] = []
        self._image_shape = (height, width)
        self._finalized = False

    @staticmethod
    def _save_png(path: Path, image: np.ndarray) -> None:
        Image.fromarray(image).save(path, format="PNG")

    @classmethod
    def _validate_bev(cls, value: Any, name: str, allowed: set[int]) -> np.ndarray:
        image = _uint8_image(value, name, cls._BEV_SHAPE)
        values = set(np.unique(image).tolist())
        if not values.issubset(allowed):
            raise ValueError(f"{name} has values outside {sorted(allowed)}")
        return image

    @staticmethod
    def _colorize_geometry(image: np.ndarray) -> np.ndarray:
        palette = np.asarray(((235, 235, 235), (220, 65, 55), (90, 90, 105)), dtype=np.uint8)
        return palette[image]

    @staticmethod
    def _colorize_mask(image: np.ndarray, color: tuple[int, int, int]) -> np.ndarray:
        result = np.zeros((*image.shape, 3), dtype=np.uint8)
        result[image == 1] = color
        return result

    @staticmethod
    def _resize_preview_panel(image: np.ndarray, height: int, width: int) -> np.ndarray:
        return np.asarray(Image.fromarray(image).resize((width, height), Image.Resampling.NEAREST))

    def _save_preview(
        self,
        frame_id: int,
        left_rgb: np.ndarray,
        right_rgb: np.ndarray,
        geometry_bev: np.ndarray,
        current_visible: np.ndarray,
        history_observed: np.ndarray,
    ) -> None:
        height, width = left_rgb.shape[:2]
        panels = (
            left_rgb,
            right_rgb,
            self._colorize_geometry(geometry_bev),
            self._colorize_mask(current_visible, (40, 190, 85)),
            self._colorize_mask(history_observed, (60, 130, 235)),
        )
        mosaic = np.concatenate(
            [self._resize_preview_panel(panel, height, width) for panel in panels], axis=1
        )
        self._save_png(self.partial_dir / f"preview_{frame_id:06d}.png", mosaic)

    def write_visual_sample(
        self,
        frame_id: int,
        left_rgb: Any,
        right_rgb: Any,
        left_depth: Any | None,
        right_depth: Any | None,
        geometry_bev: Any,
        current_visible: Any,
        history_observed: Any,
        height_min_bev: Any | None = None,
        height_max_bev: Any | None = None,
        height_valid_bev: Any | None = None,
    ) -> None:
        """Write one synchronized visual sample into the unpublished directory."""

        if self._finalized:
            raise RuntimeError("cannot write visual samples after finalize")
        if isinstance(frame_id, bool) or not isinstance(frame_id, (int, np.integer)) or frame_id < 0:
            raise ValueError("frame_id must be a non-negative integer")
        frame_id = int(frame_id)
        if self._visual_frame_ids and frame_id != self._visual_frame_ids[-1] + 1:
            raise ValueError("visual frame IDs must be consecutive")
        if frame_id in self._visual_frame_ids:
            raise ValueError(f"visual frame {frame_id} was already written")

        left = _uint8_image(left_rgb, "left_rgb")
        right = _uint8_image(right_rgb, "right_rgb")
        if left.ndim != 3 or left.shape[2] != 3:
            raise ValueError(f"left_rgb must have shape (H, W, 3), got {left.shape}")
        if right.shape != left.shape:
            raise ValueError(f"right_rgb must have shape {left.shape}, got {right.shape}")
        image_shape = left.shape[:2]
        if image_shape != self._image_shape:
            raise ValueError(
                f"RGB images must match declared image dimensions {self._image_shape}, "
                f"got {image_shape}"
            )
        geometry = self._validate_bev(geometry_bev, "geometry_bev", self._BEV_VALUES)
        visible = self._validate_bev(current_visible, "current_visible", self._MASK_VALUES)
        history = self._validate_bev(history_observed, "history_observed", self._MASK_VALUES)

        elevation_values = (height_min_bev, height_max_bev, height_valid_bev)
        if self.save_elevation:
            if any(value is None for value in elevation_values):
                raise ValueError("height_min_bev, height_max_bev, and height_valid_bev are required when save_elevation is true")
            height_min = _elevation_image(height_min_bev, "height_min_bev")
            height_max = _elevation_image(height_max_bev, "height_max_bev")
            height_valid = _elevation_valid(height_valid_bev)
            valid_cells = height_valid == 1
            if np.any(height_min[valid_cells] > height_max[valid_cells]):
                raise ValueError("height_min_bev minimum must not exceed height_max_bev maximum")
            invalid_cells = ~valid_cells
            if np.any(height_min[invalid_cells] != 0.0) or np.any(height_max[invalid_cells] != 0.0):
                raise ValueError("height elevation values at invalid cells must equal invalid value 0.0")
            np.save(self.partial_dir / f"height_min_bev_{frame_id:06d}.npy", height_min)
            np.save(self.partial_dir / f"height_max_bev_{frame_id:06d}.npy", height_max)
            self._save_png(self.partial_dir / f"height_valid_bev_{frame_id:06d}.png", height_valid)
        elif any(value is not None for value in elevation_values):
            raise ValueError("elevation assets supplied when save_elevation is false")

        if self.save_depth:
            if left_depth is None or right_depth is None:
                raise ValueError("left_depth and right_depth are required when save_depth is true")
            left_depth_array = _depth_image(left_depth, "left_depth")
            right_depth_array = _depth_image(right_depth, "right_depth")
            if left_depth_array.shape != left.shape[:2] or right_depth_array.shape != left.shape[:2]:
                raise ValueError("depth images must match the RGB image height and width")
            np.save(self.partial_dir / f"left_depth_{frame_id:06d}.npy", left_depth_array)
            np.save(self.partial_dir / f"right_depth_{frame_id:06d}.npy", right_depth_array)

        self._save_png(self.partial_dir / f"left_{frame_id:06d}.png", left)
        self._save_png(self.partial_dir / f"right_{frame_id:06d}.png", right)
        self._save_png(self.partial_dir / f"geometry_bev_{frame_id:06d}.png", geometry)
        self._save_png(self.partial_dir / f"current_visible_{frame_id:06d}.png", visible)
        self._save_png(self.partial_dir / f"history_observed_{frame_id:06d}.png", history)
        if self.save_preview:
            self._save_preview(frame_id, left, right, geometry, visible, history)
        self._visual_frame_ids.append(frame_id)

    def _write_metadata(self) -> None:
        temporary = self.partial_dir / "metadata.json.tmp"
        metadata = dict(self.metadata)
        metadata["save_depth"] = self.save_depth
        metadata["elevation_enabled"] = self.save_elevation
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True, default=_json_default)
            handle.write("\n")
        temporary.replace(self.partial_dir / "metadata.json")

    def finalize(self, buffer: EpisodeBuffer) -> Path:
        """Write state arrays and metadata in staging, then publish the directory."""

        if self._finalized:
            raise RuntimeError("episode has already been finalized")
        if not isinstance(buffer, EpisodeBuffer):
            raise TypeError("buffer must be an EpisodeBuffer")
        arrays = buffer.as_arrays()
        stereo_ids = arrays["stereo_frame_id"].astype(np.int64, copy=False).tolist()
        if stereo_ids != self._visual_frame_ids:
            raise ValueError("visual frame IDs must exactly match stereo_frame_id records")
        arrays["left_filename"] = np.asarray(
            [f"left_{frame_id:06d}.png" for frame_id in stereo_ids], dtype=str
        )
        arrays["right_filename"] = np.asarray(
            [f"right_{frame_id:06d}.png" for frame_id in stereo_ids], dtype=str
        )
        np.savez_compressed(self.partial_dir / "states.npz", **arrays)
        self._write_metadata()

        _publish_noreplace(self.partial_dir, self.output_dir)
        self._finalized = True
        return self.output_dir
