"""Lazy Isaac Sim adapter for strict Ball PointGoal Benchmark episodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

from training.action_protocol import ActionProtocol

from .grid_planning import astar_path, inflate_obstacles
from .drive_model import DriveModel
from .dynamic_lattice_planning import (
    DynamicPlanRequest,
    DynamicPlanResult,
    DynamicState,
    plan_dynamic_lattice,
)
from .kinodynamic_planning import (
    KinodynamicPlanRequest,
    KinodynamicPlanResult,
    plan_kinodynamic_path,
)
from .lighting import (
    FIXED_CAMERA_FILL_V1,
    LEGACY_SUN_V1,
    LIGHTING_PROFILES,
    install_fixed_camera_fill,
)
from .manifest import BenchmarkEpisode
from .policies import PolicyObservation
from .runner import RuntimeStep


DEFAULT_DRIVE_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)


@dataclass(frozen=True)
class IsaacRuntimeConfig:
    scene_root: Path
    contract_path: Path
    gui: bool
    record_video: bool
    collision_mode: str = "physical_contact_v2"
    ball_radius_m: float = 0.4
    safety_margin_m: float = 0.1
    occupancy_path: Path | None = None
    drive_profile_path: Path = DEFAULT_DRIVE_PROFILE
    diagnostic_open_plane: bool = False
    diagnostic_physics_trace: bool = False
    diagnostic_auxiliary_kp: float | None = None
    diagnostic_auxiliary_kd: float | None = None
    diagnostic_auxiliary_target_rate_rad_s: float | None = None
    diagnostic_auxiliary_velocity_limit_rad_s: float | None = None
    auxiliary_target_rate_rad_s: float | None = None
    auxiliary_velocity_limit_rad_s: float | None = None
    lighting_profile: str = LEGACY_SUN_V1

    def __post_init__(self) -> None:
        if self.lighting_profile not in LIGHTING_PROFILES:
            raise ValueError(f"unknown lighting_profile: {self.lighting_profile!r}")
        if self.collision_mode not in {"physical_contact_v2", "map_clearance_v1"}:
            raise ValueError("unknown collision_mode")
        scene_root = Path(self.scene_root)
        contract_path = Path(self.contract_path)
        occupancy_path = None if self.occupancy_path is None else Path(self.occupancy_path)
        drive_profile_path = Path(self.drive_profile_path)
        radius = float(self.ball_radius_m)
        safety_margin = float(self.safety_margin_m)
        auxiliary_kp = _optional_positive_gain(
            self.diagnostic_auxiliary_kp, "diagnostic_auxiliary_kp"
        )
        auxiliary_kd = _optional_positive_gain(
            self.diagnostic_auxiliary_kd, "diagnostic_auxiliary_kd"
        )
        auxiliary_target_rate = _optional_positive_gain(
            self.diagnostic_auxiliary_target_rate_rad_s,
            "diagnostic_auxiliary_target_rate_rad_s",
        )
        auxiliary_velocity_limit = _optional_positive_gain(
            self.diagnostic_auxiliary_velocity_limit_rad_s,
            "diagnostic_auxiliary_velocity_limit_rad_s",
        )
        formal_auxiliary_target_rate = _optional_positive_gain(
            self.auxiliary_target_rate_rad_s,
            "auxiliary_target_rate_rad_s",
        )
        formal_auxiliary_velocity_limit = _optional_positive_gain(
            self.auxiliary_velocity_limit_rad_s,
            "auxiliary_velocity_limit_rad_s",
        )
        if not isinstance(self.gui, bool) or not isinstance(self.record_video, bool):
            raise ValueError("gui and record_video must be booleans")
        if not isinstance(self.diagnostic_open_plane, bool):
            raise ValueError("diagnostic_open_plane must be a boolean")
        if not isinstance(self.diagnostic_physics_trace, bool):
            raise ValueError("diagnostic_physics_trace must be a boolean")
        if (
            not self.diagnostic_open_plane
            and (
                auxiliary_kp is not None
                or auxiliary_kd is not None
                or auxiliary_target_rate is not None
                or auxiliary_velocity_limit is not None
            )
        ):
            raise ValueError(
                "diagnostic auxiliary overrides require diagnostic_open_plane"
            )
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("ball_radius_m must be finite and positive")
        if not math.isfinite(safety_margin) or safety_margin < 0.0:
            raise ValueError("safety_margin_m must be finite and non-negative")
        object.__setattr__(self, "scene_root", scene_root)
        object.__setattr__(self, "contract_path", contract_path)
        object.__setattr__(self, "occupancy_path", occupancy_path)
        object.__setattr__(self, "drive_profile_path", drive_profile_path)
        object.__setattr__(self, "diagnostic_open_plane", self.diagnostic_open_plane)
        object.__setattr__(self, "diagnostic_physics_trace", self.diagnostic_physics_trace)
        object.__setattr__(self, "diagnostic_auxiliary_kp", auxiliary_kp)
        object.__setattr__(self, "diagnostic_auxiliary_kd", auxiliary_kd)
        object.__setattr__(self, "diagnostic_auxiliary_target_rate_rad_s", auxiliary_target_rate)
        object.__setattr__(self, "diagnostic_auxiliary_velocity_limit_rad_s", auxiliary_velocity_limit)
        object.__setattr__(self, "auxiliary_target_rate_rad_s", formal_auxiliary_target_rate)
        object.__setattr__(self, "auxiliary_velocity_limit_rad_s", formal_auxiliary_velocity_limit)
        object.__setattr__(self, "ball_radius_m", radius)
        object.__setattr__(self, "safety_margin_m", safety_margin)


def _optional_positive_gain(value: object, name: str) -> float | None:
    """Validate an optional diagnostic-only positive controller gain."""

    if value is None:
        return None
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be finite and positive or None")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite and positive or None") from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive or None")
    return result


def _finite_vector(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise RuntimeError(f"{name} must be finite with shape {shape}")
    return result


def _yaw_from_wxyz(value: object) -> float:
    quaternion = _finite_vector(value, (4,), "quaternion").astype(np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise RuntimeError("quaternion must have non-zero norm")
    w, x, y, z = quaternion / norm
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _roll_from_wxyz(value: object) -> float:
    """Return world roll from the same normalized world quaternion as yaw."""

    quaternion = _finite_vector(value, (4,), "quaternion").astype(np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise RuntimeError("quaternion must have non-zero norm")
    w, x, y, z = quaternion / norm
    return float(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def _gravity_in_base(value: object) -> np.ndarray:
    quaternion = _finite_vector(value, (4,), "quaternion").astype(np.float64)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    # Third row of world-from-base rotation: R.T @ [0, 0, -1].
    return np.asarray(
        (
            -2.0 * (x * z - w * y),
            -2.0 * (y * z + w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )


def _contact_filter_groups(
    sensor_paths: list[str], collision_paths: list[str]
) -> list[list[str]]:
    """Build explicit one-to-many PhysX contact filters for every sensor body."""

    if not sensor_paths:
        raise ValueError("sensor_paths must not be empty")
    if not collision_paths:
        raise ValueError("collision_paths must not be empty")
    return [list(collision_paths) for _ in sensor_paths]


def _scene_obstacle_collision_paths(
    scene_root: str, collision_paths: list[str]
) -> list[str]:
    """Remove only the known scene ground prim and its collision children."""

    scene_root = str(scene_root).rstrip("/")
    if not scene_root:
        raise ValueError("scene_root must not be empty")
    # Collection scenes contain a default ground at the stage root.  The
    # optional external scene may also carry a scene-local ground prim; both
    # are support geometry, never diagnostic obstacles.
    ground_roots = (
        "/World/defaultGroundPlane",
        f"{scene_root}/defaultGroundPlane",
    )
    return [
        path
        for path in collision_paths
        if not any(
            path == ground_root or path.startswith(f"{ground_root}/")
            for ground_root in ground_roots
        )
    ]


def _plan_kinodynamic_reference(
    *,
    occupancy_grid: np.ndarray,
    resolution_m: float,
    origin_xy: object,
    episode: BenchmarkEpisode,
    drive_profile_path: str | Path,
) -> KinodynamicPlanResult:
    """Build the privileged reference without importing or initializing Isaac."""

    if not isinstance(episode, BenchmarkEpisode):
        raise TypeError("episode must be a BenchmarkEpisode")
    drive_model = DriveModel.from_json(drive_profile_path)
    return plan_kinodynamic_path(
        KinodynamicPlanRequest(
            occupancy_grid=occupancy_grid,
            resolution_m=resolution_m,
            origin_xy=tuple(np.asarray(origin_xy, dtype=np.float64)),
            start_xy=episode.start_position_world_m[:2],
            start_yaw_rad=_yaw_from_wxyz(episode.start_quaternion_world_wxyz),
            goal_xy=episode.goal_position_world_m[:2],
        ),
        drive_model,
    )


class IsaacBenchmarkRuntime:
    """Execute one manifest episode while keeping privileged truth out of observations.

    Isaac modules are imported only from :meth:`reset`, so importing benchmark
    tooling and reviewing existing bundles never starts Kit.
    """

    _PHYSICS_DIAGNOSTIC_FIELDS = (
        "raw_command_target",
        "clipped_target",
        "smoothed_target",
        "applied_action",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "position_world_xy",
        "root_position_world_m",
        "root_quaternion_world_wxyz",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "yaw_world_rad",
        "roll_world_rad",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
        "action_block_index",
        "physics_step_in_block",
    )

    def __init__(self, config: IsaacRuntimeConfig) -> None:
        if not isinstance(config, IsaacRuntimeConfig):
            raise TypeError("config must be an IsaacRuntimeConfig")
        self.config = config
        self._simulation_app: Any | None = None
        self._closed = False
        self._episode: BenchmarkEpisode | None = None
        self._reset_observation: PolicyObservation | None = None
        self._dynamic_plan: DynamicPlanResult | None = None
        self._history_proprio: list[np.ndarray] = []
        self._history_quaternion_world_wxyz: list[np.ndarray] = []
        self._history_actions: list[np.ndarray] = []
        self._previous_action = np.zeros(2, dtype=np.float32)
        self._oracle_path = np.empty((0, 2), dtype=np.float64)
        self._reference_visualization_path: np.ndarray | None = None
        self._kinodynamic_plan: KinodynamicPlanResult | None = None
        self._captured_frames: dict[str, list[np.ndarray]] = {
            "third_person": [],
            "stereo_view": [],
        }
        self._block_physical_contact = False
        self._collision_contact_samples: list[dict[str, object]] = []
        self._contact_available = True
        self._clock_epoch_current_time: float | None = None
        self._clock_epoch_current_time_step_index: int | None = None
        self._physics_diagnostics = self._new_physics_diagnostics()
        self._diagnostic_block_index = 0
        self._diagnostic_configuration = self._empty_diagnostic_configuration()

    @property
    def occupancy(self) -> Any:
        if not hasattr(self, "_occupancy"):
            raise RuntimeError("runtime must be reset before occupancy is available")
        return self._occupancy

    @property
    def inflated_grid(self) -> np.ndarray:
        if not hasattr(self, "_inflated_grid"):
            raise RuntimeError("runtime must be reset before inflated_grid is available")
        return self._inflated_grid.copy()

    @property
    def captured_frames(self) -> dict[str, tuple[np.ndarray, ...]]:
        return {name: tuple(frames) for name, frames in self._captured_frames.items()}

    @property
    def physics_diagnostics(self) -> dict[str, np.ndarray]:
        """Return an independent copy of the optional post-physics trace."""

        diagnostics = getattr(self, "_physics_diagnostics", None)
        if diagnostics is None:
            diagnostics = self._new_physics_diagnostics()
        return {
            name: np.asarray(values).copy()
            for name, values in diagnostics.items()
        }

    @property
    def diagnostic_configuration(self) -> dict[str, object]:
        """Return controller/DOF metadata captured during runtime reset."""

        return self._copy_diagnostic_configuration(
            getattr(self, "_diagnostic_configuration", None)
        )

    @property
    def controller_diagnostics(self) -> dict[str, object]:
        """Compatibility alias for :attr:`diagnostic_configuration`."""

        return self.diagnostic_configuration

    @property
    def contact_measurement_available(self) -> bool:
        return self._contact_available

    @classmethod
    def _new_physics_diagnostics(cls) -> dict[str, list[object]]:
        return {name: [] for name in cls._PHYSICS_DIAGNOSTIC_FIELDS}

    @staticmethod
    def _empty_diagnostic_configuration() -> dict[str, object]:
        return {
            "joint_names": None,
            "joint_names_available": False,
            "requested_gains": {"kp": None, "kd": None},
            "baseline_gains": {"kp": None, "kd": None},
            "readback_gains": {"kp": None, "kd": None},
            "gain_readback_available": False,
            "gain_readback_error": "not initialized",
            "auxiliary_target_rate_limit_rad_s": None,
            "auxiliary_velocity_limit_rad_s": None,
        }

    def _auxiliary_target_rate_rad_s(self) -> float | None:
        return (
            getattr(self.config, "auxiliary_target_rate_rad_s", None)
            if getattr(self.config, "auxiliary_target_rate_rad_s", None) is not None
            else getattr(self.config, "diagnostic_auxiliary_target_rate_rad_s", None)
        )

    def _auxiliary_velocity_limit_rad_s(self) -> float | None:
        return (
            getattr(self.config, "auxiliary_velocity_limit_rad_s", None)
            if getattr(self.config, "auxiliary_velocity_limit_rad_s", None) is not None
            else getattr(self.config, "diagnostic_auxiliary_velocity_limit_rad_s", None)
        )

    @classmethod
    def _copy_diagnostic_configuration(
        cls, configuration: dict[str, object] | None
    ) -> dict[str, object]:
        if configuration is None:
            configuration = cls._empty_diagnostic_configuration()

        def copy_value(value: object) -> object:
            if isinstance(value, np.ndarray):
                return value.copy()
            if isinstance(value, dict):
                return {str(key): copy_value(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [copy_value(item) for item in value]
            return value

        return copy_value(configuration)  # type: ignore[return-value]

    def oracle_path_xy(self, episode: BenchmarkEpisode) -> np.ndarray:
        if self._episode is None or episode.episode_id != self._episode.episode_id:
            raise ValueError("oracle path is only available for the active episode")
        return self._oracle_path.copy()

    def set_reference_visualization_path(self, points_xy: object) -> None:
        """Set the injected diagnostic path drawn in the third-person view.

        The runtime's internally planned path remains available as privileged
        runtime truth, but diagnostic cases may provide a different geometry
        reference.  This setter must be called before ``reset`` so the video
        cannot label the internal all-free A* line as the acceptance reference.
        """

        path = np.asarray(points_xy, dtype=np.float64)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] < 2:
            raise ValueError("reference visualization path must have shape (N >= 2, 2)")
        if not np.all(np.isfinite(path)):
            raise ValueError("reference visualization path must be finite")
        self._reference_visualization_path = path.copy()

    def kinodynamic_plan(
        self, episode: BenchmarkEpisode
    ) -> KinodynamicPlanResult | None:
        if self._episode is None or episode.episode_id != self._episode.episode_id:
            raise ValueError("kinodynamic plan is only available for the active episode")
        return self._kinodynamic_plan

    def dynamic_plan(self, episode: BenchmarkEpisode) -> DynamicPlanResult:
        if self._episode is None or episode.episode_id != self._episode.episode_id:
            raise ValueError("dynamic plan is only available for the active episode")
        if self._dynamic_plan is not None:
            return self._dynamic_plan
        if self._reset_observation is None:
            raise RuntimeError("runtime must be reset before dynamic planning")

        observation = self._reset_observation
        proprio = observation.proprio_history[-1]
        main_command = float(proprio[12])
        if main_command > 0.0:
            drive_direction = "forward"
        elif main_command < 0.0:
            drive_direction = "reverse"
        else:
            drive_direction = "stopped"
        initial_state = DynamicState(
            x_m=float(observation.position_world_xy[0]),
            y_m=float(observation.position_world_xy[1]),
            yaw_rad=observation.yaw_world_rad,
            main_command_rad_s=main_command,
            auxiliary_angle_rad=float(proprio[10]),
            speed_m_s=float(np.linalg.norm(proprio[:2])),
            elapsed_s=0.0,
            drive_direction=drive_direction,
            stopped_time_s=0.0,
        )
        request = DynamicPlanRequest(
            occupancy_grid=self._inflated_grid,
            resolution_m=self._occupancy.resolution_m,
            origin_xy=tuple(
                np.asarray(self._occupancy.global_origin_xy, dtype=np.float64)
            ),
            initial_state=initial_state,
            goal_xy=episode.goal_position_world_m[:2],
        )
        self._dynamic_plan = plan_dynamic_lattice(
            request,
            DriveModel.from_json(self.config.drive_profile_path),
        )
        return self._dynamic_plan

    def _read_world_clock(self) -> tuple[float, int]:
        """Read both independent clock quantities directly from Isaac World."""

        if not hasattr(self, "_world"):
            raise RuntimeError("Isaac world is not initialized")
        try:
            raw_time = float(self._world.current_time)
            raw_step_value = self._world.current_time_step_index
            raw_step_float = float(raw_step_value)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise RuntimeError(
                "Isaac world must expose finite current_time and integer "
                "current_time_step_index"
            ) from error
        if (
            not math.isfinite(raw_time)
            or raw_time < 0.0
            or not math.isfinite(raw_step_float)
            or not raw_step_float.is_integer()
            or raw_step_float < 0.0
        ):
            raise RuntimeError(
                "Isaac world must expose finite non-negative current_time and "
                "non-negative integer current_time_step_index"
            )
        return raw_time, int(raw_step_float)

    def _establish_clock_epoch(self) -> None:
        """Anchor scored episode time immediately after all pre-episode warmup."""

        raw_time, raw_step = self._read_world_clock()
        self._clock_epoch_current_time = raw_time
        self._clock_epoch_current_time_step_index = raw_step

    def _clock_metadata(self) -> tuple[float, int, float, int]:
        """Return relative and raw world clock values for one captured state."""

        epoch_time = self._clock_epoch_current_time
        epoch_step = self._clock_epoch_current_time_step_index
        if epoch_time is None or epoch_step is None:
            raise RuntimeError("episode clock epoch has not been established")
        raw_time, raw_step = self._read_world_clock()
        relative_time = raw_time - epoch_time
        relative_step = raw_step - epoch_step
        if not math.isfinite(relative_time) or relative_time < 0.0 or relative_step < 0:
            raise RuntimeError("Isaac world clock moved backwards during the episode")
        return relative_time, relative_step, raw_time, raw_step

    def _configured_articulation_gains(self) -> tuple[np.ndarray, np.ndarray]:
        """Return baseline gains with diagnostic-only auxiliary overrides applied."""

        scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
        if str(scripts_dir) not in sys.path:
            sys.path.insert(0, str(scripts_dir))
        from action_smoothing import default_articulation_gains

        baseline_kps, baseline_kds = default_articulation_gains()
        baseline_kps = np.asarray(baseline_kps, dtype=np.float32).copy()
        baseline_kds = np.asarray(baseline_kds, dtype=np.float32).copy()
        if baseline_kps.shape != (3,) or baseline_kds.shape != (3,):
            raise RuntimeError("default articulation gains must have shape (3,)")
        if not np.all(np.isfinite(baseline_kps)) or not np.all(np.isfinite(baseline_kds)):
            raise RuntimeError("default articulation gains must be finite")

        configured_kps = baseline_kps.copy()
        configured_kds = baseline_kds.copy()
        auxiliary_kp = getattr(self.config, "diagnostic_auxiliary_kp", None)
        auxiliary_kd = getattr(self.config, "diagnostic_auxiliary_kd", None)
        if auxiliary_kp is not None:
            configured_kps[1] = float(auxiliary_kp)
        if auxiliary_kd is not None:
            configured_kds[1] = float(auxiliary_kd)
        self._diagnostic_baseline_gains = {
            "kp": baseline_kps.copy(),
            "kd": baseline_kds.copy(),
        }
        return configured_kps, configured_kds

    @staticmethod
    def _joint_names(robot: Any) -> tuple[str, ...] | None:
        """Read DOF names when the Isaac articulation exposes them."""

        for accessor in (
            "dof_names",
            "joint_names",
            "get_dof_names",
            "get_joint_names",
        ):
            try:
                value = getattr(robot, accessor)
                value = value() if callable(value) else value
            except Exception:
                continue
            if isinstance(value, str):
                continue
            try:
                names = tuple(str(name) for name in value)
            except (TypeError, ValueError):
                continue
            if names and all(name.strip() for name in names):
                return names
        return None

    @staticmethod
    def _gain_vector(value: object) -> np.ndarray | None:
        try:
            vector = np.asarray(value, dtype=np.float32)
        except (TypeError, ValueError):
            return None
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            return None
        return vector.copy()

    @classmethod
    def _gain_readback(
        cls, controller: Any, robot: Any
    ) -> tuple[np.ndarray | None, np.ndarray | None, str | None]:
        """Read controller gains only through explicit public getters."""

        errors: list[str] = []
        for owner_name, owner in (("controller", controller), ("robot", robot)):
            getter = getattr(owner, "get_gains", None)
            if not callable(getter):
                continue
            try:
                result = getter()
            except Exception as error:
                errors.append(
                    f"{owner_name}.get_gains: {type(error).__name__}: {error}"
                )
                continue
            proportional: object | None = None
            derivative: object | None = None
            if isinstance(result, dict):
                proportional = result.get(
                    "kps", result.get("kp", result.get("stiffness"))
                )
                derivative = result.get(
                    "kds", result.get("kd", result.get("damping"))
                )
            elif isinstance(result, (tuple, list)) and len(result) == 2:
                proportional, derivative = result
            if proportional is not None or derivative is not None:
                return (
                    cls._gain_vector(proportional),
                    cls._gain_vector(derivative),
                    None,
                )
            errors.append(f"{owner_name}.get_gains returned an unsupported value")

        proportional: np.ndarray | None = None
        derivative: np.ndarray | None = None
        for owner_name, owner in (("controller", controller), ("robot", robot)):
            for accessor, target in (
                ("get_stiffness", "kp"),
                ("get_damping", "kd"),
            ):
                getter = getattr(owner, accessor, None)
                if not callable(getter):
                    continue
                try:
                    vector = cls._gain_vector(getter())
                except Exception as error:
                    errors.append(
                        f"{owner_name}.{accessor}: {type(error).__name__}: {error}"
                    )
                    continue
                if target == "kp":
                    proportional = vector
                else:
                    derivative = vector
        if proportional is not None or derivative is not None:
            return proportional, derivative, None
        return (
            None,
            None,
            "; ".join(errors) if errors else "public gain readback is unavailable",
        )

    def _record_diagnostic_configuration(
        self,
        robot: Any,
        controller: Any,
        configured_kps: np.ndarray,
        configured_kds: np.ndarray,
    ) -> None:
        names = self._joint_names(robot)
        readback_kps, readback_kds, readback_error = self._gain_readback(
            controller, robot
        )
        baseline = getattr(self, "_diagnostic_baseline_gains", {})
        baseline_kps = baseline.get("kp")
        baseline_kds = baseline.get("kd")
        self._diagnostic_configuration = {
            "joint_names": None if names is None else list(names),
            "dof_names": None if names is None else list(names),
            "joint_names_available": names is not None,
            "requested_gains": {
                "kp": np.asarray(configured_kps, dtype=np.float32).copy(),
                "kd": np.asarray(configured_kds, dtype=np.float32).copy(),
            },
            "baseline_gains": {
                "kp": None if baseline_kps is None else np.asarray(baseline_kps).copy(),
                "kd": None if baseline_kds is None else np.asarray(baseline_kds).copy(),
            },
            "readback_gains": {"kp": readback_kps, "kd": readback_kds},
            "gain_readback_available": readback_kps is not None and readback_kds is not None,
            "gain_readback_error": readback_error,
            "auxiliary_target_rate_limit_rad_s": self._auxiliary_target_rate_rad_s(),
            "auxiliary_velocity_limit_rad_s": self._auxiliary_velocity_limit_rad_s(),
        }

    def _record_physics_diagnostic(
        self,
        raw_command_target: object,
        clipped_target: object,
        smoothed_target: object,
        applied_action: object,
        *,
        action_block_index: int,
        physics_step_in_block: int,
    ) -> None:
        """Capture actual robot/world values immediately after one physics step."""

        (
            simulation_time_s,
            physics_step_index,
            raw_simulation_time_s,
            raw_physics_step_index,
        ) = self._clock_metadata()
        diagnostics = getattr(self, "_physics_diagnostics", None)
        if diagnostics is None:
            diagnostics = self._new_physics_diagnostics()
            self._physics_diagnostics = diagnostics
        if diagnostics["raw_physics_step_index"]:
            previous_raw_step = int(diagnostics["raw_physics_step_index"][-1])
            previous_step = int(diagnostics["physics_step_index"][-1])
            if (
                raw_physics_step_index != previous_raw_step + 1
                or physics_step_index != previous_step + 1
            ):
                raise RuntimeError("diagnostic physics clock skipped a world step")
            previous_raw_time = float(diagnostics["raw_simulation_time_s"][-1])
            previous_time = float(diagnostics["simulation_time_s"][-1])
            physics_hz = 50.0
            contract = getattr(self, "_contract", None)
            schedule = getattr(contract, "schedule", None)
            if schedule is not None:
                try:
                    physics_hz = float(schedule.physics_hz)
                except (TypeError, ValueError, AttributeError):
                    physics_hz = 50.0
            expected_dt = 1.0 / physics_hz
            if (
                abs(raw_simulation_time_s - previous_raw_time - expected_dt) > 5e-5
                or abs(simulation_time_s - previous_time - expected_dt) > 5e-5
            ):
                raise RuntimeError("diagnostic physics clock skipped a world step")

        position, quaternion = self._robot.get_world_pose()
        position = _finite_vector(position, (3,), "position")
        quaternion = _finite_vector(quaternion, (4,), "quaternion")
        linear_velocity = _finite_vector(
            self._robot.get_linear_velocity(), (3,), "linear_velocity"
        )
        angular_velocity = _finite_vector(
            self._robot.get_angular_velocity(), (3,), "angular_velocity"
        )
        joint_positions = _finite_vector(
            self._robot.get_joint_positions(), (3,), "joint_positions"
        )
        joint_velocities = _finite_vector(
            self._robot.get_joint_velocities(), (3,), "joint_velocities"
        )
        yaw = _yaw_from_wxyz(quaternion)
        roll = _roll_from_wxyz(quaternion)
        raw_target = _finite_vector(raw_command_target, (2,), "raw_command_target")
        clipped = _finite_vector(clipped_target, (2,), "clipped_target")
        smoothed = _finite_vector(smoothed_target, (2,), "smoothed_target")
        applied = _finite_vector(applied_action, (2,), "applied_action")
        values: dict[str, object] = {
            "raw_command_target": raw_target.copy(),
            "clipped_target": clipped.copy(),
            "smoothed_target": smoothed.copy(),
            "applied_action": applied.copy(),
            "joint_position_rad": joint_positions.copy(),
            "joint_velocity_rad_s": joint_velocities.copy(),
            "position_world_xy": position[:2].copy(),
            "root_position_world_m": position.copy(),
            "root_quaternion_world_wxyz": quaternion.copy(),
            "linear_velocity_world_m_s": linear_velocity.copy(),
            "angular_velocity_world_rad_s": angular_velocity.copy(),
            "yaw_world_rad": yaw,
            "roll_world_rad": roll,
            "simulation_time_s": simulation_time_s,
            "physics_step_index": physics_step_index,
            "raw_simulation_time_s": raw_simulation_time_s,
            "raw_physics_step_index": raw_physics_step_index,
            "action_block_index": action_block_index,
            "physics_step_in_block": physics_step_in_block,
        }
        for name, value in values.items():
            if isinstance(value, np.ndarray):
                diagnostics[name].append(value.copy())
            else:
                diagnostics[name].append(value)

    def _reapply_episode_state_after_camera_prime(
        self, episode: BenchmarkEpisode
    ) -> None:
        """Apply the final reset state after variable camera warmup completes."""

        self._robot.set_world_pose(
            position=np.asarray(episode.start_position_world_m, dtype=np.float32),
            orientation=np.asarray(
                episode.start_quaternion_world_wxyz, dtype=np.float32
            ),
        )
        self._robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        self._robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        self._robot.set_joint_positions(np.zeros(3, dtype=np.float32))
        self._robot.set_joint_velocities(np.zeros(3, dtype=np.float32))
        self._action_smoother.reset()
        self._previous_action.fill(0.0)
        self._world.step(render=False)

    def reset(self, episode: BenchmarkEpisode) -> RuntimeStep:
        if self._closed:
            raise RuntimeError("Isaac runtime is closed")
        if self._simulation_app is not None:
            raise RuntimeError("one IsaacBenchmarkRuntime instance supports one episode")
        if not isinstance(episode, BenchmarkEpisode):
            raise TypeError("episode must be a BenchmarkEpisode")
        self._physics_diagnostics = self._new_physics_diagnostics()
        self._diagnostic_block_index = 0
        self._diagnostic_configuration = self._empty_diagnostic_configuration()
        scene_path = episode.verify_scene(self.config.scene_root)
        if not self.config.contract_path.is_file():
            raise FileNotFoundError(f"data contract does not exist: {self.config.contract_path}")
        if self.config.occupancy_path is None:
            raise ValueError("a frozen occupancy_path is required for strict benchmark audit")
        occupancy_path = episode.verify_occupancy(self.config.occupancy_path)
        self._episode = episode
        self._reset_observation = None
        self._dynamic_plan = None
        self._clock_epoch_current_time = None
        self._clock_epoch_current_time_step_index = None
        self._initialize(scene_path, occupancy_path, episode)
        # Camera priming advances a variable number of physics steps.  The
        # final state write and one fixed non-rendering step must happen after
        # priming and before the scored clock epoch/capture.
        if all(
            hasattr(self, name)
            for name in ("_world", "_robot", "_action_smoother")
        ):
            self._reapply_episode_state_after_camera_prime(episode)
            self._establish_clock_epoch()
            if getattr(self.config, "diagnostic_physics_trace", False):
                zero_action = np.zeros(2, dtype=np.float32)
                self._record_physics_diagnostic(
                    zero_action,
                    zero_action,
                    zero_action,
                    zero_action,
                    action_block_index=0,
                    physics_step_in_block=0,
                )
        # Keep simulator-independent test initializers usable when they do not
        # provide the objects required for the real post-prime reset.
        elif hasattr(self, "_world") and self._clock_epoch_current_time is None:
            self._establish_clock_epoch()
        initial_step = self._capture()
        self._reset_observation = initial_step.observation
        return initial_step

    def _initialize(
        self,
        scene_path: Path,
        occupancy_path: Path,
        episode: BenchmarkEpisode,
    ) -> None:
        from isaacsim import SimulationApp

        self._simulation_app = SimulationApp(
            {"headless": not self.config.gui, "width": 1280, "height": 720}
        )
        import carb
        import omni.usd
        from isaacsim.core.prims import RigidPrim
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.prims import XFormPrim
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
        from omni.isaac.sensor import Camera
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

        scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
        if str(scripts_dir) not in sys.path:
            sys.path.insert(0, str(scripts_dir))
        from action_smoothing import ActionSmoother
        from bev_ground_truth import (
            crop_global_occupancy,
            enable_camera_depth,
            obstacle_clearance,
        )
        from collect_data_dubins import (
            BASE_ROOT,
            LEFT_CAMERA_PATH,
            RIGHT_CAMERA_PATH,
            ROBOT_ROOT,
            ROBOT_USD,
            _camera_optical_to_base,
            _get_rgb,
            _load_collection_scene,
            _prime_stereo_cameras,
            _reset_articulation_after_occupancy,
            _warmup_physics_scene,
        )
        from collection_schema import load_contract
        from benchmark.task_cards import load_occupancy_npz

        self._get_rgb = _get_rgb
        self._crop_global_occupancy = crop_global_occupancy
        self._obstacle_clearance = obstacle_clearance
        self._ArticulationAction = ArticulationAction
        self._contract = load_contract(self.config.contract_path)
        self._action_protocol = ActionProtocol.from_contract(self._contract)
        self._action_smoother = ActionSmoother(
            dt_s=1.0 / self._contract.schedule.physics_hz,
            **(
                {
                    "max_auxiliary_rate_rad_s": float(
                        auxiliary_target_rate
                    )
                }
                if (auxiliary_target_rate := self._auxiliary_target_rate_rad_s()) is not None
                else {}
            ),
            max_auxiliary_velocity_rad_s=(
                self._auxiliary_velocity_limit_rad_s()
            ),
        )
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / self._contract.schedule.physics_hz,
            rendering_dt=1.0 / self._contract.schedule.stereo_hz,
        )
        scene_root = _load_collection_scene(
            self._world,
            str(scene_path),
            add_reference_to_stage,
            XFormPrim,
            print_fn=lambda _message: None,
        )
        if scene_root is None:
            raise RuntimeError("benchmark requires a non-empty external scene")
        stage = omni.usd.get_context().get_stage()
        for path in ("/World/SunLight", "/World/up_disk", "/World/dn_disk"):
            stage.RemovePrim(path)
        sunlight = UsdLux.DistantLight.Define(stage, "/World/SunLight")
        sunlight.CreateIntensityAttr(1000.0)
        self.lighting_lamp_positions_world_m = ()
        if self.config.lighting_profile == FIXED_CAMERA_FILL_V1:
            self.lighting_lamp_positions_world_m = install_fixed_camera_fill(
                stage,
                episode.start_position_world_m,
                episode.goal_position_world_m,
            )
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")

        robot_usd = Path(ROBOT_USD)
        if not robot_usd.is_file():
            raise FileNotFoundError(f"robot USD does not exist: {robot_usd}")
        add_reference_to_stage(usd_path=str(robot_usd), prim_path=ROBOT_ROOT)
        start_position = np.asarray(episode.start_position_world_m, dtype=np.float32)
        self._robot = self._world.scene.add(
            Articulation(
                prim_path=ROBOT_ROOT,
                name="ball_robot",
                position=start_position,
                orientation=np.asarray(episode.start_quaternion_world_wxyz, dtype=np.float32),
            )
        )
        robot_prim = stage.GetPrimAtPath(ROBOT_ROOT)
        rigid_paths = [
            str(prim.GetPath())
            for prim in Usd.PrimRange(robot_prim)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if not rigid_paths:
            raise RuntimeError("robot USD contains no rigid bodies for contact tracking")
        scene_prim = stage.GetPrimAtPath(scene_root)
        collision_paths = [
            str(prim.GetPath())
            for prim in Usd.PrimRange(scene_prim)
            if prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if not collision_paths and not self.config.diagnostic_open_plane:
            raise RuntimeError("benchmark scene contains no collision geometry")
        obstacle_collision_paths = _scene_obstacle_collision_paths(
            scene_root, collision_paths
        )
        if obstacle_collision_paths:
            self._contacts = self._world.scene.add(
                RigidPrim(
                    prim_paths_expr=rigid_paths,
                    name="benchmark_robot_contacts",
                    reset_xform_properties=False,
                    track_contact_forces=True,
                    contact_filter_prim_paths_expr=_contact_filter_groups(
                        rigid_paths, obstacle_collision_paths
                    ),
                )
            )
        else:
            # Do not pass an empty filter to PhysX: it means "all" filters.
            # With only the known ground present there are no obstacle contacts.
            self._contacts = None
            self._contact_available = True

        width = self._contract.image_width_px
        height = self._contract.image_height_px
        vertical_aperture = 18.0 * height / width

        def add_stereo_camera(path: str, y_offset: float) -> None:
            camera_prim = UsdGeom.Camera.Define(stage, path)
            camera_prim.GetFocalLengthAttr().Set(16.45)
            camera_prim.GetHorizontalApertureAttr().Set(18.0)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
            xform = UsdGeom.Xformable(camera_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(0.1, y_offset, 0.1))
            xform.AddRotateYZXOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))

        add_stereo_camera(LEFT_CAMERA_PATH, 0.30)
        add_stereo_camera(RIGHT_CAMERA_PATH, -0.30)
        third_path = "/World/BenchmarkThirdPersonCamera"
        third_prim = UsdGeom.Camera.Define(stage, third_path)
        third_prim.GetFocalLengthAttr().Set(20.0)
        third_prim.GetHorizontalApertureAttr().Set(36.0)
        midpoint = 0.5 * (
            np.asarray(episode.start_position_world_m)
            + np.asarray(episode.goal_position_world_m)
        )
        view_height = max(8.0, 2.0 * episode.euclidean_distance_m)
        third_xform = UsdGeom.Xformable(third_prim)
        third_xform.ClearXformOpOrder()
        third_xform.AddTranslateOp().Set(
            Gf.Vec3d(float(midpoint[0]), float(midpoint[1]), view_height)
        )

        self._world.reset()
        self._left_camera = Camera(
            prim_path=LEFT_CAMERA_PATH,
            resolution=(width, height),
            frequency=self._contract.schedule.stereo_hz,
        )
        self._right_camera = Camera(
            prim_path=RIGHT_CAMERA_PATH,
            resolution=(width, height),
            frequency=self._contract.schedule.stereo_hz,
        )
        self._third_camera = Camera(
            prim_path=third_path,
            resolution=(1280, 720),
            frequency=self._contract.schedule.stereo_hz,
        )
        for camera in (self._left_camera, self._right_camera, self._third_camera):
            camera.initialize()
        enable_camera_depth(self._left_camera)
        enable_camera_depth(self._right_camera)
        _camera_optical_to_base(stage, LEFT_CAMERA_PATH)
        _camera_optical_to_base(stage, RIGHT_CAMERA_PATH)
        _warmup_physics_scene(self._world, steps=10)
        # Use the exact frozen map whose file hash is in the manifest.  This is
        # the privileged audit truth exported from the same scene, not policy input.
        self._occupancy = load_occupancy_npz(occupancy_path)
        planning_grid = np.where(
            self._occupancy.grid == self._contract.bev.free, 0, 1
        ).astype(np.uint8)
        self._inflated_grid = inflate_obstacles(
            planning_grid,
            radius_m=self.config.ball_radius_m + self.config.safety_margin_m,
            resolution_m=self._occupancy.resolution_m,
            occupied=1,
        )
        self._prepare_reference_planning(episode)
        self._Gf = Gf
        self._actual_positions: list[np.ndarray] = []
        visualization_path = (
            self._oracle_path
            if self._reference_visualization_path is None
            else self._reference_visualization_path
        )
        self._actual_curve = self._create_markers(
            stage, episode, visualization_path, Gf, UsdGeom
        )

        configured_kps, configured_kds = self._configured_articulation_gains()
        self._controller = _reset_articulation_after_occupancy(
            self._world, self._robot, configured_kps, configured_kds
        )
        self._record_diagnostic_configuration(
            self._robot, self._controller, configured_kps, configured_kds
        )
        self._robot.set_world_pose(
            position=start_position,
            orientation=np.asarray(episode.start_quaternion_world_wxyz, dtype=np.float32),
        )
        self._robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        self._robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        self._robot.set_joint_positions(np.zeros(3, dtype=np.float32))
        self._robot.set_joint_velocities(np.zeros(3, dtype=np.float32))
        self._action_smoother.reset()
        self._previous_action.fill(0.0)
        self._world.step(render=True)
        _prime_stereo_cameras(self._world, self._left_camera, self._right_camera)

    @staticmethod
    def _create_markers(
        stage: Any,
        episode: BenchmarkEpisode,
        oracle_path: np.ndarray,
        gf: Any,
        usd_geom: Any,
    ) -> Any:
        colors = {
            "Start": gf.Vec3f(0.1, 0.8, 0.2),
            "Goal": gf.Vec3f(0.9, 0.1, 0.6),
        }
        for name, position in (
            ("Start", episode.start_position_world_m),
            ("Goal", episode.goal_position_world_m),
        ):
            sphere = usd_geom.Sphere.Define(stage, f"/World/BenchmarkMarkers/{name}")
            sphere.CreateRadiusAttr(0.15)
            sphere.CreateDisplayColorAttr([colors[name]])
            transform = usd_geom.Xformable(sphere)
            transform.AddTranslateOp().Set(
                gf.Vec3d(float(position[0]), float(position[1]), 0.15)
            )
        curve = usd_geom.BasisCurves.Define(stage, "/World/BenchmarkMarkers/OraclePath")
        curve.CreateTypeAttr("linear")
        curve.CreateCurveVertexCountsAttr([int(oracle_path.shape[0])])
        curve.CreatePointsAttr(
            [gf.Vec3f(float(x), float(y), 0.04) for x, y in oracle_path]
        )
        curve.CreateWidthsAttr([0.04])
        curve.CreateDisplayColorAttr([gf.Vec3f(0.1, 0.4, 0.95)])
        actual = usd_geom.BasisCurves.Define(
            stage, "/World/BenchmarkMarkers/ActualPath"
        )
        actual.CreateTypeAttr("linear")
        actual.CreateCurveVertexCountsAttr([1])
        start = episode.start_position_world_m
        actual.CreatePointsAttr(
            [gf.Vec3f(float(start[0]), float(start[1]), 0.06)]
        )
        actual.CreateWidthsAttr([0.06])
        actual.CreateDisplayColorAttr([gf.Vec3f(1.0, 0.45, 0.05)])
        return actual

    def _world_to_cell(self, position: object) -> tuple[int, int]:
        xy = np.asarray(position, dtype=np.float64)[:2]
        continuous = (
            xy - self._occupancy.global_origin_xy
        ) / self._occupancy.resolution_m
        cell = np.rint(continuous).astype(np.int64)
        if np.any(cell < 0) or np.any(cell >= np.asarray(self._occupancy.grid.shape)):
            raise ValueError("episode position lies outside the runtime occupancy map")
        return int(cell[0]), int(cell[1])

    def _prepare_reference_planning(self, episode: BenchmarkEpisode) -> None:
        """Prepare privileged path state, skipping planning for diagnostics."""

        if self.config.diagnostic_open_plane:
            if self._reference_visualization_path is None:
                raise RuntimeError(
                    "diagnostic_open_plane requires an injected reference path"
                )
            # The diagnostic caller injects the actual geometry into
            # ``run_episode``.  Keep the same geometry available for runtime
            # display/provider compatibility, but do not run either planner.
            self._oracle_path = self._reference_visualization_path.copy()
            self._kinodynamic_plan = None
            return

        start_cell = self._world_to_cell(episode.start_position_world_m)
        goal_cell = self._world_to_cell(episode.goal_position_world_m)
        path_cells = astar_path(
            self._inflated_grid, start=start_cell, goal=goal_cell, occupied=1
        )
        self._oracle_path = (
            self._occupancy.global_origin_xy
            + path_cells.astype(np.float64) * self._occupancy.resolution_m
        )
        self._kinodynamic_plan = _plan_kinodynamic_reference(
            occupancy_grid=self._inflated_grid,
            resolution_m=self._occupancy.resolution_m,
            origin_xy=self._occupancy.global_origin_xy,
            episode=episode,
            drive_profile_path=self.config.drive_profile_path,
        )

    def execute(self, action_block: np.ndarray) -> RuntimeStep:
        if self._episode is None:
            raise RuntimeError("reset() must be called before execute()")
        self._block_physical_contact = False
        clipped_block = self._action_protocol.clip_block(action_block)
        raw_block = np.asarray(action_block, dtype=np.float32)
        raw_controls = raw_block.reshape(
            self._action_protocol.controls_per_block, 2
        )
        controls = clipped_block.reshape(
            self._action_protocol.controls_per_block, 2
        )
        diagnostic_trace = bool(
            getattr(self.config, "diagnostic_physics_trace", False)
        )
        diagnostic_block_index = int(getattr(self, "_diagnostic_block_index", 0))
        for index, (raw_target, target) in enumerate(zip(raw_controls, controls)):
            if self.config.gui and not self._simulation_app.is_running():
                raise RuntimeError("Isaac GUI was closed before the episode completed")
            if self._auxiliary_velocity_limit_rad_s() is None:
                applied = self._action_smoother.step(
                    target,
                    self._action_protocol.minimum,
                    self._action_protocol.maximum,
                )
            else:
                joint_positions = _finite_vector(
                    self._robot.get_joint_positions(), (3,), "joint_positions"
                )
                joint_velocities = _finite_vector(
                    self._robot.get_joint_velocities(), (3,), "joint_velocities"
                )
                applied = self._action_smoother.step(
                    target,
                    self._action_protocol.minimum,
                    self._action_protocol.maximum,
                    auxiliary_position_rad=float(joint_positions[1]),
                    auxiliary_velocity_rad_s=float(joint_velocities[1]),
                )
            smoothed = np.asarray(
                getattr(self._action_smoother, "last_smoothed_target", applied),
                dtype=np.float32,
            )
            self._controller.apply_action(
                self._ArticulationAction(
                    joint_velocities=np.asarray([applied[0], np.nan, np.nan], dtype=np.float32),
                    joint_positions=np.asarray([np.nan, applied[1], np.nan], dtype=np.float32),
                )
            )
            # Keep each commanded control on exactly one physics tick.  With
            # rendering_dt=.1 and physics_dt=.02, step(render=True) advances
            # five physics substeps, so use the non-rendering path here and
            # refresh the cameras once after the complete control block.
            self._world.step(render=False)
            if self._uses_physical_collision():
                self._sample_collision_contact("physics")
            self._previous_action = applied.copy()
            if diagnostic_trace:
                self._record_physics_diagnostic(
                    raw_target,
                    target,
                    smoothed,
                    applied,
                    action_block_index=diagnostic_block_index,
                    physics_step_in_block=index + 1,
                )
        self._world.render()
        self._history_actions.append(np.asarray(action_block, dtype=np.float32).copy())
        if diagnostic_trace:
            self._diagnostic_block_index = diagnostic_block_index + 1
        return self._capture()

    def flush_terminal_video_frames(self, frame_count: int = 5) -> None:
        """Advance one camera period after termination outside scored evaluation."""

        if self._episode is None:
            raise RuntimeError("reset() must be called before flushing video frames")
        if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count < 1:
            raise ValueError("frame_count must be a positive integer")
        if not self.config.record_video:
            return
        for _ in range(frame_count):
            self._world.step(render=True)
            left = self._get_rgb(self._left_camera)
            right = self._get_rgb(self._right_camera)
            third = self._get_rgb(self._third_camera)
            self._captured_frames["stereo_view"].append(
                np.concatenate((left, right), axis=1).copy()
            )
            self._captured_frames["third_person"].append(third.copy())

    def _uses_physical_collision(self) -> bool:
        # Missing mode is supported only for historical duck-typed test configs.
        return getattr(self.config, "collision_mode", "map_clearance_v1") == "physical_contact_v2"

    @property
    def collision_contact_samples(self) -> list[dict[str, object]]:
        return [dict(sample) for sample in self._collision_contact_samples]

    def _sample_collision_contact(self, source: str) -> bool:
        contact = self._physical_contact()
        available = bool(self._contact_available)
        time_s, step, _, _ = self._clock_metadata()
        self._collision_contact_samples.append({
            "simulation_time_s": time_s, "physics_step_index": step,
            "source": source, "available": available, "contact": bool(contact),
        })
        if not available:
            raise RuntimeError("obstacle contact measurement unavailable")
        self._block_physical_contact = self._block_physical_contact or contact
        return bool(self._block_physical_contact)

    def _collision_state(self, clearance: float) -> tuple[bool, bool]:
        if self._uses_physical_collision():
            contact = self._sample_collision_contact("capture")
            return contact, contact
        return self._physical_contact(), bool(clearance <= 0.0)

    def _physical_contact(self) -> bool:
        """Return contact with filtered scene obstacles, excluding the ground."""

        if self._contacts is None:
            return False
        try:
            forces = np.asarray(
                self._contacts.get_contact_force_matrix(
                    dt=1.0 / self._contract.schedule.physics_hz
                ),
                dtype=np.float64,
            )
            if forces.size == 0 or forces.shape[-1] != 3 or not np.all(np.isfinite(forces)):
                raise RuntimeError("invalid filtered contact-force tensor")
            return bool(np.any(np.linalg.norm(forces, axis=-1) > 1.0))
        except Exception:
            self._contact_available = False
            return False

    def _capture(self) -> RuntimeStep:
        (
            simulation_time_s,
            physics_step_index,
            raw_simulation_time_s,
            raw_physics_step_index,
        ) = self._clock_metadata()
        position, quaternion = self._robot.get_world_pose()
        position = _finite_vector(position, (3,), "position")
        quaternion = _finite_vector(quaternion, (4,), "quaternion")
        self._actual_positions.append(position[:2].astype(np.float64))
        self._actual_curve.GetCurveVertexCountsAttr().Set(
            [len(self._actual_positions)]
        )
        self._actual_curve.GetPointsAttr().Set(
            [
                self._Gf.Vec3f(float(point[0]), float(point[1]), 0.06)
                for point in self._actual_positions
            ]
        )
        left = self._get_rgb(self._left_camera)
        right = self._get_rgb(self._right_camera)
        third = self._get_rgb(self._third_camera)
        yaw = _yaw_from_wxyz(quaternion)
        local = self._crop_global_occupancy(
            self._occupancy.grid,
            self._occupancy.global_origin_xy,
            position,
            yaw,
            self._contract.bev,
        )
        clearance = float(
            self._obstacle_clearance(local, self.config.ball_radius_m, self._contract.bev)
        )
        joint_positions = _finite_vector(
            self._robot.get_joint_positions(), (3,), "joint_positions"
        )
        joint_velocities = _finite_vector(
            self._robot.get_joint_velocities(), (3,), "joint_velocities"
        )
        proprio = np.concatenate(
            (
                _finite_vector(self._robot.get_linear_velocity(), (3,), "linear_velocity"),
                _finite_vector(self._robot.get_angular_velocity(), (3,), "angular_velocity"),
                _gravity_in_base(quaternion),
                joint_positions,
                joint_velocities,
                self._previous_action,
            )
        ).astype(np.float32)
        self._history_proprio.append(proprio)
        self._history_proprio = self._history_proprio[-5:]
        self._history_quaternion_world_wxyz.append(quaternion.copy())
        self._history_quaternion_world_wxyz = self._history_quaternion_world_wxyz[-5:]
        if not self._history_actions:
            self._history_actions.append(np.zeros(10, dtype=np.float32))
        self._history_actions = self._history_actions[-5:]
        proprio_history = self._padded_history(self._history_proprio, 5)
        quaternion_history = self._padded_history(self._history_quaternion_world_wxyz, 5)
        action_history = self._padded_history(self._history_actions, 5)
        observation = PolicyObservation(
            stereo_left_rgb=left,
            stereo_right_rgb=right,
            proprio_history=proprio_history,
            action_history=action_history,
            position_world_xy=position[:2].astype(np.float64),
            yaw_world_rad=yaw,
            quaternion_world_wxyz_history=quaternion_history,
        )
        frames = {
            "left_rgb": left,
            "right_rgb": right,
            "stereo_view": np.concatenate((left, right), axis=1),
            "third_person": third,
        }
        if self.config.record_video:
            self._captured_frames["stereo_view"].append(frames["stereo_view"].copy())
            self._captured_frames["third_person"].append(third.copy())
        contact, collision = self._collision_state(clearance)
        return RuntimeStep(
            observation=observation,
            truth_clearance_m=clearance,
            physical_contact=contact,
            collision=collision,
            frames=frames,
            simulation_time_s=simulation_time_s,
            physics_step_index=physics_step_index,
            raw_simulation_time_s=raw_simulation_time_s,
            raw_physics_step_index=raw_physics_step_index,
            roll_world_rad=_roll_from_wxyz(quaternion),
        )

    @staticmethod
    def _padded_history(values: list[np.ndarray], length: int) -> np.ndarray:
        recent = values[-length:]
        if not recent:
            raise RuntimeError("cannot build an empty observation history")
        return np.stack([recent[0]] * (length - len(recent)) + recent).astype(np.float32)

    def close(self, return_code: int = 0) -> None:
        if self._closed:
            return
        self._closed = True
        if self._simulation_app is not None:
            if int(return_code) != 0:
                try:
                    self._simulation_app.app.post_quit(return_code=int(return_code))
                except Exception:
                    pass
            self._simulation_app.close()
