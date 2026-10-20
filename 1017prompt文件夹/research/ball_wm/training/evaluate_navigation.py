"""Closed-loop PointGoal navigation evaluation helpers and Isaac runtime entrypoint."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.ball_wm_dataset import (
        IMAGENET_MEAN,
        IMAGENET_STD,
        BallWMDataset,
        _gravity_in_base,
        _load_image,
        _load_episode,
        _normalize_image,
        _proprio17,
    )
    from training.action_protocol import (
        ActionProtocol,
        load_action_protocol,
        require_action_protocol,
    )
    from training.navigation_metrics import (
        action_block_to_controls,
        episode_termination,
        path_length,
        summarize_navigation,
    )
    from training.pose_utils import pointgoal_in_base
    from training.proprioception import (
        checkpoint_velocity_frame, transform_proprio_velocities, validate_velocity_frame,
    )
else:
    from .ball_wm_dataset import (
        IMAGENET_MEAN,
        IMAGENET_STD,
        BallWMDataset,
        _gravity_in_base,
        _load_image,
        _load_episode,
        _normalize_image,
        _proprio17,
    )
    from .action_protocol import ActionProtocol, load_action_protocol, require_action_protocol
    from .navigation_metrics import (
        action_block_to_controls,
        episode_termination,
        path_length,
        summarize_navigation,
    )
    from .pose_utils import pointgoal_in_base
    from .proprioception import (
        checkpoint_velocity_frame, transform_proprio_velocities, validate_velocity_frame,
    )


from scripts.camera_rig import CAMERA_FORWARD_M, CAMERA_HALF_BASELINE_M, CAMERA_UP_M


_CONTEXT_MANIFEST_SCHEMA = "pointgoal_context_manifest_v1"
_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"
_ACTIVE_RUNTIME: Any | None = None


@dataclass(frozen=True)
class NavigationContext:
    """One recorded context and its fixed world-frame PointGoal target."""

    episode_index: int
    context_start: int
    context_last_frame: int
    goal_frame: int
    start_state_index: int
    goal_state_index: int
    scene_path: str
    initial_distance_m: float
    initial_clearance_m: float


@dataclass(frozen=True)
class RuntimeObservation:
    """One simulator-generated observation at an MPC block boundary."""

    pixels_left: torch.Tensor
    pixels_right: torch.Tensor
    proprio: torch.Tensor
    position_xy: np.ndarray
    yaw: float
    clearance_m: float
    collision: bool
    trajectory_xy: np.ndarray


@dataclass(frozen=True)
class PlannerAction:
    """One CEM action block plus the selected plan's diagnostic cost terms."""

    action_block: np.ndarray
    diagnostics: Mapping[str, float]
    goal_relative_xy: np.ndarray | None = None
    predicted_relative_trajectory: np.ndarray | None = None


def resolve_planner_goal_source(
    *,
    policy: str,
    planner_mode: str,
    pose_head_enabled: bool,
    allow_target_image_diagnostic: bool,
) -> str:
    """Validate a policy's goal input and return the serialized goal-source label."""

    if policy not in {"cem", "random", "zero"}:
        raise ValueError("policy must be one of: cem, random, zero")
    if planner_mode not in {"latent", "pose_bev"}:
        raise ValueError("planner_mode must be one of: latent, pose_bev")
    if policy != "cem":
        return "not_used"
    if planner_mode == "pose_bev":
        if not pose_head_enabled:
            raise ValueError("pose_bev planner mode requires an enabled pose head")
        return "wheel_odometry_pointgoal"
    if not allow_target_image_diagnostic:
        raise ValueError("latent planner mode requires --allow_target_image_diagnostic")
    return "recorded_target_rgb_diagnostic"


def require_checkpoint_protocol(
    checkpoint: Mapping[str, Any], expected_protocol_id: str
) -> str:
    """Reject legacy or mismatched checkpoints before planning."""

    return require_action_protocol(
        checkpoint, expected_protocol_id, artifact="checkpoint"
    )


def _clip_action_block(
    action_block: np.ndarray, action_protocol: ActionProtocol
) -> np.ndarray:
    """Convert a model block into safe, per-physics-step joint commands."""

    controls = action_protocol.clip_block(action_block)
    return controls.reshape(action_protocol.controls_per_block, -1)


def sample_random_action_block(
    rng: np.random.Generator, action_protocol: ActionProtocol
) -> np.ndarray:
    """Sample one policy-comparable 10-D action block from contract bounds."""

    return action_protocol.sample_block(rng)


def collision_from_clearance(clearance_m: float) -> bool:
    """Use the dataset's analytic-BEV collision convention."""

    clearance = float(clearance_m)
    if np.isnan(clearance) or clearance == -np.inf:
        raise ValueError("clearance_m must be finite or positive infinity")
    return clearance <= 0.0


def _finite_vector(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.isfinite(result).all():
        raise RuntimeError(f"{name} must be finite with shape {shape}")
    return result


def _yaw_from_wxyz(quaternion_wxyz: np.ndarray) -> float:
    quaternion = _finite_vector(quaternion_wxyz, (4,), "quaternion_wxyz").astype(np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise RuntimeError("quaternion_wxyz must have non-zero norm")
    w, x, y, z = quaternion / norm
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _rgb_to_model_tensor(rgb: np.ndarray, image_size: tuple[int, int]) -> torch.Tensor:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] < 3 or not np.isfinite(image).all():
        raise RuntimeError("simulator RGB must be a finite HxWx3 array")
    if len(image_size) != 2 or min(int(value) for value in image_size) < 1:
        raise ValueError("image_size must be a positive (height, width) tuple")
    tensor = torch.from_numpy(np.ascontiguousarray(image[:, :, :3])).permute(2, 0, 1).float() / 255.0
    tensor = torch.nn.functional.interpolate(
        tensor.unsqueeze(0), size=tuple(int(value) for value in image_size), mode="bilinear", align_corners=False
    ).squeeze(0)
    return (tensor - IMAGENET_MEAN[:, None, None]) / IMAGENET_STD[:, None, None]


def _observation_from_state(
    *,
    left_rgb: np.ndarray,
    right_rgb: np.ndarray,
    position: np.ndarray,
    quaternion_wxyz: np.ndarray,
    linear_velocity: np.ndarray,
    angular_velocity: np.ndarray,
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    previous_action: np.ndarray,
    clearance_m: float,
    image_size: tuple[int, int],
    proprio_velocity_frame: str = "world",
) -> RuntimeObservation:
    """Convert a current simulator sensor/state snapshot to model inputs."""

    position_value = _finite_vector(position, (3,), "position")
    quaternion = _finite_vector(quaternion_wxyz, (4,), "quaternion_wxyz")
    linear = _finite_vector(linear_velocity, (3,), "linear_velocity")
    angular = _finite_vector(angular_velocity, (3,), "angular_velocity")
    joints = _finite_vector(joint_positions, (3,), "joint_positions")
    joint_rates = _finite_vector(joint_velocities, (3,), "joint_velocities")
    action = _finite_vector(previous_action, (2,), "previous_action")
    clearance = float(clearance_m)
    if np.isnan(clearance) or clearance == -np.inf:
        raise RuntimeError("clearance_m must be finite or positive infinity")
    proprio = np.concatenate(
        [linear, angular, _gravity_in_base(quaternion).astype(np.float32), joints, joint_rates, action]
    ).astype(np.float32)
    proprio = transform_proprio_velocities(proprio, quaternion, frame=proprio_velocity_frame)
    return RuntimeObservation(
        pixels_left=_rgb_to_model_tensor(left_rgb, image_size),
        pixels_right=_rgb_to_model_tensor(right_rgb, image_size),
        proprio=torch.from_numpy(proprio),
        position_xy=position_value[:2].copy(),
        yaw=_yaw_from_wxyz(quaternion),
        clearance_m=clearance,
        collision=collision_from_clearance(clearance),
        trajectory_xy=position_value[:2].reshape(1, 2).copy(),
    )


class IsaacNavigationRuntime:
    """Lazy Isaac Sim adapter that executes one five-control model block at a time.

    Importing this module remains safe for ordinary unit tests: simulator modules
    are imported only when a caller constructs this class.
    """

    def __init__(
        self,
        scene_path: str | Path,
        image_size: int | tuple[int, int],
        ball_radius_m: float,
        device: str,
        proprio_velocity_frame: str = "world",
    ) -> None:
        self._simulation_app: Any | None = None
        self._closed = False
        self._initialization_error: str | None = None
        self._trajectory: list[np.ndarray] = []
        self._previous_action = np.zeros(2, dtype=np.float32)
        self.device = str(device)
        self.proprio_velocity_frame = validate_velocity_frame(proprio_velocity_frame)
        self.scene_path = Path(scene_path)
        if not self.scene_path.is_file():
            raise FileNotFoundError(f"Isaac scene does not exist: {self.scene_path}")
        if isinstance(image_size, int):
            self.image_size = (int(image_size), int(image_size))
        else:
            self.image_size = tuple(int(value) for value in image_size)
        if len(self.image_size) != 2 or min(self.image_size) < 1:
            raise ValueError("image_size must be a positive integer or (height, width)")
        self.ball_radius_m = float(ball_radius_m)
        if not np.isfinite(self.ball_radius_m) or self.ball_radius_m <= 0.0:
            raise ValueError("ball_radius_m must be finite and positive")

        try:
            self._initialize()
        except Exception as exc:
            # Kit shutdown terminates this Python process in the target Isaac
            # environment.  Preserve the error for the evaluator to serialize
            # before the CLI performs that shutdown.
            self._initialization_error = f"{type(exc).__name__}: {exc}"

    def _ensure_ready(self) -> None:
        if self._initialization_error is not None:
            raise RuntimeError(f"Isaac runtime initialization failed: {self._initialization_error}")

    def _initialize(self) -> None:
        # SimulationApp must precede every omni/pxr import in a process.
        from isaacsim import SimulationApp

        self._simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})
        import carb
        import omni.usd
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.prims import XFormPrim
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
        from omni.isaac.sensor import Camera
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

        script_dir = Path(__file__).resolve().parents[1] / "scripts"
        import sys

        if str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))
        from action_smoothing import ActionSmoother, default_articulation_gains
        from bev_ground_truth import build_analytic_global_map, crop_global_occupancy, obstacle_clearance
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

        contract_path = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"
        self._contract = load_contract(contract_path)
        self._action_protocol = ActionProtocol.from_contract(self._contract)
        self._crop_global_occupancy = crop_global_occupancy
        self._obstacle_clearance = obstacle_clearance
        self._ArticulationAction = ArticulationAction
        self._proportional_gains, self._derivative_gains = default_articulation_gains()
        self._action_smoother = ActionSmoother(
            dt_s=1.0 / self._contract.schedule.physics_hz,
        )
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / self._contract.schedule.physics_hz,
            rendering_dt=1.0 / self._contract.schedule.stereo_hz,
        )
        scene_root = _load_collection_scene(
            self._world,
            str(self.scene_path),
            add_reference_to_stage,
            XFormPrim,
            print_fn=lambda _message: None,
        )
        if scene_root is None:
            raise RuntimeError("closed-loop evaluation requires a non-empty external scene")
        stage = omni.usd.get_context().get_stage()
        for path in ("/World/SunLight", "/World/up_disk", "/World/dn_disk"):
            stage.RemovePrim(path)
        sunlight = UsdLux.DistantLight.Define(stage, "/World/SunLight")
        sunlight.CreateIntensityAttr(1000.0)
        for path, z_offset, rotation in (
            ("/World/up_disk", 1.0, 180.0),
            ("/World/dn_disk", -1.0, 0.0),
        ):
            light = UsdLux.DiskLight.Define(stage, path)
            light.CreateIntensityAttr(5000.0)
            light.CreateRadiusAttr(50.0)
            xform = UsdGeom.Xformable(light)
            xform.ClearXformOpOrder()
            xform.AddRotateXYZOp().Set(Gf.Vec3f(rotation, 0.0, 0.0))
            xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, z_offset))
        carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLighting")

        if not Path(ROBOT_USD).is_file():
            raise FileNotFoundError(f"robot USD does not exist: {ROBOT_USD}")
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        self._robot = self._world.scene.add(
            Articulation(
                prim_path=ROBOT_ROOT,
                name="ball_robot",
                position=np.asarray([0.0, 0.0, 0.45], dtype=np.float32),
            )
        )
        camera_x, camera_y, camera_z = CAMERA_FORWARD_M, CAMERA_HALF_BASELINE_M, CAMERA_UP_M
        focal_length, horizontal_aperture = 16.45, 18.0
        vertical_aperture = horizontal_aperture * self._contract.image_height_px / self._contract.image_width_px

        def add_camera_prim(path: str, y_offset: float) -> None:
            camera_prim = UsdGeom.Camera.Define(stage, path)
            camera_prim.GetFocalLengthAttr().Set(focal_length)
            camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
            xform = UsdGeom.Xformable(camera_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(camera_x, y_offset, camera_z))
            xform.AddRotateYZXOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))

        add_camera_prim(LEFT_CAMERA_PATH, camera_y)
        add_camera_prim(RIGHT_CAMERA_PATH, -camera_y)
        self._world.reset()
        self._left_camera = Camera(
            prim_path=LEFT_CAMERA_PATH,
            resolution=(self._contract.image_width_px, self._contract.image_height_px),
            frequency=self._contract.schedule.stereo_hz,
        )
        self._right_camera = Camera(
            prim_path=RIGHT_CAMERA_PATH,
            resolution=(self._contract.image_width_px, self._contract.image_height_px),
            frequency=self._contract.schedule.stereo_hz,
        )
        self._left_camera.initialize()
        self._right_camera.initialize()
        # Retain the exact collector sensor setup so camera readiness is checked
        # against its depth annotators before the first evaluation episode.
        from bev_ground_truth import enable_camera_depth

        enable_camera_depth(self._left_camera)
        enable_camera_depth(self._right_camera)
        _camera_optical_to_base(stage, LEFT_CAMERA_PATH)
        _camera_optical_to_base(stage, RIGHT_CAMERA_PATH)
        _warmup_physics_scene(self._world, steps=10)
        self._occupancy = build_analytic_global_map(
            stage,
            scene_root,
            ROBOT_ROOT,
            self._contract.bev,
            Usd,
            UsdGeom,
            UsdPhysics,
        )
        self._controller = _reset_articulation_after_occupancy(
            self._world,
            self._robot,
            self._proportional_gains,
            self._derivative_gains,
        )
        _prime_stereo_cameras(self._world, self._left_camera, self._right_camera)
        self._get_rgb = _get_rgb

    def reset_to_context(self, state_row: Mapping[str, np.ndarray]) -> RuntimeObservation:
        """Restore a saved 50-Hz state and return a fresh simulator observation."""

        self._ensure_ready()
        required = ("pos", "quat", "lin_vel", "ang_vel", "joint_pos", "joint_vel", "applied_action")
        if any(name not in state_row for name in required):
            missing = [name for name in required if name not in state_row]
            raise RuntimeError(f"saved state is missing fields: {missing}")
        position = _finite_vector(state_row["pos"], (3,), "pos")
        quaternion = _finite_vector(state_row["quat"], (4,), "quat")
        linear_velocity = _finite_vector(state_row["lin_vel"], (3,), "lin_vel")
        angular_velocity = _finite_vector(state_row["ang_vel"], (3,), "ang_vel")
        joint_positions = _finite_vector(state_row["joint_pos"], (3,), "joint_pos")
        joint_velocities = _finite_vector(state_row["joint_vel"], (3,), "joint_vel")
        self._previous_action = _finite_vector(state_row["applied_action"], (2,), "applied_action")
        self._action_smoother.reset(
            np.clip(
                self._previous_action,
                np.asarray(self._action_protocol.minimum, dtype=np.float32),
                np.asarray(self._action_protocol.maximum, dtype=np.float32),
            )
        )

        self._world.reset()
        self._controller = self._robot.get_articulation_controller()
        self._controller.set_gains(kps=self._proportional_gains, kds=self._derivative_gains)
        self._robot.set_world_pose(position=position, orientation=quaternion)
        self._robot.set_linear_velocity(linear_velocity)
        self._robot.set_angular_velocity(angular_velocity)
        self._robot.set_joint_positions(joint_positions)
        self._robot.set_joint_velocities(joint_velocities)
        self._controller.apply_action(
            self._ArticulationAction(
                joint_velocities=np.asarray([0.0, np.nan, np.nan], dtype=np.float32),
                joint_positions=np.asarray([np.nan, 0.0, np.nan], dtype=np.float32),
            )
        )
        self._world.step(render=False)
        self._trajectory = [position[:2].copy()]
        return self._capture_observation()

    def execute_action_block(self, action_block: np.ndarray) -> RuntimeObservation:
        """Execute exactly five bounded 50-Hz controls and return the final frame."""

        self._ensure_ready()
        controls = _clip_action_block(action_block, self._action_protocol)
        for control_index, control in enumerate(controls):
            applied_control = self._action_smoother.step(
                control,
                self._action_protocol.minimum,
                self._action_protocol.maximum,
            )
            self._controller.apply_action(
                self._ArticulationAction(
                    joint_velocities=np.asarray([applied_control[0], np.nan, np.nan], dtype=np.float32),
                    joint_positions=np.asarray([np.nan, applied_control[1], np.nan], dtype=np.float32),
                )
            )
            self._world.step(render=control_index == len(controls) - 1)
            self._previous_action = applied_control.copy()
        return self._capture_observation()

    def _capture_observation(self) -> RuntimeObservation:
        position, quaternion = self._robot.get_world_pose()
        position_value = _finite_vector(position, (3,), "simulator position")
        quaternion_value = _finite_vector(quaternion, (4,), "simulator quaternion")
        yaw = _yaw_from_wxyz(quaternion_value)
        geometry_bev = self._crop_global_occupancy(
            self._occupancy.grid,
            self._occupancy.global_origin_xy,
            position_value,
            yaw,
            self._contract.bev,
        )
        clearance = self._obstacle_clearance(geometry_bev, self.ball_radius_m, self._contract.bev)
        observation = _observation_from_state(
            left_rgb=self._get_rgb(self._left_camera),
            right_rgb=self._get_rgb(self._right_camera),
            position=position_value,
            quaternion_wxyz=quaternion_value,
            linear_velocity=self._robot.get_linear_velocity(),
            angular_velocity=self._robot.get_angular_velocity(),
            joint_positions=self._robot.get_joint_positions(),
            joint_velocities=self._robot.get_joint_velocities(),
            previous_action=self._previous_action,
            clearance_m=clearance,
            image_size=self.image_size,
            proprio_velocity_frame=self.proprio_velocity_frame,
        )
        self._trajectory.append(observation.position_xy.copy())
        return replace(observation, trajectory_xy=np.stack(self._trajectory).astype(np.float32))

    def close(self, return_code: int = 0) -> None:
        """Close Kit exactly once, including after partial initialization failures."""

        if self._closed:
            return
        self._closed = True
        if self._simulation_app is not None:
            if int(return_code) != 0:
                self._simulation_app.app.post_quit(return_code=int(return_code))
            self._simulation_app.close()


def _matches_scene(scene_name: str | None, episode_path: Path, metadata_scene: str) -> bool:
    if scene_name is None:
        return True
    return scene_name == metadata_scene or scene_name in episode_path.parts or scene_name in metadata_scene


def select_navigation_contexts(
    dataset: BallWMDataset,
    scene_name: str | None,
    num_eval: int,
    goal_offset: int,
    seed: int,
    min_initial_goal_distance_m: float = 0.0,
    require_initial_clearance: bool = False,
    min_initial_clearance_m: float = 0.0,
) -> list[NavigationContext]:
    """Choose one reproducible valid context per episode for fair policy comparison."""

    if num_eval < 0:
        raise ValueError("num_eval must be non-negative")
    if goal_offset < 1:
        raise ValueError("goal_offset must be positive")
    if not math.isfinite(float(min_initial_goal_distance_m)) or min_initial_goal_distance_m < 0.0:
        raise ValueError("min_initial_goal_distance_m must be finite and non-negative")
    if not math.isfinite(float(min_initial_clearance_m)) or min_initial_clearance_m < 0.0:
        raise ValueError("min_initial_clearance_m must be finite and non-negative")
    history_size = int(dataset.config.history_frames)
    rng = np.random.default_rng(seed)
    candidates: dict[int, list[NavigationContext]] = {}

    # Deliberately derive legal starts from the raw episode rather than
    # ``dataset.samples``.  The latter changes with a checkpoint's training
    # rollout horizon, which would make a fixed seed select different contexts
    # for baseline and candidate models.
    for episode_index, episode_path in enumerate(dataset.episode_paths):
        episode = _load_episode(str(episode_path))
        metadata = episode["metadata"]
        if not isinstance(metadata, dict):
            raise RuntimeError("episode metadata must be a mapping")
        scene_path = str(metadata.get("scene", episode_path))
        if not _matches_scene(scene_name, episode_path, scene_path):
            continue
        num_frames = int(episode["num_frames"])
        states = episode["states"]
        if not isinstance(states, dict):
            raise RuntimeError("episode states must be a mapping")
        state_positions = states.get("pos")
        recorded_collision = states.get("collision")
        recorded_clearance = states.get("minimum_obstacle_distance")
        max_context_start = num_frames - history_size - goal_offset
        for context_start in range(max(0, max_context_start)):
            context_last_frame = int(context_start) + history_size - 1
            goal_frame = context_last_frame + goal_offset
            start_state_index = 5 * (context_last_frame + 1) - 1
            goal_state_index = 5 * (goal_frame + 1) - 1
            if state_positions is not None and goal_state_index >= len(state_positions):
                continue
            initial_distance_m = 0.0
            if state_positions is not None:
                if start_state_index >= len(state_positions):
                    continue
                start_xy = np.asarray(state_positions[start_state_index, :2], dtype=np.float64)
                goal_xy = np.asarray(state_positions[goal_state_index, :2], dtype=np.float64)
                if start_xy.shape != (2,) or goal_xy.shape != (2,) or not np.isfinite(start_xy).all() or not np.isfinite(goal_xy).all():
                    continue
                initial_distance_m = float(np.linalg.norm(goal_xy - start_xy))
            if initial_distance_m < float(min_initial_goal_distance_m):
                continue

            initial_clearance_m = float("inf")
            if recorded_clearance is not None and context_last_frame < len(recorded_clearance):
                initial_clearance_m = float(recorded_clearance[context_last_frame])
            initial_collision = False
            if recorded_collision is not None and context_last_frame < len(recorded_collision):
                initial_collision = bool(recorded_collision[context_last_frame])
            if require_initial_clearance and (
                recorded_collision is None
                or recorded_clearance is None
                or context_last_frame >= len(recorded_collision)
                or context_last_frame >= len(recorded_clearance)
                or initial_collision
                or not math.isfinite(initial_clearance_m)
                or initial_clearance_m <= 0.0
                or initial_clearance_m < float(min_initial_clearance_m)
            ):
                continue
            candidates.setdefault(int(episode_index), []).append(
                NavigationContext(
                    episode_index=int(episode_index),
                    context_start=int(context_start),
                    context_last_frame=context_last_frame,
                    goal_frame=goal_frame,
                    start_state_index=start_state_index,
                    goal_state_index=goal_state_index,
                    scene_path=scene_path,
                    initial_distance_m=initial_distance_m,
                    initial_clearance_m=initial_clearance_m,
                )
            )

    selected_per_episode = [
        contexts[int(rng.integers(len(contexts)))]
        for _, contexts in sorted(candidates.items())
    ]
    if num_eval > len(selected_per_episode):
        raise ValueError(
            f"requested {num_eval} navigation contexts but only {len(selected_per_episode)} episodes are available"
        )
    selected_count = len(selected_per_episode) if num_eval == 0 else num_eval
    indices = rng.permutation(len(selected_per_episode))[:selected_count]
    return [selected_per_episode[int(index)] for index in indices]


def write_context_manifest(
    path: Path,
    data_root: Path,
    history_size: int,
    goal_offset: int,
    min_initial_goal_distance_m: float,
    require_initial_clearance: bool,
    seed: int,
    contexts: Sequence[NavigationContext],
    min_initial_clearance_m: float = 0.0,
) -> dict[str, object]:
    """Write a strict, self-validating context manifest shared by all policies."""

    if history_size < 1 or goal_offset < 1:
        raise ValueError("history_size and goal_offset must be positive")
    if not math.isfinite(float(min_initial_goal_distance_m)) or min_initial_goal_distance_m < 0.0:
        raise ValueError("min_initial_goal_distance_m must be finite and non-negative")
    if not math.isfinite(float(min_initial_clearance_m)) or min_initial_clearance_m < 0.0:
        raise ValueError("min_initial_clearance_m must be finite and non-negative")
    root = Path(data_root).resolve()
    seen_episode_indices: set[int] = set()
    entries: list[dict[str, object]] = []
    for context in contexts:
        if context.episode_index in seen_episode_indices:
            raise ValueError("context manifest contains duplicate episode_index")
        if not math.isfinite(context.initial_distance_m) or context.initial_distance_m < float(min_initial_goal_distance_m):
            raise ValueError("context initial_distance_m does not satisfy manifest protocol")
        if (
            not math.isfinite(context.initial_clearance_m)
            or context.initial_clearance_m <= 0.0
            or context.initial_clearance_m < float(min_initial_clearance_m)
        ):
            raise ValueError("context initial_clearance_m must be finite and positive")
        seen_episode_indices.add(context.episode_index)
        episode_path = root / str(context.episode_index)
        entries.append(
            {
                "episode_index": int(context.episode_index),
                "context_start": int(context.context_start),
                "context_last_frame": int(context.context_last_frame),
                "goal_frame": int(context.goal_frame),
                "start_state_index": int(context.start_state_index),
                "goal_state_index": int(context.goal_state_index),
                "scene_path": str(context.scene_path),
                "initial_distance_m": float(context.initial_distance_m),
                "initial_clearance_m": float(context.initial_clearance_m),
            }
        )
    payload: dict[str, object] = {
        "schema_version": _CONTEXT_MANIFEST_SCHEMA,
        "data_root": str(root),
        "history_frames": int(history_size),
        "goal_offset": int(goal_offset),
        "min_initial_goal_distance_m": float(min_initial_goal_distance_m),
        "min_initial_clearance_m": float(min_initial_clearance_m),
        "require_initial_clearance": bool(require_initial_clearance),
        "seed": int(seed),
        "num_contexts": len(entries),
        "contexts": entries,
    }
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2, allow_nan=False))
    return payload


def load_context_manifest(
    path: Path,
    data_root: Path,
    dataset: BallWMDataset,
    scene_name: str | None,
) -> list[NavigationContext]:
    """Load a manifest and re-check every context against its raw recorded state."""

    try:
        payload = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read context manifest: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != _CONTEXT_MANIFEST_SCHEMA:
        raise ValueError("context manifest has an unsupported schema_version")
    root = Path(data_root).resolve()
    if Path(str(payload.get("data_root", ""))).resolve() != root:
        raise ValueError("context manifest data_root does not match evaluation data_root")
    history_size = payload.get("history_frames")
    goal_offset = payload.get("goal_offset")
    min_distance = payload.get("min_initial_goal_distance_m")
    min_clearance = payload.get("min_initial_clearance_m", 0.0)
    require_clearance = payload.get("require_initial_clearance")
    entries = payload.get("contexts")
    if (
        not isinstance(history_size, int)
        or history_size != int(dataset.config.history_frames)
        or not isinstance(goal_offset, int)
        or goal_offset < 1
        or not isinstance(min_distance, (int, float))
        or not math.isfinite(float(min_distance))
        or float(min_distance) < 0.0
        or not isinstance(min_clearance, (int, float))
        or not math.isfinite(float(min_clearance))
        or float(min_clearance) < 0.0
        or not isinstance(require_clearance, bool)
        or not isinstance(entries, list)
        or payload.get("num_contexts") != len(entries)
    ):
        raise ValueError("context manifest has an invalid protocol or context count")

    seen_episode_indices: set[int] = set()
    contexts: list[NavigationContext] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("context manifest entry must be an object")
        required_integer_fields = (
            "episode_index",
            "context_start",
            "context_last_frame",
            "goal_frame",
            "start_state_index",
            "goal_state_index",
        )
        if any(not isinstance(entry.get(field), int) for field in required_integer_fields):
            raise ValueError("context manifest has non-integer frame or state fields")
        episode_index = int(entry["episode_index"])
        if episode_index in seen_episode_indices:
            raise ValueError("context manifest has duplicate episode_index")
        if episode_index < 0 or episode_index >= len(dataset.episode_paths):
            raise ValueError("context manifest episode_index is not present in dataset")
        episode_path = Path(dataset.episode_paths[episode_index])
        data = _load_episode(str(episode_path))
        states = data.get("states")
        metadata = data.get("metadata")
        if not isinstance(states, dict) or not isinstance(metadata, dict):
            raise ValueError("context manifest episode has invalid recorded data")
        scene_path = str(metadata.get("scene", episode_path))
        context_start = int(entry["context_start"])
        context_last_frame = int(entry["context_last_frame"])
        goal_frame = int(entry["goal_frame"])
        start_state_index = int(entry["start_state_index"])
        goal_state_index = int(entry["goal_state_index"])
        if context_start < 0 or context_last_frame != context_start + history_size - 1:
            raise ValueError("context manifest context_last_frame is not aligned")
        if goal_frame != context_last_frame + goal_offset:
            raise ValueError("context manifest goal_frame is not aligned")
        if start_state_index != 5 * (context_last_frame + 1) - 1:
            raise ValueError("context manifest start_state_index is not aligned")
        if goal_state_index != 5 * (goal_frame + 1) - 1:
            raise ValueError("context manifest goal_state_index is not aligned")
        if goal_frame >= int(data["num_frames"]) or "pos" not in states:
            raise ValueError("context manifest goal_frame is outside the recorded episode")
        positions = np.asarray(states["pos"])
        if start_state_index >= len(positions) or goal_state_index >= len(positions):
            raise ValueError("context manifest state index is outside the recorded episode")
        start_xy = np.asarray(positions[start_state_index, :2], dtype=np.float64)
        goal_xy = np.asarray(positions[goal_state_index, :2], dtype=np.float64)
        initial_distance_m = float(np.linalg.norm(goal_xy - start_xy))
        if not math.isfinite(initial_distance_m) or initial_distance_m < float(min_distance):
            raise ValueError("context manifest initial_distance_m does not satisfy protocol")
        collision = states.get("collision")
        clearance = states.get("minimum_obstacle_distance")
        if collision is None or clearance is None or context_last_frame >= len(collision) or context_last_frame >= len(clearance):
            raise ValueError("context manifest initial clearance fields are unavailable")
        initial_clearance_m = float(clearance[context_last_frame])
        if (
            not math.isfinite(initial_clearance_m)
            or initial_clearance_m <= 0.0
            or initial_clearance_m < float(min_clearance)
            or bool(collision[context_last_frame])
        ):
            raise ValueError("context manifest initial_clearance_m does not satisfy protocol")
        stored_distance = entry.get("initial_distance_m")
        stored_clearance = entry.get("initial_clearance_m")
        if (
            not isinstance(stored_distance, (int, float))
            or not isinstance(stored_clearance, (int, float))
            or not math.isclose(float(stored_distance), initial_distance_m, abs_tol=1e-5)
            or not math.isclose(float(stored_clearance), initial_clearance_m, abs_tol=1e-5)
        ):
            raise ValueError("context manifest initial diagnostics do not match raw data")
        if str(entry.get("scene_path")) != scene_path:
            raise ValueError("context manifest scene_path does not match raw metadata")
        seen_episode_indices.add(episode_index)
        contexts.append(
            NavigationContext(
                episode_index=episode_index,
                context_start=context_start,
                context_last_frame=context_last_frame,
                goal_frame=goal_frame,
                start_state_index=start_state_index,
                goal_state_index=goal_state_index,
                scene_path=scene_path,
                initial_distance_m=initial_distance_m,
                initial_clearance_m=initial_clearance_m,
            )
        )
    return [context for context in contexts if _matches_scene(scene_name, dataset.episode_paths[context.episode_index], context.scene_path)]


def resolve_navigation_contexts(
    dataset: BallWMDataset,
    scene_name: str | None,
    num_eval: int,
    goal_offset: int,
    seed: int,
    context_manifest_path: Path | None,
    min_initial_goal_distance_m: float,
    require_initial_clearance: bool,
    min_initial_clearance_m: float = 0.0,
) -> list[NavigationContext]:
    """Resolve test contexts from a shared manifest or direct seeded selection."""

    if context_manifest_path is None:
        return select_navigation_contexts(
            dataset,
            scene_name,
            num_eval,
            goal_offset,
            seed,
            min_initial_goal_distance_m=min_initial_goal_distance_m,
            require_initial_clearance=require_initial_clearance,
            min_initial_clearance_m=min_initial_clearance_m,
        )
    try:
        payload = json.loads(Path(context_manifest_path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read context manifest: {context_manifest_path}") from exc
    if not isinstance(payload, dict):
        raise ValueError("context manifest must be an object")
    if (
        payload.get("goal_offset") != int(goal_offset)
        or not math.isclose(
            float(payload.get("min_initial_goal_distance_m", float("nan"))),
            float(min_initial_goal_distance_m),
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(payload.get("min_initial_clearance_m", 0.0)),
            float(min_initial_clearance_m),
            abs_tol=1e-9,
        )
        or payload.get("require_initial_clearance") is not bool(require_initial_clearance)
    ):
        raise ValueError("context manifest protocol does not match evaluator arguments")
    if not dataset.episode_paths:
        raise ValueError("cannot resolve a manifest for an empty dataset")
    dataset_root = Path(dataset.episode_paths[0]).resolve().parent.parent
    return load_context_manifest(context_manifest_path, dataset_root, dataset, scene_name)


def build_model_context(
    sample: Mapping[str, torch.Tensor],
    history_size: int,
) -> dict[str, torch.Tensor]:
    """Create a batched history, reserving the final action row for the planner."""

    if history_size < 1:
        raise ValueError("history_size must be positive")
    required = ("pixels_left", "pixels_right", "action", "proprio")
    if any(name not in sample for name in required):
        raise ValueError("sample is missing a model context tensor")
    context = {name: sample[name][:history_size].clone().unsqueeze(0) for name in required}
    if context["action"].size(1) != history_size:
        raise ValueError("sample does not contain the requested history")
    context["action"][:, -1] = 0.0
    return context


def _recorded_goal_info(
    dataset: BallWMDataset,
    episode_index: int,
    goal_frame: int,
) -> dict[str, torch.Tensor]:
    """Encode the fixed PointGoal target from its one recorded stereo/state frame.

    The target frame supplies CEM's latent goal only.  No target action nor a
    later recorded observation enters the executing policy context.
    """

    episode = dataset.episode_paths[episode_index]
    data = _load_episode(str(episode))
    if goal_frame < 0 or goal_frame >= int(data["num_frames"]):
        raise ValueError("goal_frame is outside the recorded episode")
    states = data["states"]
    if not isinstance(states, dict):
        raise RuntimeError("episode states must be a mapping")
    state_index = 5 * (int(goal_frame) + 1) - 1
    if state_index >= len(states["pos"]):
        raise ValueError("goal frame does not have an aligned state")
    image_size = tuple(int(value) for value in dataset.config.image_size)
    left = _load_image(episode / f"left_{goal_frame:06d}.png", image_size)
    right = _load_image(episode / f"right_{goal_frame:06d}.png", image_size)
    if dataset.config.normalize:
        left = _normalize_image(left)
        right = _normalize_image(right)
    return {
        "pixels_left": left.unsqueeze(0).unsqueeze(0),
        "pixels_right": right.unsqueeze(0).unsqueeze(0),
        "action": torch.zeros((1, 1, 10), dtype=torch.float32),
        "proprio": torch.from_numpy(_proprio17(
            states, state_index,
            velocity_frame=getattr(dataset.config, "proprio_velocity_frame", "world"),
        )).unsqueeze(0).unsqueeze(0),
    }


def _recorded_context_sample(
    dataset: BallWMDataset,
    episode_index: int,
    context_start: int,
) -> dict[str, torch.Tensor]:
    """Load exactly the recorded visual history without requiring future labels."""

    episode = dataset.episode_paths[episode_index]
    data = _load_episode(str(episode))
    states = data["states"]
    if not isinstance(states, dict):
        raise RuntimeError("episode states must be a mapping")
    history_size = int(dataset.config.history_frames)
    if context_start < 0 or context_start + history_size > int(data["num_frames"]):
        raise ValueError("context history is outside the recorded episode")
    image_size = tuple(int(value) for value in dataset.config.image_size)
    action_block = int(getattr(dataset.config, "action_block", 5))
    left_images: list[torch.Tensor] = []
    right_images: list[torch.Tensor] = []
    proprio_rows: list[np.ndarray] = []
    action_rows: list[np.ndarray] = []
    for frame in range(context_start, context_start + history_size):
        left = _load_image(episode / f"left_{frame:06d}.png", image_size)
        right = _load_image(episode / f"right_{frame:06d}.png", image_size)
        if dataset.config.normalize:
            left = _normalize_image(left)
            right = _normalize_image(right)
        left_images.append(left)
        right_images.append(right)
        state_index = 5 * (frame + 1) - 1
        next_start = 5 * (frame + 1)
        next_end = next_start + action_block
        if state_index >= len(states["pos"]):
            raise ValueError("context frame does not have an aligned state")
        proprio_rows.append(_proprio17(
            states, state_index,
            velocity_frame=getattr(dataset.config, "proprio_velocity_frame", "world"),
        ))
        action_source = getattr(dataset.config, "action_source", "applied_action")
        if next_end <= len(states[action_source]):
            action_rows.append(states[action_source][next_start:next_end].reshape(-1))
        else:
            action_rows.append(np.zeros(action_block * 2, dtype=np.float32))
    return {
        "pixels_left": torch.stack(left_images),
        "pixels_right": torch.stack(right_images),
        "action": torch.from_numpy(np.stack(action_rows).astype(np.float32)),
        "proprio": torch.from_numpy(np.stack(proprio_rows).astype(np.float32)),
    }


def _json_compatible(value: object) -> object:
    """Convert NumPy values and non-finite diagnostics to strict JSON values."""

    if isinstance(value, np.ndarray):
        return _json_compatible(value.tolist())
    if isinstance(value, np.generic):
        return _json_compatible(value.item())
    if isinstance(value, Mapping):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


def render_diagnostic_trajectory_plot(
    output_path: Path,
    episodes: Sequence[Mapping[str, object]],
    *,
    title: str,
) -> Path:
    """Render executed PointGoal trajectories, starts, goals, and endpoints to a PNG."""

    if not isinstance(title, str) or not title.strip():
        raise ValueError("title must be a non-empty string")
    records: list[tuple[np.ndarray, np.ndarray]] = []
    for episode in episodes:
        trajectory = np.asarray(episode.get("trajectory_xy", []), dtype=np.float64)
        goal = np.asarray(episode.get("goal_xy", []), dtype=np.float64)
        if trajectory.ndim != 2 or trajectory.shape[1:] != (2,) or not np.isfinite(trajectory).all():
            continue
        if len(trajectory) < 1 or goal.shape != (2,) or not np.isfinite(goal).all():
            continue
        records.append((trajectory, goal))
    if not records:
        raise ValueError("diagnostic plot requires at least one finite trajectory and goal")

    all_points = np.concatenate(
        [np.concatenate((trajectory, goal.reshape(1, 2)), axis=0) for trajectory, goal in records],
        axis=0,
    )
    world_min = all_points.min(axis=0)
    world_max = all_points.max(axis=0)
    span = np.maximum(world_max - world_min, 1.0)
    padding = 0.1 * float(max(span))
    world_min -= padding
    world_max += padding
    span = world_max - world_min

    width, height, margin, header = 960, 640, 56, 72
    plot_width = width - 2 * margin
    plot_height = height - header - margin
    scale = min(plot_width / span[0], plot_height / span[1])
    draw_width = span[0] * scale
    draw_height = span[1] * scale
    left = margin + (plot_width - draw_width) / 2.0
    top = header + (plot_height - draw_height) / 2.0

    def to_pixel(point: np.ndarray) -> tuple[int, int]:
        return (
            int(round(left + (float(point[0]) - world_min[0]) * scale)),
            int(round(top + (world_max[1] - float(point[1])) * scale)),
        )

    image = Image.new("RGB", (width, height), (250, 251, 252))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((margin, 20), title, fill=(28, 35, 42), font=font)
    draw.text(
        (margin, 42),
        "line: executed path   circle: start   cross: PointGoal   square: endpoint",
        fill=(70, 79, 87),
        font=font,
    )
    draw.rectangle((int(left), int(top), int(left + draw_width), int(top + draw_height)), outline=(156, 163, 175), width=2)
    colors = ((31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189))
    for index, (trajectory, goal) in enumerate(records):
        color = colors[index % len(colors)]
        pixels = [to_pixel(point) for point in trajectory]
        if len(pixels) > 1:
            draw.line(pixels, fill=color, width=3, joint="curve")
        start_x, start_y = pixels[0]
        end_x, end_y = pixels[-1]
        goal_x, goal_y = to_pixel(goal)
        radius = 5
        draw.ellipse((start_x - radius, start_y - radius, start_x + radius, start_y + radius), outline=color, width=2)
        draw.rectangle((end_x - radius, end_y - radius, end_x + radius, end_y + radius), fill=color)
        draw.line((goal_x - radius, goal_y - radius, goal_x + radius, goal_y + radius), fill=(0, 128, 0), width=2)
        draw.line((goal_x - radius, goal_y + radius, goal_x + radius, goal_y - radius), fill=(0, 128, 0), width=2)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path, format="PNG")
    return output_path


def evaluation_exit_code(result: Mapping[str, object]) -> int:
    """Return the CLI status required by the serialized runtime outcome."""

    return 1 if int(result.get("runtime_error_count", 0)) > 0 else 0


def _state_row(states: Mapping[str, np.ndarray], index: int) -> dict[str, np.ndarray]:
    required = ("pos", "quat", "lin_vel", "ang_vel", "joint_pos", "joint_vel", "applied_action")
    if any(name not in states for name in required):
        missing = [name for name in required if name not in states]
        raise RuntimeError(f"episode states are missing fields: {missing}")
    if index < 0 or index >= len(states["pos"]):
        raise ValueError("state index is outside the recorded episode")
    return {name: np.asarray(states[name][index]).copy() for name in required}


def _recorded_reference_length(
    states: Mapping[str, np.ndarray],
    start_state_index: int,
    goal_state_index: int,
) -> float:
    """Use the recorded expert segment as the SPL reference-path diagnostic."""

    if "pos" not in states:
        raise RuntimeError("episode states are missing pos")
    positions = np.asarray(states["pos"])
    if start_state_index < 0 or goal_state_index >= len(positions) or goal_state_index < start_state_index:
        raise ValueError("invalid reference state range")
    return path_length(positions[start_state_index : goal_state_index + 1, :2])


def _runtime_error_episode(
    context: NavigationContext,
    message: str,
    goal_xy: np.ndarray | None,
    reference_length_m: float | None,
    include_planner_trace: bool = False,
) -> dict[str, object]:
    result: dict[str, object] = {
        "episode_index": context.episode_index,
        "context_start": context.context_start,
        "context_last_frame": context.context_last_frame,
        "goal_frame": context.goal_frame,
        "scene_path": context.scene_path,
        "goal_xy": None if goal_xy is None else np.asarray(goal_xy, dtype=np.float32).tolist(),
        "initial_distance_m": context.initial_distance_m,
        "initial_clearance_m": context.initial_clearance_m,
        "termination_reason": "runtime_error",
        "collision": False,
        "steps": 0,
        "final_distance_m": None,
        "min_clearance_m": None,
        "reference_length_m": reference_length_m,
        "path_length_m": None,
        "trajectory_xy": [],
        "runtime_error": message,
    }
    if include_planner_trace:
        result["planner_trace"] = []
    return result


def run_navigation_evaluation(
    checkpoint_path: Path,
    data_root: Path,
    output_path: Path,
    scene_name: str | None,
    num_eval: int,
    goal_offset: int,
    goal_tolerance_m: float,
    max_mpc_steps: int,
    horizon: int,
    samples: int,
    elites: int,
    iterations: int,
    safety_weight: float,
    policy: str,
    seed: int,
    device_name: str,
    context_manifest_path: Path | None = None,
    min_initial_goal_distance_m: float = 0.0,
    require_initial_clearance: bool = False,
    min_initial_clearance_m: float = 0.0,
    save_planner_diagnostics: bool = False,
    planner_mode: str = "pose_bev",
    smoothness_weight: float = 0.0,
    safety_threshold: float = float("inf"),
    bev_resolution_m: float = 0.1,
    unknown_weight: float = 1.0,
    allow_target_image_diagnostic: bool = False,
) -> dict[str, object]:
    """Run real Isaac Sim PointGoal rollouts and write success-based metrics."""

    global _ACTIVE_RUNTIME
    _ACTIVE_RUNTIME = None
    if min(horizon, samples, elites, iterations) < 1:
        raise ValueError("horizon, samples, elites, and iterations must be positive")
    action_protocol = load_action_protocol(_CONTRACT_PATH)
    torch.manual_seed(seed)
    device = torch.device(device_name if not device_name.startswith("cuda") or torch.cuda.is_available() else "cpu")
    if __package__ in (None, ""):
        from training.joint_jepa import JointModelConfig, build_joint_model
        from training.planner import plan
    else:
        from .joint_jepa import JointModelConfig, build_joint_model
        from .planner import plan

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if "config" not in checkpoint or "model" not in checkpoint:
        raise ValueError("checkpoint must contain config and model")
    require_checkpoint_protocol(checkpoint, action_protocol.protocol_id)
    checkpoint_velocity_frame(checkpoint)
    config = JointModelConfig(**checkpoint["config"])
    goal_source = resolve_planner_goal_source(
        policy=policy,
        planner_mode=planner_mode,
        pose_head_enabled=bool(config.pose_head_enabled),
        allow_target_image_diagnostic=allow_target_image_diagnostic,
    )
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"])
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
            "evaluation dataset action protocol does not match the active contract: "
            f"{dataset.action_protocol_id} != {action_protocol.protocol_id}"
        )
    contexts = resolve_navigation_contexts(
        dataset,
        scene_name,
        num_eval,
        goal_offset,
        seed,
        context_manifest_path,
        min_initial_goal_distance_m,
        require_initial_clearance,
        min_initial_clearance_m,
    )
    random_rng = np.random.default_rng(seed)
    episodes: list[dict[str, object]] = []
    runtime: IsaacNavigationRuntime | None = None
    runtime_scene: str | None = None

    for context in contexts:
            goal_xy: np.ndarray | None = None
            reference_length_m: float | None = None
            try:
                data = _load_episode(str(dataset.episode_paths[context.episode_index]))
                states = data["states"]
                if not isinstance(states, dict):
                    raise RuntimeError("episode states must be a mapping")
                goal_xy = _finite_vector(states["pos"][context.goal_state_index, :2], (2,), "goal_xy")
                reference_length_m = _recorded_reference_length(
                    states,
                    context.start_state_index,
                    context.goal_state_index,
                )
                initial_context = build_model_context(
                    _recorded_context_sample(dataset, context.episode_index, context.context_start),
                    config.history_size,
                )
                goal_emb: torch.Tensor | None = None
                if goal_source == "recorded_target_rgb_diagnostic":
                    goal_info = {
                        name: value.to(device)
                        for name, value in _recorded_goal_info(
                            dataset, context.episode_index, context.goal_frame
                        ).items()
                    }
                    with torch.no_grad():
                        goal_emb = model.encode(goal_info)["emb"].detach()
                if runtime is None:
                    metadata = data["metadata"]
                    if not isinstance(metadata, dict):
                        raise RuntimeError("episode metadata must be a mapping")
                    runtime = IsaacNavigationRuntime(
                        scene_path=context.scene_path,
                        image_size=config.image_size,
                        ball_radius_m=float(metadata.get("ball_radius_m", 0.4)),
                        device=str(device),
                        proprio_velocity_frame=config.proprio_velocity_frame,
                    )
                    runtime_scene = context.scene_path
                elif runtime_scene != context.scene_path:
                    raise RuntimeError("selected contexts span multiple scenes; rerun with --scene")
                initial_observation = runtime.reset_to_context(_state_row(states, context.start_state_index))

                def select_action(
                    current_context: Mapping[str, torch.Tensor],
                    current_observation: RuntimeObservation,
                    current_goal_xy: np.ndarray,
                ) -> np.ndarray | PlannerAction:
                    if policy == "zero":
                        return np.zeros(action_protocol.action_block_dim, dtype=np.float32)
                    if policy == "random":
                        return sample_random_action_block(random_rng, action_protocol)
                    goal_relative_xy: torch.Tensor | None = None
                    if planner_mode == "pose_bev":
                        goal_relative_xy = torch.from_numpy(
                            pointgoal_in_base(
                                current_observation.position_xy,
                                current_observation.yaw,
                                current_goal_xy,
                            )
                        ).to(device).reshape(1, 2)
                    with torch.no_grad():
                        planned = plan(
                            model,
                            dict(current_context),
                            action_protocol=action_protocol,
                            goal_emb=goal_emb,
                            horizon=horizon,
                            samples=samples,
                            elites=elites,
                            iterations=iterations,
                            safety_weight=safety_weight,
                            history_size=config.history_size,
                            device=device,
                            planner_mode=planner_mode,
                            goal_relative_xy=goal_relative_xy,
                            smoothness_weight=smoothness_weight,
                            safety_threshold=safety_threshold,
                            bev_resolution_m=bev_resolution_m,
                            robot_radius_m=runtime.ball_radius_m,
                            unknown_weight=unknown_weight,
                        )
                    action_block = planned.actions[0].detach().cpu().numpy().astype(np.float32)
                    if not save_planner_diagnostics:
                        return action_block
                    elite_costs = planned.elite_costs.detach().cpu()
                    diagnostics = {
                        "total_cost": float(planned.cost),
                        "goal_cost": float(planned.goal_cost),
                        "safety_cost": float(planned.safety_cost),
                        "smoothness_cost": float(planned.smoothness_cost),
                        "fallback_used": float(planned.fallback_used),
                        "elite_cost_min": float(elite_costs.min()),
                        "elite_cost_mean": float(elite_costs.mean()),
                        "elite_cost_std": float(elite_costs.std(unbiased=False)),
                    }
                    if planner_mode == "latent":
                        diagnostics.update(
                            {
                                "latent_goal_cost": float(planned.goal_cost),
                                "bev_safety_cost": float(planned.safety_cost),
                            }
                        )
                    else:
                        diagnostics.update(
                            {
                                "relative_pose_goal_cost": float(planned.goal_cost),
                                "local_bev_safety_cost": float(planned.safety_cost),
                            }
                        )
                    return PlannerAction(
                        action_block=action_block,
                        diagnostics=diagnostics,
                        goal_relative_xy=(
                            None if goal_relative_xy is None else goal_relative_xy[0].detach().cpu().numpy()
                        ),
                        predicted_relative_trajectory=(
                            None
                            if planned.predicted_relative_trajectory is None
                            else planned.predicted_relative_trajectory.detach().cpu().numpy()
                        ),
                    )

                episode = _run_closed_loop_episode(
                    runtime=runtime,
                    context=initial_context,
                    initial_position_xy=initial_observation.position_xy,
                    goal_xy=goal_xy,
                    action_selector=select_action,
                    max_mpc_steps=max_mpc_steps,
                    goal_tolerance_m=goal_tolerance_m,
                    reference_length_m=reference_length_m,
                    initial_collision=initial_observation.collision,
                    initial_clearance_m=initial_observation.clearance_m,
                    collect_planner_trace=save_planner_diagnostics,
                    initial_observation=initial_observation,
                )
                episode.update(
                    {
                        "episode_index": context.episode_index,
                        "context_start": context.context_start,
                        "context_last_frame": context.context_last_frame,
                        "goal_frame": context.goal_frame,
                        "scene_path": context.scene_path,
                        "goal_xy": goal_xy.tolist(),
                        "initial_distance_m": context.initial_distance_m,
                        "initial_clearance_m": context.initial_clearance_m,
                        "runtime_error": None,
                    }
                )
                episodes.append(episode)
            except Exception as exc:
                episodes.append(
                    _runtime_error_episode(
                        context,
                        f"{type(exc).__name__}: {exc}",
                        goal_xy,
                        reference_length_m,
                        include_planner_trace=save_planner_diagnostics,
                    )
                )

    _ACTIVE_RUNTIME = runtime

    summary = summarize_navigation(episodes, goal_tolerance_m, max_mpc_steps)
    result: dict[str, object] = {
        **summary,
        "collision_source": "analytic_bev_clearance",
        "policy": policy,
        "checkpoint": str(checkpoint_path),
        "data_root": str(data_root),
        "scene": scene_name,
        "goal_offset": int(goal_offset),
        "seed": int(seed),
        "device": str(device),
        "protocol": {
            "action_protocol_id": action_protocol.protocol_id,
            "controls_per_block": action_protocol.controls_per_block,
            "context_manifest": None if context_manifest_path is None else str(context_manifest_path),
            "min_initial_goal_distance_m": float(min_initial_goal_distance_m),
            "require_initial_clearance": bool(require_initial_clearance),
            "min_initial_clearance_m": float(min_initial_clearance_m),
            "planner_mode": planner_mode,
            "goal_source": goal_source,
            "smoothness_weight": float(smoothness_weight),
            "safety_threshold": float(safety_threshold),
            "bev_resolution_m": float(bev_resolution_m),
            "unknown_weight": float(unknown_weight),
        },
        "episodes": episodes,
    }
    if save_planner_diagnostics:
        plot_path = output_path.with_name(f"{output_path.stem}_planner_diagnostics.png")
        successful_episodes = [
            episode
            for episode in episodes
            if episode.get("runtime_error") is None and episode.get("trajectory_xy") and episode.get("goal_xy")
        ]
        result["planner_diagnostics"] = {
            "schema_version": "pointgoal_planner_trace_v1",
            "trajectory_plot": (
                str(
                    render_diagnostic_trajectory_plot(
                        plot_path,
                        successful_episodes,
                        title=f"{policy.upper()} PointGoal diagnostics — {scene_name or 'all scenes'}",
                    )
                )
                if successful_episodes
                else None
            ),
        }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(_json_compatible(result), indent=2, allow_nan=False))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Isaac Sim closed-loop PointGoal success evaluation")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scene", type=str, default=None)
    parser.add_argument("--num_eval", type=int, default=5)
    parser.add_argument("--goal_offset", type=int, default=5)
    parser.add_argument("--context_manifest", type=Path, default=None)
    parser.add_argument("--min_initial_goal_distance", type=float, default=0.0)
    parser.add_argument("--min_initial_clearance", type=float, default=0.0)
    parser.add_argument("--require_initial_clearance", action="store_true")
    parser.add_argument("--goal_tolerance", type=float, default=0.2)
    parser.add_argument("--max_mpc_steps", type=int, default=50)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--elites", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--safety_weight", type=float, default=0.1)
    parser.add_argument("--planner_mode", choices=("latent", "pose_bev"), default="pose_bev")
    parser.add_argument("--smoothness_weight", type=float, default=0.0)
    parser.add_argument("--safety_threshold", type=float, default=float("inf"))
    parser.add_argument("--bev_resolution_m", type=float, default=0.1)
    parser.add_argument("--unknown_weight", type=float, default=1.0)
    parser.add_argument("--allow_target_image_diagnostic", action="store_true")
    parser.add_argument("--policy", choices=("cem", "random", "zero"), default="cem")
    parser.add_argument("--save_planner_diagnostics", action="store_true")
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args(argv)
    result = run_navigation_evaluation(
        checkpoint_path=args.checkpoint,
        data_root=args.data_root,
        output_path=args.out,
        scene_name=args.scene,
        num_eval=args.num_eval,
        goal_offset=args.goal_offset,
        goal_tolerance_m=args.goal_tolerance,
        max_mpc_steps=args.max_mpc_steps,
        horizon=args.horizon,
        samples=args.samples,
        elites=args.elites,
        iterations=args.iterations,
        safety_weight=args.safety_weight,
        policy=args.policy,
        seed=args.seed,
        device_name=args.device,
        context_manifest_path=args.context_manifest,
        min_initial_goal_distance_m=args.min_initial_goal_distance,
        min_initial_clearance_m=args.min_initial_clearance,
        require_initial_clearance=args.require_initial_clearance,
        save_planner_diagnostics=args.save_planner_diagnostics,
        planner_mode=args.planner_mode,
        smoothness_weight=args.smoothness_weight,
        safety_threshold=args.safety_threshold,
        bev_resolution_m=args.bev_resolution_m,
        unknown_weight=args.unknown_weight,
        allow_target_image_diagnostic=args.allow_target_image_diagnostic,
    )
    print(json.dumps(_json_compatible(result), indent=2, allow_nan=False), flush=True)
    exit_code = evaluation_exit_code(result)
    if _ACTIVE_RUNTIME is not None:
        _ACTIVE_RUNTIME.close(return_code=exit_code)
    return exit_code


def advance_model_context(
    context: Mapping[str, torch.Tensor],
    executed_action_block: np.ndarray,
    observation: RuntimeObservation,
) -> dict[str, torch.Tensor]:
    """Roll a model context forward using only the executed action and Isaac data."""

    required = ("pixels_left", "pixels_right", "action", "proprio")
    if any(name not in context for name in required):
        raise ValueError("context is missing a model input tensor")
    history_size = int(context["action"].size(1))
    if history_size < 2:
        raise ValueError("closed-loop context requires at least two frames")
    action = torch.as_tensor(
        action_block_to_controls(executed_action_block).reshape(-1),
        dtype=context["action"].dtype,
        device=context["action"].device,
    ).reshape(1, 1, -1)
    zero_action = torch.zeros_like(action)
    updated = {
        "pixels_left": torch.cat(
            [context["pixels_left"][:, 1:], observation.pixels_left.to(context["pixels_left"]).reshape(1, 1, *observation.pixels_left.shape)],
            dim=1,
        ),
        "pixels_right": torch.cat(
            [context["pixels_right"][:, 1:], observation.pixels_right.to(context["pixels_right"]).reshape(1, 1, *observation.pixels_right.shape)],
            dim=1,
        ),
        "proprio": torch.cat(
            [context["proprio"][:, 1:], observation.proprio.to(context["proprio"]).reshape(1, 1, -1)],
            dim=1,
        ),
        "action": torch.cat([context["action"][:, 1:-1], action, zero_action], dim=1),
    }
    if any(value.size(1) != history_size for value in updated.values()):
        raise RuntimeError("advanced model context has the wrong history length")
    return updated


def _run_closed_loop_episode(
    *,
    runtime: Any,
    context: Mapping[str, torch.Tensor],
    initial_position_xy: np.ndarray,
    goal_xy: np.ndarray,
    action_selector: Any,
    max_mpc_steps: int,
    goal_tolerance_m: float,
    reference_length_m: float,
    initial_collision: bool = False,
    initial_clearance_m: float = np.inf,
    collect_planner_trace: bool = False,
    initial_observation: RuntimeObservation | None = None,
) -> dict[str, object]:
    """Execute receding-horizon blocks until a real terminal condition occurs."""

    start_xy = _finite_vector(initial_position_xy, (2,), "initial_position_xy")
    target_xy = _finite_vector(goal_xy, (2,), "goal_xy")
    if max_mpc_steps < 1:
        raise ValueError("max_mpc_steps must be positive")
    if goal_tolerance_m <= 0.0:
        raise ValueError("goal_tolerance_m must be positive")
    points = [start_xy.copy()]
    current_context = dict(context)
    current_observation = initial_observation
    distance = float(np.linalg.norm(start_xy - target_xy))
    clearance = float(initial_clearance_m)
    terminal = episode_termination(
        distance,
        bool(initial_collision),
        step=0,
        max_steps=max_mpc_steps,
        goal_tolerance_m=goal_tolerance_m,
    )
    steps = 0
    planner_trace: list[dict[str, object]] = []
    while terminal is None:
        steps += 1
        if current_observation is None:
            selected = action_selector(current_context)
        else:
            selected = action_selector(current_context, current_observation, target_xy)
        if isinstance(selected, PlannerAction):
            action_block = np.asarray(selected.action_block, dtype=np.float32)
            diagnostics = {str(name): float(value) for name, value in selected.diagnostics.items()}
            if not all(math.isfinite(value) for value in diagnostics.values()):
                raise RuntimeError("planner diagnostics must be finite")
            goal_relative_xy = (
                None
                if selected.goal_relative_xy is None
                else _finite_vector(selected.goal_relative_xy, (2,), "planner goal_relative_xy")
            )
            predicted_relative_trajectory = None
            if selected.predicted_relative_trajectory is not None:
                predicted_relative_trajectory = np.asarray(selected.predicted_relative_trajectory, dtype=np.float32)
                if (
                    predicted_relative_trajectory.ndim != 2
                    or predicted_relative_trajectory.shape[0] < 1
                    or predicted_relative_trajectory.shape[1] != 4
                    or not np.isfinite(predicted_relative_trajectory).all()
                ):
                    raise RuntimeError("planner predicted_relative_trajectory must be finite with shape (T, 4)")
        else:
            action_block = np.asarray(selected, dtype=np.float32)
            diagnostics = None
            goal_relative_xy = None
            predicted_relative_trajectory = None
        previous_distance = distance
        observation = runtime.execute_action_block(action_block)
        points.append(_finite_vector(observation.position_xy, (2,), "observation.position_xy"))
        distance = float(np.linalg.norm(points[-1] - target_xy))
        clearance = min(clearance, float(observation.clearance_m))
        if collect_planner_trace:
            trace_entry: dict[str, object] = {
                "step": steps,
                "selected_action_block": action_block.tolist(),
                "actual_position_xy": points[-1].tolist(),
                "actual_distance_m": distance,
                "distance_delta_m": distance - previous_distance,
                "actual_clearance_m": float(observation.clearance_m),
                "actual_collision": bool(observation.collision),
            }
            if diagnostics is not None:
                trace_entry = {**diagnostics, **trace_entry}
            if goal_relative_xy is not None:
                trace_entry["goal_relative_xy"] = goal_relative_xy.tolist()
            if predicted_relative_trajectory is not None:
                trace_entry["predicted_relative_trajectory"] = predicted_relative_trajectory.tolist()
            planner_trace.append(trace_entry)
        terminal = episode_termination(
            distance,
            observation.collision,
            step=steps,
            max_steps=max_mpc_steps,
            goal_tolerance_m=goal_tolerance_m,
        )
        current_context = advance_model_context(current_context, action_block, observation)
        current_observation = observation
    trajectory = np.stack(points).astype(np.float32)
    result: dict[str, object] = {
        "termination_reason": terminal,
        "collision": terminal == "collision",
        "steps": steps,
        "final_distance_m": distance,
        "min_clearance_m": clearance,
        "reference_length_m": float(reference_length_m),
        "path_length_m": path_length(trajectory),
        "trajectory_xy": trajectory.tolist(),
    }
    if collect_planner_trace:
        result["planner_trace"] = planner_trace
    return result


if __name__ == "__main__":
    raise SystemExit(main())
