"""Formal Isaac Sim stereo, state, action, and BEV data collector.

The existing Dubins waypoint generator and pure-pursuit controller remain the
navigation policy.  This module owns simulation advancement explicitly: every
physics-loop iteration applies its action, advances the world exactly once,
then records the resulting state at the completed-step timestamp.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import replace
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import traceback
from typing import Any

import numpy as np

from action_smoothing import (
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    ActionSmoother,
    default_articulation_gains,
)
from bev_ground_truth import (
    BEVConfig,
    GlobalElevation,
    GlobalOccupancy,
    IsaacOccupancyProvider,
    VisibilityHistory,
    build_analytic_global_map,
    build_global_elevation,
    crop_global_elevation,
    crop_global_occupancy,
    depth_to_visible_mask,
    enable_camera_depth,
    is_spawn_free,
    obstacle_clearance,
    read_camera_depth,
)
from collection_schema import (
    CollectionContract,
    EpisodeBuffer,
    EpisodeWriter,
    RateSchedule,
    StateRecord,
    StereoRecord,
    collection_step,
    derive_stereo_extrinsics,
    load_contract,
)
from trajectory_map import TrajectoryRecord, render_trajectory_map


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTRACT = SCRIPT_DIR.parent / "configs" / "data_contract.yaml"
DEFAULT_SAVE_DIR = SCRIPT_DIR.parent / "data" / "trajectories_dubins"
ROBOT_USD = (
    SCRIPT_DIR.parents[2]
    / "project"
    / "rl_env_0.4"
    / "rl_env"
    / "rotunbot_wheel_sim_model_0.4"
    / "Rotunbot_wheel.usd"
)
ROBOT_ROOT = "/World/BallRobot"
BASE_ROOT = f"{ROBOT_ROOT}/base_link"
LEFT_CAMERA_PATH = f"{BASE_ROOT}/cam_left"
RIGHT_CAMERA_PATH = f"{BASE_ROOT}/cam_right"
ROBOT_START = np.array([0.0, 0.0, 0.45])
MAX_VELOCITY = 1.5
CAMERA_OPTICAL_TO_USD = np.diag([1.0, -1.0, -1.0, 1.0])
SIGN_PROBES = ("none", "main_positive", "main_negative", "aux_positive", "aux_negative")


@contextmanager
def _collection_runtime(simulation_app: Any, stderr: Any | None = None):
    """Report failures before Isaac cleanup disables logging and unloads plugins."""

    error_stream = sys.stderr if stderr is None else stderr
    failure: tuple[BaseException, Any] | None = None
    try:
        yield
    except BaseException as exc:
        failure = (exc, exc.__traceback__)
        print(
            f"Collection failed: {type(exc).__name__}: {exc}",
            file=error_stream,
            flush=True,
        )
        traceback.print_exception(type(exc), exc, exc.__traceback__, file=error_stream)
    finally:
        if failure is None:
            simulation_app.close()
        else:
            try:
                simulation_app.app.post_quit(return_code=1)
            except BaseException as status_exc:
                print(
                    "Could not request Kit failure status: "
                    f"{type(status_exc).__name__}: {status_exc}",
                    file=error_stream,
                    flush=True,
                )
            try:
                simulation_app.close()
            except BaseException as cleanup_exc:
                print(
                    "Simulation cleanup raised "
                    f"{type(cleanup_exc).__name__}: {cleanup_exc}; "
                    "preserving collection failure",
                    file=error_stream,
                    flush=True,
                )
            original, original_traceback = failure
            raise original.with_traceback(original_traceback)


def _reset_articulation_after_occupancy(
    world: Any,
    robot: Any,
    proportional_gains: np.ndarray,
    derivative_gains: np.ndarray,
) -> Any:
    """Rebuild Isaac tensor views invalidated by temporary OMap collision edits."""

    world.reset()
    controller = robot.get_articulation_controller()
    controller.set_gains(kps=proportional_gains, kds=derivative_gains)
    return controller


def _prime_stereo_cameras(
    world: Any,
    left_camera: Any,
    right_camera: Any,
    max_steps: int = 20,
) -> None:
    """Advance bounded pre-episode renders until both depth annotators are ready."""

    last_error: RuntimeError | None = None
    for _ in range(max_steps):
        world.step(render=True)
        try:
            read_camera_depth(left_camera)
            read_camera_depth(right_camera)
        except RuntimeError as exc:
            last_error = exc
            continue
        return
    detail = f": {last_error}" if last_error is not None else ""
    raise RuntimeError(
        f"stereo depth annotators were not ready after {max_steps} rendering warmup steps{detail}"
    )


def _warmup_physics_scene(world: Any, steps: int = 10) -> None:
    """Advance physics without rendering so OMap sees cooked collision geometry.

    Isaac's occupancy generator queries the PhysX scene directly.  Calling it
    immediately after ``World.reset()`` can run before colliders are registered
    with the simulation scene, producing an all-free map even for a valid scene.
    Bounded non-rendering steps initialize PhysX without touching the formal
    episode loop, which starts from a fresh reset afterwards.
    """

    if steps < 1:
        raise ValueError("physics warmup steps must be positive")
    for _ in range(steps):
        world.step(render=False)


def _audit_episode_in_subprocess(
    episode_path: Path,
    python_executable: str = sys.executable,
) -> None:
    """Audit in a clean interpreter, isolated from Kit's bundled Python modules."""

    completed = subprocess.run(
        [python_executable, str(SCRIPT_DIR / "audit_dataset.py"), str(episode_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        details = "\n".join(
            text for text in (completed.stdout.strip(), completed.stderr.strip()) if text
        )
        suffix = f"\n{details}" if details else ""
        raise RuntimeError(f"episode audit failed for {episode_path}{suffix}")


def _build_parser(contract: CollectionContract) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--scene", type=str, default="")
    parser.add_argument("--num_episodes", type=int, default=100)
    parser.add_argument("--episode_len", type=int, default=300)
    parser.add_argument("--num_waypoints", type=int, default=4)
    parser.add_argument("--img_w", type=int, default=contract.image_width_px)
    parser.add_argument("--img_h", type=int, default=contract.image_height_px)
    parser.add_argument("--save_dir", type=Path, default=DEFAULT_SAVE_DIR)
    parser.add_argument("--ball_radius", type=float, default=0.4)
    parser.add_argument(
        "--main_accel_rad_s2",
        type=float,
        default=DEFAULT_MAIN_ACCELERATION_RAD_S2,
        help="maximum main-joint command acceleration in radians per second squared",
    )
    parser.add_argument(
        "--aux_kp", type=float, default=DEFAULT_AUXILIARY_KP,
        help="auxiliary steering position gain",
    )
    parser.add_argument(
        "--aux_kd", type=float, default=DEFAULT_AUXILIARY_KD,
        help="auxiliary steering damping gain",
    )
    parser.add_argument(
        "--aux_rate_deg_s",
        type=float,
        default=math.degrees(DEFAULT_AUXILIARY_RATE_RAD_S),
        help="maximum auxiliary target rate in degrees per second",
    )
    parser.add_argument(
        "--min_turn_r",
        type=float,
        default=None,
        help="minimum turn radius; defaults to the active auxiliary-angle limit",
    )
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--physics_hz", type=int, default=contract.schedule.physics_hz)
    parser.add_argument("--control_hz", type=int, default=contract.schedule.control_hz)
    parser.add_argument("--stereo_hz", type=int, default=contract.schedule.stereo_hz)
    parser.add_argument("--bev_extent_m", type=float, default=contract.bev.extent_m)
    parser.add_argument("--bev_resolution_m", type=float, default=contract.bev.resolution_m)
    parser.add_argument("--bev_teacher", choices=("omap", "analytic"), default="analytic")
    parser.add_argument(
        "--elevation_teacher",
        choices=contract.elevation_teacher_modes,
        default=contract.elevation_teacher_modes[0],
    )
    parser.add_argument("--no_save_depth", action="store_true")
    parser.add_argument("--no_save_elevation", action="store_true")
    parser.add_argument("--save_preview", action="store_true")
    parser.add_argument("--audit_after_episode", action="store_true")
    parser.add_argument("--sign_probe", choices=SIGN_PROBES, default="none")
    parser.add_argument("--sign_probe_steps", type=int, default=100)
    return parser


def _parse_args(argv: list[str] | None) -> tuple[argparse.Namespace, CollectionContract]:
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    preliminary_args, _ = preliminary.parse_known_args(argv)
    contract = load_contract(preliminary_args.contract)
    parser = _build_parser(contract)
    args = parser.parse_args(argv)
    if args.min_turn_r is None:
        maximum_auxiliary = max(
            abs(contract.action_minimum[1]), abs(contract.action_maximum[1])
        )
        args.min_turn_r = args.ball_radius / math.tan(maximum_auxiliary)
    fixed_values = {
        "img_w": contract.image_width_px,
        "img_h": contract.image_height_px,
        "bev_extent_m": contract.bev.extent_m,
        "bev_resolution_m": contract.bev.resolution_m,
    }
    for name, expected in fixed_values.items():
        if getattr(args, name) != expected:
            parser.error(f"--{name} must equal the schema 1.1 contract value {expected}")
    try:
        schedule = RateSchedule(
            physics_hz=args.physics_hz,
            control_hz=args.control_hz,
            stereo_hz=args.stereo_hz,
            state_hz=contract.schedule.state_hz,
        )
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    if contract.action_names != (
        "main_joint_velocity_cmd",
        "auxiliary_joint_angle_cmd",
    ):
        parser.error(f"unsupported action fields: {contract.action_names}")
    if args.elevation_teacher not in contract.elevation_teacher_modes:
        parser.error(f"unsupported elevation teacher: {args.elevation_teacher}")
    for name in ("num_episodes", "episode_len", "img_w", "img_h", "sign_probe_steps"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name} must be positive")
    if args.num_waypoints < 0:
        parser.error("--num_waypoints must be non-negative")
    if args.ball_radius <= 0.0 or args.min_turn_r <= 0.0:
        parser.error("--ball_radius and --min_turn_r must be positive")
    if (
        not np.isfinite(
            [args.main_accel_rad_s2, args.aux_kp, args.aux_kd, args.aux_rate_deg_s]
        ).all()
        or min(args.main_accel_rad_s2, args.aux_kp, args.aux_kd, args.aux_rate_deg_s) <= 0.0
    ):
        parser.error("--main_accel_rad_s2, --aux_kp, --aux_kd, and --aux_rate_deg_s must be positive")
    if args.scene and not Path(args.scene).is_file():
        parser.error(f"scene does not exist: {args.scene}")
    return args, replace(contract, schedule=schedule)


def norm_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def dubins_path_length(
    x: float,
    y: float,
    theta: float,
    target_x: float,
    target_y: float,
    min_turn_radius: float = 1.386,
) -> tuple[float, list[tuple[float, float, float]]]:
    """Return the collector's existing CSC Dubins-length approximation."""

    delta_x, delta_y = target_x - x, target_y - y
    distance = math.hypot(delta_x, delta_y)
    if distance < 1e-6:
        return 0.0, [(x, y, theta)]
    target_theta = math.atan2(delta_y, delta_x)
    heading_delta = norm_angle(target_theta - norm_angle(theta))
    alpha = abs(heading_delta)
    if alpha < 1e-3:
        return distance / MAX_VELOCITY, [(x, y, theta), (target_x, target_y, theta)]
    curve_length = min_turn_radius * alpha
    straight_length = max(0.0, distance - 2.0 * min_turn_radius * (1.0 - math.cos(alpha)))
    total_distance = 2.0 * curve_length + straight_length
    total_time = total_distance / MAX_VELOCITY + 2.0 * curve_length / (MAX_VELOCITY * 0.3)
    return total_time, [(x, y, theta), (target_x, target_y, target_theta)]


def pure_pursuit_control(
    current_x: float,
    current_y: float,
    current_theta: float,
    path: list[tuple[float, float]],
    ball_radius: float,
    min_turn_radius: float,
) -> tuple[float, float, bool]:
    """Compute the existing main-velocity/auxiliary-angle controller action."""

    if not path:
        return 0.0, 0.0, False
    target = path[0]
    delta_x, delta_y = target[0] - current_x, target[1] - current_y
    distance = math.hypot(delta_x, delta_y)
    if distance < 0.15:
        path.pop(0)
        return 0.0, 0.0, len(path) == 0
    heading_error = norm_angle(math.atan2(delta_y, delta_x) - current_theta)
    velocity_command = max(0.2, min(MAX_VELOCITY, distance * 0.8))
    maximum_curvature = 1.0 / min_turn_radius
    curvature = 2.0 * math.sin(heading_error) / max(distance, 0.1)
    curvature = float(np.clip(curvature, -maximum_curvature * 0.7, maximum_curvature * 0.7))
    main_velocity = velocity_command / ball_radius
    auxiliary_position = math.atan(min(abs(curvature * ball_radius), maximum_curvature * ball_radius))
    if curvature < 0.0:
        auxiliary_position = -auxiliary_position
    maximum_steer = math.atan(maximum_curvature * ball_radius)
    return (
        float(np.clip(main_velocity, -2.0, 2.0)),
        float(np.clip(auxiliary_position, -maximum_steer, maximum_steer)),
        False,
    )


def generate_waypoints(count: int, center_range: float = 5.0) -> list[tuple[float, float]]:
    waypoints: list[tuple[float, float]] = []
    previous_x, previous_y = 0.0, 0.0
    for _ in range(count):
        angle = random.uniform(0.0, 2.0 * math.pi)
        distance = random.uniform(1.0, center_range)
        previous_x += distance * math.cos(angle)
        previous_y += distance * math.sin(angle)
        waypoints.append((previous_x, previous_y))
    return waypoints


def _yaw_from_wxyz(quaternion: Any) -> float:
    try:
        orientation = np.asarray(quaternion, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("base pose quaternion must be a finite vector with shape (4,)") from exc
    if orientation.shape != (4,) or not np.isfinite(orientation).all():
        raise RuntimeError("base pose quaternion must be a finite vector with shape (4,)")
    norm = float(np.linalg.norm(orientation))
    if not np.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise RuntimeError("base pose quaternion must have non-zero finite norm")
    w, x, y, z = orientation / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _assert_planar_elevation_pose(position: Any, quaternion: Any) -> float:
    """Validate a finite pose and return yaw for a gravity-aligned elevation crop.

    The elevation frame is horizontal and yaw-only.  A rolling spherical base may
    have arbitrary finite roll/pitch; those components are intentionally ignored
    when cropping the static world map.  Invalid pose inputs fail closed before
    any elevation labels are emitted.
    """

    try:
        base_position = np.asarray(position, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("elevation labels require a finite base position") from exc
    if base_position.shape != (3,) or not np.isfinite(base_position).all():
        raise RuntimeError("elevation labels require a finite base position")
    try:
        return _yaw_from_wxyz(quaternion)
    except RuntimeError as exc:
        raise RuntimeError("elevation labels require a finite base pose quaternion") from exc


def _probe_action(name: str, minimum: np.ndarray, maximum: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if name == "main_positive":
        action[0] = maximum[0]
    elif name == "main_negative":
        action[0] = minimum[0]
    elif name == "aux_positive":
        action[1] = maximum[1]
    elif name == "aux_negative":
        action[1] = minimum[1]
    return action


def _ground_only_occupancy(config: BEVConfig, waypoint_count: int) -> GlobalOccupancy:
    """Create a compatibility all-free map when no external scene is loaded."""

    half_span = max(32.0, waypoint_count * 5.0 + config.extent_m)
    side = int(math.ceil(2.0 * half_span / config.resolution_m))
    grid = np.full((side, side), config.free, dtype=np.uint8)
    origin = np.array(
        [-half_span + config.resolution_m / 2.0] * 2,
        dtype=np.float64,
    )
    return GlobalOccupancy(grid=grid, global_origin_xy=origin, resolution_m=config.resolution_m)


def get_cam_pose(stage: Any, prim_path: str) -> np.ndarray:
    """Read a prim's USD world transform without advancing simulation."""

    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"camera/base prim does not exist: {prim_path}")
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    return np.asarray(matrix).reshape(4, 4).T.astype(np.float32)


def _camera_optical_to_base(stage: Any, camera_path: str) -> np.ndarray:
    world_from_base = get_cam_pose(stage, BASE_ROOT).astype(np.float64)
    world_from_camera_usd = get_cam_pose(stage, camera_path).astype(np.float64)
    return np.linalg.inv(world_from_base) @ world_from_camera_usd @ CAMERA_OPTICAL_TO_USD


def _get_rgb(camera: Any) -> np.ndarray:
    """Read RGB from the current completed step without stepping."""

    frame = camera.get_current_frame()
    if not frame or frame.get("rgba") is None:
        raise RuntimeError(f"camera {camera.prim_path} has no rgba frame")
    rgba = np.asarray(frame["rgba"])
    if rgba.ndim != 3 or rgba.shape[2] < 3:
        raise RuntimeError(f"camera {camera.prim_path} rgba has invalid shape {rgba.shape}")
    return np.asarray(rgba[:, :, :3], dtype=np.uint8)


def _finite_depth_for_storage(depth: np.ndarray) -> np.ndarray:
    return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)


def _episode_metadata(
    args: argparse.Namespace,
    contract: CollectionContract,
    episode: int,
    waypoints: list[tuple[float, float]],
    left_intrinsics: np.ndarray,
    right_intrinsics: np.ndarray,
    left_optical_to_base: np.ndarray,
    right_optical_to_base: np.ndarray,
) -> dict[str, Any]:
    extrinsics = derive_stereo_extrinsics(left_optical_to_base, right_optical_to_base)
    return {
        "schema_version": contract.schema_version,
        "contract_path": str(args.contract.resolve()),
        "scene": str(Path(args.scene).resolve()) if args.scene else "ground_only",
        "episode_index": episode,
        "random_seed": episode * 1000,
        "rates_hz": {
            "physics": contract.schedule.physics_hz,
            "control": contract.schedule.control_hz,
            "state": contract.schedule.state_hz,
            "stereo": contract.schedule.stereo_hz,
        },
        "physics_dt_s": 1.0 / contract.schedule.physics_hz,
        "rendering_dt_s": 1.0 / contract.schedule.stereo_hz,
        "image_width_px": args.img_w,
        "image_height_px": args.img_h,
        "action_protocol_id": contract.action_protocol_id,
        "controls_per_block": contract.controls_per_block,
        "action_names": contract.action_names,
        "action_minimum": contract.action_minimum,
        "action_maximum": contract.action_maximum,
        "action_semantics": "slew_limited_target; recorded after completed physics step",
        "action_execution": {
            "main_acceleration_limit_rad_s2": getattr(
                args, "main_accel_rad_s2", DEFAULT_MAIN_ACCELERATION_RAD_S2
            ),
            "auxiliary_target_rate_limit_rad_s": math.radians(
                getattr(
                    args,
                    "aux_rate_deg_s",
                    math.degrees(DEFAULT_AUXILIARY_RATE_RAD_S),
                )
            ),
            "articulation_gains": {
                "main_kp": 0.0,
                "main_kd": 80.0,
                "auxiliary_kp": getattr(args, "aux_kp", DEFAULT_AUXILIARY_KP),
                "auxiliary_kd": getattr(args, "aux_kd", DEFAULT_AUXILIARY_KD),
            },
        },
        "bev_extent_m": contract.bev.extent_m,
        "bev_resolution_m": contract.bev.resolution_m,
        "bev_shape": contract.bev.shape,
        "bev_class_codes": {
            "free": contract.bev.free,
            "occupied": contract.bev.occupied,
            "unknown": contract.bev.unknown,
        },
        "bev_frame": "base_x_forward_y_left; image_up=+x; image_left=+y",
        "bev_teacher": args.bev_teacher,
        "elevation_enabled": not args.no_save_elevation,
        "elevation_teacher": args.elevation_teacher,
        "elevation_frame": "base_xy_yaw_with_world_gravity_height",
        "elevation_pose_policy": "yaw_only; body_roll_pitch_ignored",
        "elevation_shape": list(contract.bev.shape),
        "elevation_unit": "m",
        "elevation_height_reference": "z_world - z_base_world",
        "elevation_dtype": contract.elevation_dtype,
        "elevation_invalid_value": contract.elevation_invalid_value,
        "elevation_file_naming": {
            "height_min_bev": "height_min_bev_{stereo_frame_id:06d}.npy",
            "height_max_bev": "height_max_bev_{stereo_frame_id:06d}.npy",
            "height_valid_bev": "height_valid_bev_{stereo_frame_id:06d}.png",
        },
        "visual_history_frames": contract.visual_history_frames,
        "ball_radius_m": args.ball_radius,
        "collision_definition": "minimum_obstacle_distance_m <= 0",
        "minimum_obstacle_distance_definition": (
            "nearest occupied BEV cell-center distance minus ball_radius_m"
        ),
        "left_intrinsics": left_intrinsics,
        "right_intrinsics": right_intrinsics,
        "intrinsics_matrix_convention": contract.intrinsics_matrix_convention,
        "extrinsic_matrix_convention": contract.extrinsic_matrix_convention,
        "left_to_right_extrinsic": extrinsics.left_to_right_extrinsic,
        "base_to_left_extrinsic": extrinsics.base_to_left_extrinsic,
        "base_to_right_extrinsic": extrinsics.base_to_right_extrinsic,
        "left_optical_to_base_extrinsic": extrinsics.left_optical_to_base_extrinsic,
        "right_optical_to_base_extrinsic": extrinsics.right_optical_to_base_extrinsic,
        "camera_pose_storage": (
            "column vectors; camera USD local to world, "
            "p_world = T_camera_usd_to_world @ p_camera_usd"
        ),
        "depth_frame": "OpenCV optical x_right_y_down_z_forward",
        "depth_invalid_storage_value": 0.0,
        "waypoints_world_xy": waypoints,
        "sign_probe": args.sign_probe,
        "sign_probe_steps": args.sign_probe_steps if args.sign_probe != "none" else 0,
        "file_naming": {
            "left_rgb": "left_{stereo_frame_id:06d}.png",
            "right_rgb": "right_{stereo_frame_id:06d}.png",
            "left_depth": "left_depth_{stereo_frame_id:06d}.npy",
            "right_depth": "right_depth_{stereo_frame_id:06d}.npy",
            "height_min_bev": "height_min_bev_{stereo_frame_id:06d}.npy",
            "height_max_bev": "height_max_bev_{stereo_frame_id:06d}.npy",
            "height_valid_bev": "height_valid_bev_{stereo_frame_id:06d}.png",
            "geometry_bev": "geometry_bev_{stereo_frame_id:06d}.png",
            "current_visible": "current_visible_{stereo_frame_id:06d}.png",
            "history_observed": "history_observed_{stereo_frame_id:06d}.png",
            "preview": "preview_{stereo_frame_id:06d}.png",
        },
    }


def _load_collection_scene(
    world: Any,
    scene_path: str,
    add_reference_to_stage: Any,
    xform_prim: Any,
    print_fn: Any = print,
) -> str | None:
    """Add the shared floor collider, then optionally reference external clutter."""

    world.scene.add_default_ground_plane()
    if not scene_path:
        print_fn("Scene: ground-only compatibility mode")
        return None
    scene_root = "/World/Scene"
    add_reference_to_stage(usd_path=os.path.abspath(scene_path), prim_path=scene_root)
    xform_prim(scene_root).set_local_pose(translation=np.zeros(3))
    print_fn(f"Scene: {scene_path}")
    return scene_root


def _run_collection(args: argparse.Namespace, contract: CollectionContract) -> int:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
    with _collection_runtime(simulation_app):
        import carb
        import omni.usd
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.prims import XFormPrim
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
        from omni.isaac.sensor import Camera
        from pxr import Gf, UsdGeom, UsdLux

        args.save_dir.mkdir(parents=True, exist_ok=True)
        schedule = contract.schedule
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / schedule.physics_hz,
            rendering_dt=1.0 / schedule.stereo_hz,
        )
        scene_root = _load_collection_scene(
            world,
            args.scene,
            add_reference_to_stage,
            XFormPrim,
        )

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

        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        robot = world.scene.add(
            Articulation(prim_path=ROBOT_ROOT, name="ball_robot", position=ROBOT_START)
        )
        camera_x, camera_y, camera_z = 0.1, 0.30, 0.10
        focal_length, horizontal_aperture = 16.45, 18.0
        vertical_aperture = horizontal_aperture * args.img_h / args.img_w

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
        world.reset()

        proportional_gains, derivative_gains = default_articulation_gains()
        action_smoother = ActionSmoother(
            dt_s=1.0 / schedule.physics_hz,
            max_main_acceleration_rad_s2=args.main_accel_rad_s2,
            max_auxiliary_rate_rad_s=math.radians(args.aux_rate_deg_s),
        )

        left_camera = Camera(
            prim_path=LEFT_CAMERA_PATH,
            resolution=(args.img_w, args.img_h),
            frequency=schedule.stereo_hz,
        )
        right_camera = Camera(
            prim_path=RIGHT_CAMERA_PATH,
            resolution=(args.img_w, args.img_h),
            frequency=schedule.stereo_hz,
        )
        left_camera.initialize()
        right_camera.initialize()
        enable_camera_depth(left_camera)
        enable_camera_depth(right_camera)
        left_intrinsics = np.asarray(left_camera.get_intrinsics_matrix(), dtype=np.float64)
        right_intrinsics = np.asarray(right_camera.get_intrinsics_matrix(), dtype=np.float64)
        if left_intrinsics.shape != (3, 3) or right_intrinsics.shape != (3, 3):
            raise RuntimeError("Isaac Camera intrinsics must have shape (3, 3)")
        left_optical_to_base = _camera_optical_to_base(stage, LEFT_CAMERA_PATH)
        right_optical_to_base = _camera_optical_to_base(stage, RIGHT_CAMERA_PATH)

        # Cook the PhysX scene before OMap so collision geometry is registered;
        # the formal episode loop still starts from the reset below.
        _warmup_physics_scene(world, steps=10)

        if scene_root:
            if args.bev_teacher == "analytic":
                from pxr import Usd, UsdGeom, UsdPhysics

                occupancy = build_analytic_global_map(
                    stage,
                    scene_root,
                    ROBOT_ROOT,
                    contract.bev,
                    args.ball_radius,
                    Usd,
                    UsdGeom,
                    UsdPhysics,
                )
            else:
                occupancy = IsaacOccupancyProvider(
                    stage=stage,
                    scene_root=scene_root,
                    robot_root=ROBOT_ROOT,
                    config=contract.bev,
                    physics_sync=lambda: _warmup_physics_scene(world, steps=3),
                ).build_global_map()
        else:
            occupancy = _ground_only_occupancy(contract.bev, args.num_waypoints)

        save_elevation = not args.no_save_elevation
        elevation: GlobalElevation | None = None
        if save_elevation:
            if scene_root:
                from pxr import Usd, UsdPhysics

                elevation = build_global_elevation(
                    args.elevation_teacher,
                    stage,
                    scene_root,
                    ROBOT_ROOT,
                    occupancy,
                    contract.bev,
                    Usd,
                    UsdGeom,
                    UsdPhysics,
                    physics_sync=lambda: _warmup_physics_scene(world, steps=3),
                )
            else:
                elevation = GlobalElevation(
                    height_min=np.zeros(occupancy.grid.shape, dtype=np.float32),
                    height_max=np.zeros(occupancy.grid.shape, dtype=np.float32),
                    valid=np.ones(occupancy.grid.shape, dtype=np.uint8),
                    global_origin_xy=occupancy.global_origin_xy,
                    resolution_m=occupancy.resolution_m,
                )

        controller = _reset_articulation_after_occupancy(
            world,
            robot,
            proportional_gains,
            derivative_gains,
        )
        # Camera callbacks first establish timing and annotators initially expose
        # scalar placeholders. Prime them outside the formal episode loop, only
        # after OMap collision edits have been followed by a physics-view reset.
        _prime_stereo_cameras(world, left_camera, right_camera)

        action_minimum = np.asarray(contract.action_minimum, dtype=np.float32)
        action_maximum = np.asarray(contract.action_maximum, dtype=np.float32)
        maximum_curvature = 1.0 / args.min_turn_r
        maximum_steer = math.atan(maximum_curvature * args.ball_radius)
        print(f"Joints: {robot.dof_names} | ball radius={args.ball_radius:.3f} m")
        print(
            f"Rates physics/control/state/stereo: {schedule.physics_hz}/"
            f"{schedule.control_hz}/{schedule.state_hz}/{schedule.stereo_hz} Hz"
        )
        print(f"Maximum steer={math.degrees(maximum_steer):.1f} deg")

        zero_articulation_action = ArticulationAction(
            joint_velocities=np.array([0.0, np.nan, np.nan]),
            joint_positions=np.array([np.nan, 0.0, np.nan]),
        )

        def reset_robot(episode: int) -> tuple[float, float, float]:
            rng = np.random.default_rng(episode * 1000 + 7)
            start_x = start_y = 0.0
            found_free_spawn = False
            for _ in range(100):
                start_x = rng.uniform(-2.0, 2.0)
                start_y = rng.uniform(-2.0, 2.0)
                if is_spawn_free(
                    occupancy.grid,
                    occupancy.global_origin_xy,
                    np.array([start_x, start_y]),
                    args.ball_radius,
                    contract.bev,
                ):
                    found_free_spawn = True
                    break
            if not found_free_spawn:
                raise RuntimeError(
                    "no collision-free episode start found after 100 samples; "
                    "check scene occupancy and spawn bounds"
                )
            robot.set_world_pose(position=np.array([start_x, start_y, ROBOT_START[2]]))
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
            robot.set_joint_positions(np.zeros(robot.num_dof))
            robot.set_joint_velocities(np.zeros(robot.num_dof))
            controller.set_gains(kps=proportional_gains, kds=derivative_gains)
            action_smoother.reset()
            controller.apply_action(zero_articulation_action)
            world.step(render=False)  # Reset warmup; outside the formal physics loop.
            position, quaternion = robot.get_world_pose()
            return float(position[0]), float(position[1]), _yaw_from_wxyz(quaternion)

        episode_count = 1 if args.sign_probe != "none" else args.num_episodes
        total_states = 0
        scene_trajectories: list[TrajectoryRecord] = []
        for episode in range(episode_count):
            current_x, current_y, current_yaw = reset_robot(episode)
            probe_start = np.array([current_x, current_y, current_yaw], dtype=np.float64)
            waypoints = [] if args.sign_probe != "none" else generate_waypoints(args.num_waypoints)
            current_path = waypoints.copy()
            reached = 0
            episode_steps = args.sign_probe_steps if args.sign_probe != "none" else args.episode_len
            buffer = EpisodeBuffer()
            writer = EpisodeWriter(
                args.save_dir / f"ep{episode:04d}",
                metadata=_episode_metadata(
                    args,
                    contract,
                    episode,
                    waypoints,
                    left_intrinsics,
                    right_intrinsics,
                    left_optical_to_base,
                    right_optical_to_base,
                ),
                save_depth=not args.no_save_depth,
                save_preview=args.save_preview,
                save_elevation=save_elevation,
            )
            held_commanded = np.zeros(2, dtype=np.float32)
            held_applied = np.zeros(2, dtype=np.float32)
            episode_points = [[current_x, current_y]]
            episode_collision_points: list[list[float]] = []
            visibility_history = VisibilityHistory(
                max_frames=contract.visual_history_frames,
                config=contract.bev,
            )
            stereo_frame_id = 0

            for loop_index in range(episode_steps):
                decision = collection_step(loop_index, schedule)
                if decision.is_control:
                    if args.sign_probe != "none":
                        held_commanded = _probe_action(
                            args.sign_probe,
                            action_minimum,
                            action_maximum,
                        )
                    else:
                        main_command, auxiliary_command, done = pure_pursuit_control(
                            current_x,
                            current_y,
                            current_yaw,
                            current_path,
                            args.ball_radius,
                            args.min_turn_r,
                        )
                        held_commanded = np.array(
                            [main_command, auxiliary_command],
                            dtype=np.float32,
                        )
                        if done:
                            reached += 1
                            if reached < len(waypoints):
                                current_path = [waypoints[reached]]
                    clipped_target = np.clip(
                        held_commanded,
                        action_minimum,
                        action_maximum,
                    ).astype(np.float32)
                    held_applied = action_smoother.step(
                        clipped_target,
                        action_minimum,
                        action_maximum,
                    )
                    controller.apply_action(
                        ArticulationAction(
                            joint_velocities=np.array([held_applied[0], np.nan, np.nan]),
                            joint_positions=np.array([np.nan, held_applied[1], np.nan]),
                        )
                    )

                world.step(render=decision.is_stereo)

                position, quaternion = robot.get_world_pose()
                linear_velocity = robot.get_linear_velocity()
                angular_velocity = robot.get_angular_velocity()
                joint_positions = robot.get_joint_positions()
                joint_velocities = robot.get_joint_velocities()
                current_x, current_y = float(position[0]), float(position[1])
                current_yaw = _yaw_from_wxyz(quaternion)
                episode_points.append([current_x, current_y])
                buffer.append_state(
                    StateRecord(
                        physics_step=decision.physics_step,
                        sim_time=decision.sim_time,
                        state_timestamp=decision.sim_time,
                        pos=position,
                        quat=quaternion,
                        lin_vel=linear_velocity,
                        ang_vel=angular_velocity,
                        joint_pos=joint_positions,
                        joint_vel=joint_velocities,
                        commanded_action=held_commanded,
                        applied_action=held_applied,
                        action_command_timestamp=decision.sim_time,
                        action_applied_timestamp=decision.sim_time,
                    )
                )

                if decision.is_stereo:
                    left_rgb = _get_rgb(left_camera)
                    right_rgb = _get_rgb(right_camera)
                    left_depth = read_camera_depth(left_camera)
                    right_depth = read_camera_depth(right_camera)
                    base_pose = np.array([current_x, current_y, current_yaw], dtype=np.float64)
                    geometry_bev = crop_global_occupancy(
                        occupancy.grid,
                        occupancy.global_origin_xy,
                        position,
                        current_yaw,
                        contract.bev,
                    )
                    left_visible = depth_to_visible_mask(
                        left_depth,
                        left_intrinsics,
                        left_optical_to_base,
                        contract.bev,
                    )
                    right_visible = depth_to_visible_mask(
                        right_depth,
                        right_intrinsics,
                        right_optical_to_base,
                        contract.bev,
                    )
                    current_visible = np.logical_or(left_visible, right_visible).astype(np.uint8)
                    history_observed = visibility_history.update(current_visible, base_pose)
                    clearance = obstacle_clearance(
                        geometry_bev,
                        args.ball_radius,
                        contract.bev,
                    )
                    if elevation is None:
                        height_min_bev = height_max_bev = height_valid_bev = None
                    else:
                        current_yaw = _assert_planar_elevation_pose(position, quaternion)
                        height_min_bev, height_max_bev, height_valid_bev = crop_global_elevation(
                            elevation,
                            position,
                            current_yaw,
                            contract.bev,
                        )
                    writer.write_visual_sample(
                        frame_id=stereo_frame_id,
                        left_rgb=left_rgb,
                        right_rgb=right_rgb,
                        left_depth=_finite_depth_for_storage(left_depth),
                        right_depth=_finite_depth_for_storage(right_depth),
                        geometry_bev=geometry_bev,
                        current_visible=current_visible,
                        history_observed=history_observed,
                        height_min_bev=height_min_bev,
                        height_max_bev=height_max_bev,
                        height_valid_bev=height_valid_bev,
                    )
                    if clearance <= 0.0:
                        episode_collision_points.append([current_x, current_y])
                    buffer.append_stereo(
                        StereoRecord(
                            frame_id=stereo_frame_id,
                            physics_step=decision.physics_step,
                            left_timestamp=decision.sim_time,
                            right_timestamp=decision.sim_time,
                            cam_left_pose=get_cam_pose(stage, LEFT_CAMERA_PATH),
                            cam_right_pose=get_cam_pose(stage, RIGHT_CAMERA_PATH),
                            left_filename=f"left_{stereo_frame_id:06d}.png",
                            right_filename=f"right_{stereo_frame_id:06d}.png",
                            collision=clearance <= 0.0,
                            minimum_obstacle_distance=clearance,
                        )
                    )
                    stereo_frame_id += 1

            output_path = writer.finalize(buffer)
            if args.audit_after_episode:
                _audit_episode_in_subprocess(output_path)
                print(f"Audit passed: {output_path}")
            scene_trajectories.append(
                TrajectoryRecord(
                    label=f"ep{episode:04d}",
                    points_xy=np.asarray(episode_points, dtype=np.float64),
                    collision_points_xy=np.asarray(
                        episode_collision_points,
                        dtype=np.float64,
                    ).reshape((-1, 2)),
                )
            )
            total_states += episode_steps
            print(
                f"Episode {episode + 1}/{episode_count}: waypoints={len(waypoints)} "
                f"reached={reached} states={episode_steps} stereo={stereo_frame_id}"
            )
            if args.sign_probe != "none":
                translation = np.array([current_x, current_y]) - probe_start[:2]
                yaw_delta = norm_angle(current_yaw - probe_start[2])
                print(
                    f"Sign probe {args.sign_probe}: translation_xy_m="
                    f"[{translation[0]:.6f}, {translation[1]:.6f}], "
                    f"distance_m={np.linalg.norm(translation):.6f}, yaw_rad={yaw_delta:.6f}"
                )

        scene_name = Path(args.scene).stem if args.scene else "ground_only"
        trajectory_path = render_trajectory_map(
            args.save_dir / "trajectory_map.png",
            occupancy,
            scene_trajectories,
            title=f"Robot trajectory — {scene_name}",
        )
        print(f"Trajectory map: {trajectory_path}")
        print(f"Collection complete: {total_states} state samples in {args.save_dir}")
        return 0


def main(argv: list[str] | None = None) -> int:
    args, contract = _parse_args(argv)
    return _run_collection(args, contract)


if __name__ == "__main__":
    raise SystemExit(main())
