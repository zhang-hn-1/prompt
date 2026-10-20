#!/usr/bin/env python3
"""Run auditable Ball PointGoal Benchmark V1 episodes in Isaac Sim."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from typing import Any, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.artifacts import (  # noqa: E402
    REQUIRED_TRAJECTORY_LAYERS,
    AuditBundleWriter,
    write_rgb_video,
)
from benchmark.isaac_runtime import (  # noqa: E402
    DEFAULT_DRIVE_PROFILE,
    IsaacBenchmarkRuntime,
    IsaacRuntimeConfig,
)
from benchmark.manifest import BenchmarkEpisode, load_manifest  # noqa: E402
from benchmark.policies import (  # noqa: E402
    AngleAwareConstraintOraclePolicy,
    ConstraintAwareTrajectoryOraclePolicy,
    DynamicOraclePolicy,
    GoalGreedyPolicy,
    OraclePathPolicy,
    RandomPolicy,
    StopPolicy,
)
from benchmark.world_model_policy import WorldModelPolicy  # noqa: E402
from benchmark.runner import RunnerConfig, run_episode  # noqa: E402
from scripts.trajectory_map import (  # noqa: E402
    NavigationTaskOverlay,
    TrajectoryRecord,
    render_navigation_task_map,
    render_trajectory_map,
)
from training.action_protocol import load_action_protocol  # noqa: E402
from training.navigation_metrics import (  # noqa: E402
    cluster_bootstrap_interval,
    summarize_navigation,
)


DEFAULT_CONTRACT = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"
DEFAULT_SCENE_INDEX = (
    Path(__file__).resolve().parents[1] / "configs" / "benchmark_v1" / "scene_index.json"
)
DEFAULT_ORACLE_LOOKAHEAD_M = 0.5
DEFAULT_CONSTRAINT_POLICY_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "oracle_constraint_v1.json"
)
DEFAULT_INTERVENTION_POLICY_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "oracle_intervention_v1.json"
)


def _validate_oracle_lookahead_m(value: object) -> float:
    if isinstance(value, bool):
        raise ValueError("oracle_lookahead_m must be finite and positive")
    try:
        lookahead_m = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("oracle_lookahead_m must be finite and positive") from error
    if not math.isfinite(lookahead_m) or lookahead_m <= 0.0:
        raise ValueError("oracle_lookahead_m must be finite and positive")
    return lookahead_m


def _parse_oracle_lookahead_m(value: str) -> float:
    try:
        return _validate_oracle_lookahead_m(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _parse_auxiliary_velocity_limit_rad_s(value: str) -> float:
    try:
        limit = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise argparse.ArgumentTypeError(
            "auxiliary_velocity_limit_rad_s must be finite and positive"
        ) from error
    if not math.isfinite(limit) or limit <= 0.0:
        raise argparse.ArgumentTypeError(
            "auxiliary_velocity_limit_rad_s must be finite and positive"
        )
    return limit


def _load_constraint_policy_config(path: Path | None) -> tuple[dict[str, object], str, Path]:
    selected = DEFAULT_CONSTRAINT_POLICY_CONFIG if path is None else Path(path)
    resolved = selected.expanduser().resolve()
    raw = resolved.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"constraint policy config is not valid JSON: {resolved}") from error
    if not isinstance(payload, dict):
        raise ValueError("constraint policy config must be a JSON object")
    if payload.get("schema_version") != "oracle_constraint_v1":
        raise ValueError("constraint policy config schema_version must be oracle_constraint_v1")
    if payload.get("policy_name") != "oracle_constraint_v1":
        raise ValueError("constraint policy config policy_name must be oracle_constraint_v1")
    allowed = {
        "auxiliary_velocity_limit_rad_s",
        "auxiliary_saturation_threshold_rad_s",
        "auxiliary_release_threshold_rad_s",
        "scheduled_main_command_rad_s",
        "maximum_main_command_rad_s",
        "turning_curvature_threshold_1_m",
        "lateral_error_threshold_m",
        "heading_error_threshold_rad",
    }
    unknown = sorted(set(payload) - {"schema_version", "policy_name", *allowed})
    if unknown:
        raise ValueError(f"constraint policy config contains unknown keys: {unknown}")
    return {key: payload[key] for key in allowed if key in payload}, digest, resolved


def _load_intervention_policy_config(path: Path | None) -> tuple[dict[str, object], str, Path]:
    selected = DEFAULT_INTERVENTION_POLICY_CONFIG if path is None else Path(path)
    resolved = selected.expanduser().resolve()
    raw = resolved.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"intervention policy config is not valid JSON: {resolved}") from error
    if not isinstance(payload, dict):
        raise ValueError("intervention policy config must be a JSON object")
    if payload.get("schema_version") != "oracle_intervention_v1":
        raise ValueError("intervention policy config schema_version must be oracle_intervention_v1")
    if payload.get("policy_name") != "oracle_intervention_v1":
        raise ValueError("intervention policy config policy_name must be oracle_intervention_v1")
    allowed = {
        "auxiliary_velocity_limit_rad_s",
        "auxiliary_saturation_threshold_rad_s",
        "auxiliary_release_threshold_rad_s",
        "scheduled_main_command_rad_s",
        "maximum_main_command_rad_s",
        "turning_curvature_threshold_1_m",
        "lateral_error_threshold_m",
        "heading_error_threshold_rad",
        "auxiliary_angle_saturation_threshold_rad",
        "auxiliary_angle_release_threshold_rad",
        "angle_scheduled_main_command_rad_s",
    }
    unknown = sorted(set(payload) - {"schema_version", "policy_name", *allowed})
    if unknown:
        raise ValueError(f"intervention policy config contains unknown keys: {unknown}")
    return {key: payload[key] for key in allowed if key in payload}, digest, resolved


def _evaluation_protocol_id(args: argparse.Namespace) -> str:
    mode = getattr(args, "collision_mode", "map_clearance_v1")
    return {
        "map_clearance_v1": "ball_pointgoal_v1_1_35s",
        "physical_contact_v2": "ball_pointgoal_v1_2_contact_35s",
    }[mode]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--collision-mode", choices=("physical_contact_v2", "map_clearance_v1"),
        default="physical_contact_v2",
        help="Physical obstacle contact (default), or explicit historical map-clearance scoring.",
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--split", required=True, choices=("golden", "dev", "test"))
    parser.add_argument(
        "--policy",
        required=True,
        choices=(
            "stop",
            "random",
            "goal_greedy",
            "oracle",
            "oracle_v2",
            "oracle_constraint_v1",
            "oracle_intervention_v1",
            "world_model",
        ),
    )
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--gui", action="store_true")
    mode.add_argument("--headless", action="store_true")
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument("--episode-id")
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--scene-index", type=Path, default=DEFAULT_SCENE_INDEX)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument(
        "--drive-profile",
        type=Path,
        default=DEFAULT_DRIVE_PROFILE,
        help="drive dynamics profile used by the Oracle and runtime",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="world-model checkpoint; required with --policy world_model",
    )
    parser.add_argument(
        "--device",
        default="cuda",
        help="torch device for --policy world_model (default: cuda)",
    )
    parser.add_argument("--wm-horizon", type=int, default=5)
    parser.add_argument("--wm-samples", type=int, default=64)
    parser.add_argument("--wm-elites", type=int, default=8)
    parser.add_argument("--wm-iterations", type=int, default=3)
    parser.add_argument("--wm-safety-weight", type=float, default=0.0)
    parser.add_argument(
        "--allow-legacy-checkpoint",
        action="store_true",
        help="explicitly allow a legacy aux16 checkpoint for diagnostic adapter runs",
    )
    parser.add_argument(
        "--oracle-lookahead-m",
        type=_parse_oracle_lookahead_m,
        default=DEFAULT_ORACLE_LOOKAHEAD_M,
        help=(
            "V1 oracle path lookahead in metres (diagnostic override; only valid "
            "with --policy oracle; default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--auxiliary-velocity-limit-rad-s",
        type=_parse_auxiliary_velocity_limit_rad_s,
        default=None,
        help=(
            "formal auxiliary joint velocity supervisor in rad/s; omitted keeps "
            "the tracked baseline controller"
        ),
    )
    parser.add_argument(
        "--policy-config",
        type=Path,
        default=None,
        help=(
            "versioned policy JSON; used by the constraint/intervention Oracle policies "
            "(defaults to its checked-in config)"
        ),
    )
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    return parser


def _make_policy(
    name: str,
    protocol: Any,
    seed: int,
    *,
    oracle_lookahead_m: float = DEFAULT_ORACLE_LOOKAHEAD_M,
    checkpoint: Path | None = None,
    device: str = "cuda",
    wm_horizon: int = 5,
    wm_samples: int = 64,
    wm_elites: int = 8,
    wm_iterations: int = 3,
    wm_safety_weight: float = 0.0,
    allow_legacy_checkpoint: bool = False,
    policy_config: Path | None = None,
) -> Any:
    lookahead_m = _validate_oracle_lookahead_m(oracle_lookahead_m)
    if name != "oracle" and lookahead_m != DEFAULT_ORACLE_LOOKAHEAD_M:
        raise ValueError("--oracle-lookahead-m is only supported with --policy oracle")
    if name == "stop":
        return StopPolicy(protocol)
    if name == "random":
        return RandomPolicy(protocol, seed=seed)
    if name == "goal_greedy":
        return GoalGreedyPolicy(protocol)
    if name == "oracle":
        return OraclePathPolicy(protocol, lookahead_m=lookahead_m)
    if name == "oracle_v2":
        return DynamicOraclePolicy(protocol)
    if name == "oracle_constraint_v1":
        config, _, _ = _load_constraint_policy_config(policy_config)
        return ConstraintAwareTrajectoryOraclePolicy(protocol, **config)
    if name == "oracle_intervention_v1":
        config, _, _ = _load_intervention_policy_config(policy_config)
        return AngleAwareConstraintOraclePolicy(protocol, **config)
    if name == "world_model":
        if checkpoint is None:
            raise ValueError("--checkpoint is required with --policy world_model")
        return WorldModelPolicy(
            protocol,
            checkpoint=checkpoint,
            device=device,
            horizon=wm_horizon,
            samples=wm_samples,
            elites=wm_elites,
            iterations=wm_iterations,
            safety_weight=wm_safety_weight,
            allow_legacy_checkpoint=allow_legacy_checkpoint,
        )
    raise ValueError(f"unsupported policy: {name!r}")


def _selected_oracle_lookahead_m(args: argparse.Namespace) -> float:
    return _validate_oracle_lookahead_m(
        getattr(args, "oracle_lookahead_m", DEFAULT_ORACLE_LOOKAHEAD_M)
    )


def _resolve_occupancy_path(
    scene_index_path: Path,
    episode: BenchmarkEpisode,
    data_root: Path | None,
) -> Path:
    with scene_index_path.open("r", encoding="utf-8") as stream:
        index = json.load(stream)
    scenes = index.get("scenes") if isinstance(index, dict) else None
    if not isinstance(scenes, list):
        raise ValueError("scene index must contain a scenes array")
    matches = [
        scene for scene in scenes
        if isinstance(scene, dict) and scene.get("scene_uri") == episode.scene_uri
    ]
    if len(matches) != 1:
        raise ValueError(
            f"scene index must contain exactly one record for {episode.scene_uri}"
        )
    uri = matches[0].get("occupancy_uri")
    if not isinstance(uri, str) or not uri:
        raise ValueError("scene-index record requires occupancy_uri")
    prefix = "${DATA_ROOT}/"
    if uri.startswith(prefix):
        root = data_root
        if root is None:
            environment_root = os.environ.get("DATA_ROOT")
            root = Path(environment_root) if environment_root else scene_index_path.parent
        root = root.expanduser().resolve()
        relative = Path(uri[len(prefix) :])
        if relative.is_absolute() or not relative.parts or ".." in relative.parts:
            raise ValueError("occupancy_uri escapes DATA_ROOT")
        resolved = root.joinpath(relative).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as error:
            raise ValueError("occupancy_uri escapes DATA_ROOT") from error
    else:
        path = Path(uri).expanduser()
        resolved = path.resolve() if path.is_absolute() else (
            scene_index_path.parent / path
        ).resolve()
    return episode.verify_occupancy(resolved)


def _yaw_from_wxyz(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _world_to_cell(world_xy: Sequence[float], occupancy: Any) -> tuple[int, int]:
    continuous = (
        np.asarray(world_xy, dtype=np.float64)[:2] - occupancy.global_origin_xy
    ) / occupancy.resolution_m
    cell = np.rint(continuous).astype(np.int64)
    if np.any(cell < 0) or np.any(cell >= np.asarray(occupancy.grid.shape)):
        raise ValueError("benchmark point lies outside the occupancy map")
    return int(cell[0]), int(cell[1])


def _oracle_cells(runtime: IsaacBenchmarkRuntime, episode: BenchmarkEpisode) -> np.ndarray:
    path = runtime.oracle_path_xy(episode)
    occupancy = runtime.occupancy
    return np.rint(
        (path - occupancy.global_origin_xy) / occupancy.resolution_m
    ).astype(np.int64)


def _kinodynamic_reference_arrays(
    runtime: IsaacBenchmarkRuntime,
    episode: BenchmarkEpisode,
) -> dict[str, np.ndarray]:
    plan = runtime.kinodynamic_plan(episode)
    if plan.status != "success" or len(plan.waypoints) < 2:
        raise RuntimeError(f"kinodynamic reference unavailable: {plan.status}")
    return {
        "kinodynamic_reference_pose": np.asarray(
            [(item.x_m, item.y_m, item.yaw_rad) for item in plan.waypoints],
            dtype=np.float64,
        ),
        "kinodynamic_reference_drive_direction": np.asarray(
            [1 if item.drive_direction == "forward" else -1 for item in plan.waypoints],
            dtype=np.int8,
        ),
        "kinodynamic_reference_command": np.asarray(
            [
                (item.main_command_rad_s, item.auxiliary_angle_rad)
                for item in plan.waypoints
            ],
            dtype=np.float64,
        ),
    }


def _dynamic_reference_arrays(
    runtime: IsaacBenchmarkRuntime,
    episode: BenchmarkEpisode,
) -> dict[str, np.ndarray]:
    plan = runtime.dynamic_plan(episode)
    if plan.status != "success" or len(plan.waypoints) < 2:
        raise RuntimeError(f"dynamic reference unavailable: {plan.status}")
    direction_codes = {"reverse": -1, "stopped": 0, "forward": 1}
    return {
        "dynamic_reference_time_s": np.asarray(
            [item.time_from_start_s for item in plan.waypoints], dtype=np.float64
        ),
        "dynamic_reference_pose": np.asarray(
            [(item.x_m, item.y_m, item.yaw_rad) for item in plan.waypoints],
            dtype=np.float64,
        ),
        "dynamic_reference_speed_m_s": np.asarray(
            [item.speed_m_s for item in plan.waypoints], dtype=np.float64
        ),
        "dynamic_reference_command": np.asarray(
            [
                (item.main_command_rad_s, item.auxiliary_angle_rad)
                for item in plan.waypoints
            ],
            dtype=np.float64,
        ),
        "dynamic_reference_drive_direction": np.asarray(
            [direction_codes[item.drive_direction] for item in plan.waypoints],
            dtype=np.int8,
        ),
    }


def _cross_track_errors(points_xy: object, reference_xy: object) -> np.ndarray:
    points = np.asarray(points_xy, dtype=np.float64)
    reference = np.asarray(reference_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] == 0:
        raise ValueError("points_xy must have shape (N, 2), N >= 1")
    if reference.ndim != 2 or reference.shape[1] != 2 or reference.shape[0] < 2:
        raise ValueError("reference_xy must have shape (N, 2), N >= 2")
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(reference)):
        raise ValueError("cross-track inputs must be finite")
    return np.linalg.norm(
        points[:, None, :] - reference[None, :, :], axis=2
    ).min(axis=1)


def _combine_vertical_panels(
    upper_path: str | Path,
    lower_path: str | Path,
    output_path: str | Path,
    *,
    cv2_module: Any | None = None,
) -> Path:
    """Combine audit panels without using Pillow inside an Isaac process."""

    if cv2_module is None:
        import cv2 as cv2_module

    upper = cv2_module.imread(str(upper_path), cv2_module.IMREAD_COLOR)
    lower = cv2_module.imread(str(lower_path), cv2_module.IMREAD_COLOR)
    if upper is None or lower is None:
        raise RuntimeError("could not read trajectory audit panel")
    if upper.ndim != 3 or lower.ndim != 3 or upper.shape[1:] != lower.shape[1:]:
        raise ValueError("trajectory audit panels must have matching width and channels")
    combined = np.concatenate((upper, lower), axis=0)
    destination = Path(output_path)
    if not cv2_module.imwrite(str(destination), combined):
        raise RuntimeError(f"could not write trajectory audit panel: {destination}")
    return destination


def _render_task_and_trajectory(
    writer: AuditBundleWriter,
    runtime: IsaacBenchmarkRuntime,
    episode: BenchmarkEpisode,
    result: Any,
) -> None:
    occupancy = runtime.occupancy
    oracle_cells = _oracle_cells(runtime, episode)
    overlay = NavigationTaskOverlay(
        inflated_grid=runtime.inflated_grid,
        start_cell=_world_to_cell(episode.start_position_world_m, occupancy),
        start_yaw_rad=_yaw_from_wxyz(episode.start_quaternion_world_wxyz),
        goal_cell=_world_to_cell(episode.goal_position_world_m, occupancy),
        oracle_path_cells=oracle_cells,
        metrics=(
            f"difficulty: {episode.difficulty}",
            f"euclidean: {episode.euclidean_distance_m:.3f} m",
            f"A*: {episode.oracle_path_length_m:.3f} m",
            f"policy: {result.policy_name}",
            f"termination: {result.termination_reason}",
        ),
    )
    task_path, layers = render_navigation_task_map(
        writer.path("task_card.png"),
        occupancy,
        overlay,
        title=f"PointGoal task {episode.episode_id}",
    )
    if not all(layers.values()):
        raise RuntimeError("task-card renderer omitted a required layer")

    collision_points = result.positions_world_xy[result.collisions]
    if result.policy_name == "oracle_v2":
        reference_arrays = _dynamic_reference_arrays(runtime, episode)
        reference_xy = reference_arrays["dynamic_reference_pose"][:, :2]
        reference_label = "dynamic reference"
    else:
        reference_arrays = _kinodynamic_reference_arrays(runtime, episode)
        reference_xy = reference_arrays["kinodynamic_reference_pose"][:, :2]
        reference_label = "kinodynamic reference"
    geometric_xy = runtime.oracle_path_xy(episode)
    actual_panel = writer.path("actual_trajectory_panel.png")
    render_trajectory_map(
        actual_panel,
        occupancy,
        (
            TrajectoryRecord(
                label="geometric A*",
                points_xy=geometric_xy,
                color_rgb=(35, 105, 210),
                draw_endpoint_markers=False,
            ),
            TrajectoryRecord(
                label=reference_label,
                points_xy=reference_xy,
                color_rgb=(142, 68, 173),
                draw_endpoint_markers=False,
            ),
            TrajectoryRecord(
                label=f"actual: {result.policy_name}",
                points_xy=result.positions_world_xy,
                collision_points_xy=collision_points,
                color_rgb=(230, 126, 34),
                end_color_rgb=(204, 45, 146),
            ),
        ),
        title=f"Executed trajectory {episode.episode_id}",
    )
    _combine_vertical_panels(
        task_path,
        actual_panel,
        writer.path("trajectory_overlay.png"),
    )
    actual_panel.unlink()


def _finite_clearances(
    values: np.ndarray,
    runtime: IsaacBenchmarkRuntime,
) -> tuple[np.ndarray, np.ndarray]:
    source = np.asarray(values, dtype=np.float64)
    infinite = np.isposinf(source)
    if np.any(np.isnan(source)) or np.any(np.isneginf(source)):
        raise ValueError("runtime clearances cannot contain NaN or negative infinity")
    occupancy = runtime.occupancy
    maximum = float(np.linalg.norm(np.asarray(occupancy.grid.shape)) * occupancy.resolution_m)
    return np.where(infinite, maximum, source), infinite


def _execution_state_arrays(result: Any) -> dict[str, np.ndarray]:
    """Return state- and action-aligned arrays needed to audit control failures."""

    names = (
        "yaw_world_rad",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "applied_action",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
        "roll_world_rad",
        "policy_reference_index",
        "policy_cross_track_error_m",
        "policy_active_drive_direction",
        "policy_desired_drive_direction",
        "policy_pending_drive_direction",
        "policy_stopped_cycles",
        "policy_braking_state_code",
        "dynamic_reference_index",
        "dynamic_position_error_m",
        "dynamic_yaw_error_rad",
        "dynamic_auxiliary_error_rad",
    )
    arrays = {name: np.asarray(getattr(result, name)) for name in names}
    intervention_names = (
        "policy_reference_progress_s_m",
        "policy_reference_heading_rad",
        "policy_reference_curvature_1_m",
        "policy_desired_curvature_1_m",
        "policy_baseline_main_command_rad_s",
        "policy_scheduled_main_command_rad_s",
        "policy_auxiliary_target_angle_rad",
        "policy_auxiliary_velocity_rad_s",
        "policy_scheduler_active",
        "policy_scheduler_trigger_code",
    )
    if all(hasattr(result, name) for name in intervention_names):
        arrays.update({name: np.asarray(getattr(result, name)) for name in intervention_names})
    return arrays


def _command_text(command: Sequence[str]) -> str | None:
    try:
        completed = subprocess.run(
            list(command), capture_output=True, text=True, check=False, timeout=10
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    value = completed.stdout.strip()
    return value or None


def _environment_payload(
    args: argparse.Namespace,
    episode: BenchmarkEpisode,
    protocol: Any,
    policy: Any,
    runtime: IsaacBenchmarkRuntime,
) -> dict[str, object]:
    oracle_lookahead_m = _selected_oracle_lookahead_m(args)
    policy_config_payload: dict[str, object] | None = None
    if args.policy in {"oracle_constraint_v1", "oracle_intervention_v1"}:
        loader = (
            _load_constraint_policy_config
            if args.policy == "oracle_constraint_v1"
            else _load_intervention_policy_config
        )
        config_values, config_sha256, config_path = loader(getattr(args, "policy_config", None))
        policy_config_payload = {
            "path": str(config_path),
            "sha256": config_sha256,
            "values": config_values,
        }
    try:
        import cv2

        opencv_version: str | None = cv2.__version__
    except Exception:
        opencv_version = None
    try:
        import torch

        torch_version: str | None = torch.__version__
        cuda_version: str | None = torch.version.cuda
        gpu_name: str | None = (
            torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
        )
    except Exception:
        torch_version = None
        cuda_version = None
        gpu_name = None
    repository_root = Path(__file__).resolve().parents[2]
    manifest = load_manifest(args.manifest)
    payload: dict[str, object] = {
        "benchmark_version": manifest.benchmark_version,
        "evaluation_protocol_id": _evaluation_protocol_id(args),
        "episode_budget_s": RunnerConfig().episode_budget_s,
        "max_action_blocks": RunnerConfig().max_action_blocks,
        "manifest_sha256": manifest.manifest_sha256,
        "git_commit": _command_text(
            ("git", "-C", str(repository_root), "rev-parse", "HEAD")
        ),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "opencv_version": opencv_version,
        "torch_version": torch_version,
        "cuda_version": cuda_version,
        "gpu_name": gpu_name,
        "nvidia_driver": _command_text(
            ("nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader")
        ),
        "action_protocol_id": protocol.protocol_id,
        "action_minimum": list(protocol.minimum),
        "action_maximum": list(protocol.maximum),
        "controls_per_block": protocol.controls_per_block,
        "localization_source": "perfect_localization",
        "scene_uri": episode.scene_uri,
        "scene_sha256": episode.scene_sha256,
        "occupancy_sha256": episode.occupancy_sha256,
        "policy": args.policy,
        "policy_config": policy_config_payload,
        "oracle_lookahead_m": oracle_lookahead_m,
        "drive_profile_path": str(
            getattr(
                getattr(runtime, "config", None),
                "drive_profile_path",
                DEFAULT_DRIVE_PROFILE,
            )
        ),
        "policy_online_input_permissions": sorted(policy.online_input_permissions),
        "privileged": bool(policy.privileged),
        "cli_seed": args.seed,
        "episode_seed": episode.seed,
        "gui": bool(args.gui),
        "record_video": True,
        "video_fps": 10.0,
        "physics_hz": 50,
        "control_hz": 50,
        "stereo_hz": 10,
        "replan_hz": 10,
        "physical_contact_source": (
            "filtered_scene_contact_force_gt_1N"
            if runtime.contact_measurement_available
            else "unavailable"
        ),
        "collision_mode": getattr(args, "collision_mode", "map_clearance_v1"),
        "collision_source": ("filtered_obstacle_contact_gt_1N_50Hz_block_latched"
            if getattr(args, "collision_mode", "map_clearance_v1") == "physical_contact_v2"
            else "frozen_truth_clearance_le_zero"),
        "map_clearance_role": "diagnostic_risk_only" if getattr(args, "collision_mode", "map_clearance_v1") == "physical_contact_v2" else "collision_adjudication",
        "execution_state_alignment": "reset_plus_post_action_states",
        "policy_diagnostic_alignment": "post_policy_act_pre_runtime_execute",
        "drive_direction_codes": {"reverse": -1, "none_or_unknown": 0, "forward": 1},
        "braking_state_codes": {
            "unknown": 0,
            "tracking": 1,
            "direction_change": 2,
            "goal_braking": 3,
            "goal_reached": 4,
            "tracking_infeasible": 5,
        },
        "trajectory_overlay_layers": sorted(REQUIRED_TRAJECTORY_LAYERS),
    }
    auxiliary_velocity_limit = getattr(args, "auxiliary_velocity_limit_rad_s", None)
    if auxiliary_velocity_limit is not None:
        payload["auxiliary_velocity_limit_rad_s"] = float(auxiliary_velocity_limit)
    adapter_metadata = getattr(policy, "adapter_metadata", None)
    if adapter_metadata is not None:
        payload["policy_adapter"] = adapter_metadata
    if args.policy == "oracle_v2":
        payload["dynamic_planner"] = {
            "planner_version": "dynamic_lattice_v2",
            "drive_profile_path": str(runtime.config.drive_profile_path),
            "primitive_duration_s": 1.0,
            "integration_dt_s": 0.02,
            "main_acceleration_limit_rad_s2": 0.5,
            "auxiliary_rate_limit_rad_s": math.radians(30.0),
            "maximum_predicted_duration_s": 32.0,
            "node_budget": 500_000,
            "planning_timeout_s": 60.0,
        }
    return payload


def _maximum_reference_rates(reference_arrays: dict[str, np.ndarray]) -> tuple[float, float]:
    time_s = np.asarray(reference_arrays["dynamic_reference_time_s"], dtype=np.float64)
    command = np.asarray(reference_arrays["dynamic_reference_command"], dtype=np.float64)
    intervals = np.diff(time_s)
    rates = np.abs(np.diff(command, axis=0)) / intervals[:, None]
    return float(np.max(rates[:, 0])), float(np.max(rates[:, 1]))


def _failure_audit_scalar(value: object) -> object:
    if value is None or isinstance(value, str):
        return value
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None
    return None


def _write_failure_audit(
    writer: AuditBundleWriter,
    episode: BenchmarkEpisode,
    error: BaseException,
    *,
    result: Any | None,
    planning_audit: dict[str, object],
) -> None:
    plan = planning_audit.get("plan")
    payload = {
        "schema_version": "benchmark_failure_audit_v1",
        "status": "unpublished_partial",
        "episode_id": episode.episode_id,
        "exception_type": type(error).__name__,
        "exception_message": str(error),
        "result": {
            "runtime_error": _failure_audit_scalar(
                getattr(result, "runtime_error", None)
            ),
            "termination_reason": _failure_audit_scalar(
                getattr(result, "termination_reason", None)
            ),
        },
        "planning_audit": {
            "status": _failure_audit_scalar(getattr(plan, "status", None)),
            "expanded_nodes": _failure_audit_scalar(
                getattr(plan, "expanded_nodes", None)
            ),
            "duration_s": _failure_audit_scalar(getattr(plan, "duration_s", None)),
            "planning_wall_time_s": _failure_audit_scalar(
                planning_audit.get("planning_wall_time_s")
            ),
        },
    }
    try:
        writer.write_json("failure.json", payload)
    except Exception as audit_error:
        print(
            "Benchmark failure audit write failed: "
            f"{type(audit_error).__name__}: {audit_error}",
            file=sys.stderr,
            flush=True,
        )
    concise_message = " ".join(str(error).splitlines())
    print(
        f"Benchmark episode {episode.episode_id} failed: "
        f"{type(error).__name__}: {concise_message}",
        file=sys.stderr,
        flush=True,
    )


def _collision_audit_payload(args: argparse.Namespace, runtime: IsaacBenchmarkRuntime) -> dict[str, object]:
    return {
        "evaluation_protocol_id": _evaluation_protocol_id(args),
        "collision_mode": getattr(args, "collision_mode", "map_clearance_v1"),
        "contact_measurement_available": runtime.contact_measurement_available,
        "contact_threshold_n": 1.0,
        "samples": runtime.collision_contact_samples,
    }


def _run_one(args: argparse.Namespace, episode: BenchmarkEpisode) -> Path:
    writer = AuditBundleWriter(args.run_root, episode.episode_id)
    writer.begin()
    # Retain the exact frozen manifest bytes used for the run.
    writer.path("manifest.json").write_bytes(args.manifest.read_bytes())
    protocol = load_action_protocol(args.contract)
    if protocol.protocol_id != "ball_action_v2_aux30":
        raise ValueError("Benchmark V1 requires ball_action_v2_aux30")
    occupancy_path = _resolve_occupancy_path(
        args.scene_index, episode, args.data_root
    )
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            collision_mode=getattr(args, "collision_mode", "map_clearance_v1"),
            scene_root=args.scene_root,
            contract_path=args.contract,
            gui=bool(args.gui),
            record_video=True,
            occupancy_path=occupancy_path,
            drive_profile_path=getattr(args, "drive_profile", DEFAULT_DRIVE_PROFILE),
            auxiliary_velocity_limit_rad_s=getattr(
                args, "auxiliary_velocity_limit_rad_s", None
            ),
        )
    )
    return_code = 1
    result: Any | None = None
    planning_audit: dict[str, object] = {}
    failure_error: BaseException | None = None
    try:
        oracle_lookahead_m = _selected_oracle_lookahead_m(args)
        policy = _make_policy(
            args.policy,
            protocol,
            args.seed + episode.seed,
            oracle_lookahead_m=oracle_lookahead_m,
            checkpoint=getattr(args, "checkpoint", None),
            device=getattr(args, "device", "cuda"),
            wm_horizon=getattr(args, "wm_horizon", 5),
            wm_samples=getattr(args, "wm_samples", 64),
            wm_elites=getattr(args, "wm_elites", 8),
            wm_iterations=getattr(args, "wm_iterations", 3),
            wm_safety_weight=getattr(args, "wm_safety_weight", 0.0),
            allow_legacy_checkpoint=getattr(args, "allow_legacy_checkpoint", False),
            policy_config=getattr(args, "policy_config", None),
        )
        runner_config = RunnerConfig()
        if args.policy == "oracle_v2":
            dynamic_plan_provider = runtime.dynamic_plan

            def audited_dynamic_plan(active_episode: BenchmarkEpisode) -> Any:
                if "plan" in planning_audit:
                    return planning_audit["plan"]
                started = time.monotonic()
                try:
                    plan = dynamic_plan_provider(active_episode)
                finally:
                    planning_audit["planning_wall_time_s"] = (
                        time.monotonic() - started
                    )
                planning_audit["plan"] = plan
                return plan

            runtime.dynamic_plan = audited_dynamic_plan  # type: ignore[method-assign]
        result = run_episode(
            runtime,
            policy,
            episode,
            action_protocol=protocol,
            config=runner_config,
        )
        if result.positions_world_xy.shape[0] == 0:
            raise RuntimeError(result.runtime_error or "runtime produced no initial state")
        _render_task_and_trajectory(writer, runtime, episode, result)
        if result.runtime_error is None:
            runtime.flush_terminal_video_frames()
        frames = runtime.captured_frames
        write_rgb_video(writer.path("third_person.mp4"), frames["third_person"], fps=10.0)
        write_rgb_video(writer.path("stereo_view.mp4"), frames["stereo_view"], fps=10.0)

        writer.write_npz(
            "actions.npz",
            action_blocks=result.actions,
            controls=result.actions.reshape(-1, 2),
        )
        finite_clearance, infinite_clearance = _finite_clearances(result.clearances_m, runtime)
        reference_arrays = (
            _dynamic_reference_arrays(runtime, episode)
            if args.policy == "oracle_v2"
            else _kinodynamic_reference_arrays(runtime, episode)
        )
        writer.write_npz(
            "states.npz",
            position_world_xy=result.positions_world_xy,
            clearance_m=finite_clearance,
            clearance_was_positive_infinity=infinite_clearance,
            physical_contact=result.physical_contacts,
            collision=result.collisions,
            **_execution_state_arrays(result),
            **reference_arrays,
        )
        metrics = result.to_metrics_dict()
        metrics["evaluation_protocol_id"] = _evaluation_protocol_id(args)
        metrics["collision_mode"] = getattr(args, "collision_mode", "map_clearance_v1")
        metrics["map_nonpositive_clearance_samples"] = int(np.count_nonzero(np.asarray(result.clearances_m) <= 0.0))
        metrics["episode_budget_s"] = runner_config.episode_budget_s
        metrics["max_action_blocks"] = runner_config.max_action_blocks
        metrics["oracle_lookahead_m"] = oracle_lookahead_m
        if args.policy in {"oracle_constraint_v1", "oracle_intervention_v1"}:
            loader = (
                _load_constraint_policy_config
                if args.policy == "oracle_constraint_v1"
                else _load_intervention_policy_config
            )
            config_values, config_sha256, config_path = loader(getattr(args, "policy_config", None))
            metrics["policy_config"] = {
                "path": str(config_path),
                "sha256": config_sha256,
                "values": config_values,
            }
        if metrics["minimum_clearance_m"] is not None and not math.isfinite(
            float(metrics["minimum_clearance_m"])
        ):
            metrics["minimum_clearance_m"] = None
        plan = (
            runtime.dynamic_plan(episode)
            if args.policy == "oracle_v2"
            else runtime.kinodynamic_plan(episode)
        )
        reference_xy = np.asarray(
            [(item.x_m, item.y_m) for item in plan.waypoints], dtype=np.float64
        )
        cross_track = _cross_track_errors(result.positions_world_xy, reference_xy)
        diagnostics = getattr(policy, "diagnostics", {})
        failure_subreason = (
            diagnostics.get("failure_subreason")
            if isinstance(diagnostics, dict)
            else None
        )
        metrics["certification"] = {
            "planning_status": plan.status,
            "expanded_nodes": plan.expanded_nodes,
            "direction_changes": plan.direction_changes,
            "minimum_clearance_m": metrics["minimum_clearance_m"],
            "maximum_cross_track_error_m": float(np.max(cross_track)),
            "mean_cross_track_error_m": float(np.mean(cross_track)),
            "failure_subreason": failure_subreason,
        }
        if args.policy == "oracle_v2":
            maximum_main_rate, maximum_auxiliary_rate = _maximum_reference_rates(
                reference_arrays
            )
            metrics["certification"].update(
                {
                    "planner_version": "dynamic_lattice_v2",
                    "planning_wall_time_s": float(
                        planning_audit["planning_wall_time_s"]
                    ),
                    "predicted_duration_s": plan.duration_s,
                    "maximum_main_rate_rad_s2": maximum_main_rate,
                    "maximum_auxiliary_rate_rad_s": maximum_auxiliary_rate,
                }
            )
        writer.write_json("collision_audit.json", _collision_audit_payload(args, runtime))
        writer.write_json("metrics.json", metrics)
        writer.write_json(
            "environment.json",
            _environment_payload(args, episode, protocol, policy, runtime),
        )
        writer.write_json(
            "human_review.json",
            {"status": "pending", "reviewer": "", "labels": [], "notes": ""},
        )
        published = writer.publish()
        return_code = 0
        return published
    except BaseException as error:
        failure_error = error
        # Persist sensor failures before native close terminates the process.
        # Audit I/O must not mask the original execution failure.
        try:
            writer.write_json("collision_audit.json", _collision_audit_payload(args, runtime))
        except Exception:
            pass
        _write_failure_audit(
            writer,
            episode,
            error,
            result=result,
            planning_audit=planning_audit,
        )
        raise
    finally:
        try:
            runtime.close(return_code=return_code)
        except BaseException:
            if failure_error is None:
                raise


def _select_episodes(args: argparse.Namespace) -> tuple[BenchmarkEpisode, ...]:
    manifest = load_manifest(args.manifest)
    if manifest.action_protocol_id != "ball_action_v2_aux30":
        raise ValueError("manifest action protocol is not Benchmark V1")
    selected = tuple(
        episode
        for episode in manifest.episodes
        if episode.split == args.split
        and (args.episode_id is None or episode.episode_id == args.episode_id)
    )
    if not selected:
        raise ValueError("no manifest episodes matched --split/--episode-id")
    return selected


def _child_command(args: argparse.Namespace, episode_id: str) -> list[str]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--manifest",
        str(args.manifest),
        "--split",
        args.split,
        "--policy",
        args.policy,
        "--scene-root",
        str(args.scene_root),
        "--run-root",
        str(args.run_root),
        "--gui" if args.gui else "--headless",
        "--record-video",
        "--episode-id",
        episode_id,
        "--seed",
        str(args.seed),
        "--oracle-lookahead-m",
        str(_selected_oracle_lookahead_m(args)),
        "--contract",
        str(args.contract),
        "--scene-index",
        str(args.scene_index),
        "--drive-profile",
        str(getattr(args, "drive_profile", DEFAULT_DRIVE_PROFILE)),
        "--collision-mode",
        getattr(args, "collision_mode", "map_clearance_v1"),
        "--_worker",
    ]
    auxiliary_velocity_limit = getattr(args, "auxiliary_velocity_limit_rad_s", None)
    if auxiliary_velocity_limit is not None:
        command.extend(
            ("--auxiliary-velocity-limit-rad-s", str(auxiliary_velocity_limit))
        )
    policy_config = getattr(args, "policy_config", None)
    if policy_config is not None:
        command.extend(("--policy-config", str(policy_config)))
    if args.data_root is not None:
        command.extend(("--data-root", str(args.data_root)))
    checkpoint = getattr(args, "checkpoint", None)
    if checkpoint is not None:
        command.extend(("--checkpoint", str(checkpoint)))
    device = getattr(args, "device", "cuda")
    wm_horizon = getattr(args, "wm_horizon", 5)
    wm_samples = getattr(args, "wm_samples", 64)
    wm_elites = getattr(args, "wm_elites", 8)
    wm_iterations = getattr(args, "wm_iterations", 3)
    wm_safety_weight = getattr(args, "wm_safety_weight", 0.0)
    command.extend(
        (
            "--device",
            str(device),
            "--wm-horizon",
            str(wm_horizon),
            "--wm-samples",
            str(wm_samples),
            "--wm-elites",
            str(wm_elites),
            "--wm-iterations",
            str(wm_iterations),
            "--wm-safety-weight",
            str(wm_safety_weight),
        )
    )
    if getattr(args, "allow_legacy_checkpoint", False):
        command.append("--allow-legacy-checkpoint")
    return command


def _write_summary(
    run_root: Path,
    episodes: Sequence[BenchmarkEpisode],
    *,
    oracle_lookahead_m: float = DEFAULT_ORACLE_LOOKAHEAD_M,
) -> Path:
    selected_oracle_lookahead_m = _validate_oracle_lookahead_m(oracle_lookahead_m)
    metrics: list[dict[str, object]] = []
    for episode in episodes:
        path = run_root / f"episode_{episode.episode_id}" / "metrics.json"
        if not path.is_file():
            raise RuntimeError(f"published episode is missing metrics: {path}")
        with path.open("r", encoding="utf-8") as stream:
            payload = json.load(stream)
        recorded_lookahead = payload.get(
            "oracle_lookahead_m", DEFAULT_ORACLE_LOOKAHEAD_M
        )
        if (
            _validate_oracle_lookahead_m(recorded_lookahead)
            != selected_oracle_lookahead_m
        ):
            raise ValueError("summary requires matching oracle lookahead values")
        payload["scene_id"] = Path(episode.scene_uri).stem
        metrics.append(payload)
    config = RunnerConfig()
    protocol_ids = {item.get("evaluation_protocol_id") for item in metrics}
    if len(protocol_ids) != 1 or not protocol_ids <= {"ball_pointgoal_v1_1_35s", "ball_pointgoal_v1_2_contact_35s"}:
        raise ValueError("summary requires matching 35-second evaluation protocols")
    evaluation_protocol_id = next(iter(protocol_ids))
    if any(
        item.get("episode_budget_s") != config.episode_budget_s
        for item in metrics
    ):
        raise ValueError("summary requires matching 35-second evaluation protocols")
    summary = summarize_navigation(
        metrics,
        goal_tolerance_m=config.goal_tolerance_m,
        max_mpc_steps=config.max_action_blocks,
    )
    summary["evaluation_protocol_id"] = evaluation_protocol_id
    summary["collision_mode"] = "physical_contact_v2" if evaluation_protocol_id == "ball_pointgoal_v1_2_contact_35s" else "map_clearance_v1"
    summary["episode_budget_s"] = config.episode_budget_s
    summary["oracle_lookahead_m"] = selected_oracle_lookahead_m
    summary["scene_cluster_bootstrap"] = {
        field: cluster_bootstrap_interval(
            metrics,
            field=field,
            cluster_field="scene_id",
            seed=3072,
            samples=1000,
        )
        for field in ("success", "collision", "timeout", "runtime_error", "spl")
    }
    destination = run_root / "summary.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if not args.record_video:
        print("Formal golden/dev/test benchmark runs require --record-video", file=sys.stderr)
        return 2
    if args.seed < 0:
        print("--seed must be non-negative", file=sys.stderr)
        return 2
    if (
        args.policy != "oracle"
        and args.oracle_lookahead_m != DEFAULT_ORACLE_LOOKAHEAD_M
    ):
        print(
            "--oracle-lookahead-m is only supported with --policy oracle",
            file=sys.stderr,
        )
        return 2
    try:
        episodes = _select_episodes(args)
        if len(episodes) > 1 and not args._worker:
            for episode in episodes:
                completed = subprocess.run(_child_command(args, episode.episode_id), check=False)
                if completed.returncode != 0:
                    raise RuntimeError(
                        f"episode worker failed for {episode.episode_id} with code "
                        f"{completed.returncode}"
                    )
            summary_path = _write_summary(
                args.run_root,
                episodes,
                oracle_lookahead_m=args.oracle_lookahead_m,
            )
            print(f"Benchmark summary: {summary_path}")
            return 0

        output = _run_one(args, episodes[0])
        if not args._worker:
            summary_path = _write_summary(
                args.run_root,
                episodes,
                oracle_lookahead_m=args.oracle_lookahead_m,
            )
            print(f"Benchmark summary: {summary_path}")
        print(f"Published audit bundle: {output}")
        return 0
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        print(f"Benchmark run failed: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
