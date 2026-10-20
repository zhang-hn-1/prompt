#!/usr/bin/env python3
"""Generate, render, review, and freeze Ball PointGoal Benchmark V1 tasks.

All subcommands except ``export-scene`` are pure Python and never import
Isaac Sim.  A scene-index JSON contains a ``scenes`` array; each scene record
provides ``scene_uri`` and ``occupancy_uri`` plus optional SHA256 values,
``minimum_nearby_obstacles`` validation overrides, and an optional
``corridors`` array of ``mask_uri``/``width_m`` records for E2.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np
import yaml
from scipy.ndimage import label


SCRIPT_DIR = Path(__file__).resolve().parent
BALL_WM_ROOT = SCRIPT_DIR.parent
if str(BALL_WM_ROOT) not in sys.path:
    sys.path.insert(0, str(BALL_WM_ROOT))

from benchmark.grid_planning import inflate_obstacles  # noqa: E402
from benchmark.manifest import BenchmarkEpisode, HumanReview  # noqa: E402
from benchmark.task_cards import (  # noqa: E402
    CANDIDATE_SCHEMA_VERSION,
    freeze_candidates,
    load_occupancy_npz,
    render_candidates,
    review_candidates,
)
from benchmark.task_generation import (  # noqa: E402
    build_two_sided_corridor_mask,
    validate_task,
)


DEFAULT_BENCHMARK_CONFIG = BALL_WM_ROOT / "configs" / "benchmark_v1.yaml"
SCENE_ROOT_PREFIX = "${SCENE_ROOT}/"
DATA_ROOT_PREFIX = "${DATA_ROOT}/"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_root_uri(uri: str, *, prefix: str, root: Path, field: str) -> Path:
    if not uri.startswith(prefix):
        raise ValueError(f"{field} must start with {prefix}")
    relative = Path(uri[len(prefix) :])
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        raise ValueError(f"{field} escapes its configured root")
    resolved = root.resolve().joinpath(relative).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"{field} escapes its configured root") from error
    return resolved


def _atomic_write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def _load_object(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _load_config(path: Path = DEFAULT_BENCHMARK_CONFIG) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise ValueError("benchmark config root must be a mapping")
    return payload


def _scene_records(scene_index: Mapping[str, object]) -> list[dict[str, object]]:
    scenes = scene_index.get("scenes")
    if not isinstance(scenes, list) or not scenes:
        raise ValueError("scene index must contain a non-empty scenes array")
    if any(not isinstance(scene, dict) for scene in scenes):
        raise ValueError("each scene-index entry must be an object")
    return scenes  # type: ignore[return-value]


def _resolve_data_uri(uri: object, *, data_root: Path, index_dir: Path) -> Path:
    if not isinstance(uri, str) or not uri:
        raise ValueError("data URI must be a non-empty string")
    if uri.startswith(DATA_ROOT_PREFIX):
        return _resolve_root_uri(
            uri,
            prefix=DATA_ROOT_PREFIX,
            root=data_root,
            field="data URI",
        )
    path = Path(uri)
    return path.resolve() if path.is_absolute() else (index_dir / path).resolve()


def _sample_corridor_conditioned_pair(
    inflated_grid: np.ndarray,
    corridor_mask: np.ndarray,
    *,
    minimum_distance_m: float,
    maximum_distance_m: float,
    minimum_corridor_path_length_m: float,
    resolution_m: float,
    rng: np.random.Generator,
) -> tuple[tuple[int, int], tuple[int, int]]:
    """Sample free endpoints on opposite extensions of a corridor centerline."""

    inflated = np.asarray(inflated_grid)
    corridor = np.asarray(corridor_mask, dtype=bool)
    if inflated.ndim != 2 or corridor.shape != inflated.shape:
        raise ValueError("inflated grid and corridor mask must have matching 2-D shapes")
    resolution = float(resolution_m)
    minimum_cells = float(minimum_distance_m) / resolution
    maximum_cells = float(maximum_distance_m) / resolution
    minimum_corridor_cells = int(
        math.ceil(float(minimum_corridor_path_length_m) / resolution)
    ) + 1
    if resolution <= 0.0 or minimum_cells <= 0.0 or maximum_cells < minimum_cells:
        raise ValueError("distance bounds and resolution must be positive and ordered")

    usable = corridor & (inflated == 0)
    components, count = label(usable, structure=np.ones((3, 3), dtype=np.uint8))
    component_sizes = np.bincount(components.ravel(), minlength=count + 1)
    eligible = [
        index
        for index in range(1, count + 1)
        if component_sizes[index] >= max(2, minimum_corridor_cells)
    ]
    if not eligible:
        raise ValueError("corridor mask contains no connected free centerline")

    def nearest_free(target: np.ndarray) -> tuple[int, int] | None:
        radius = max(2, int(math.ceil(1.0 / resolution)))
        lower = np.maximum(np.floor(target - radius).astype(np.int64), 0)
        upper = np.minimum(np.ceil(target + radius).astype(np.int64) + 1, inflated.shape)
        candidates = np.argwhere(
            inflated[lower[0] : upper[0], lower[1] : upper[1]] == 0
        ) + lower
        if candidates.size == 0:
            return None
        nearest = candidates[np.argmin(np.sum((candidates - target) ** 2, axis=1))]
        return int(nearest[0]), int(nearest[1])

    for _ in range(32):
        component = eligible[int(rng.integers(0, len(eligible)))]
        points = np.argwhere(components == component).astype(np.float64)
        center = points.mean(axis=0)
        centered = points - center
        _, eigenvectors = np.linalg.eigh(centered.T @ centered)
        tangent = eigenvectors[:, -1]
        dominant = int(np.argmax(np.abs(tangent)))
        if tangent[dominant] < 0.0:
            tangent = -tangent
        total = float(rng.uniform(minimum_cells, maximum_cells))
        fraction = float(rng.uniform(0.4, 0.6))
        start = nearest_free(center - tangent * total * fraction)
        goal = nearest_free(center + tangent * total * (1.0 - fraction))
        if start is None or goal is None or start == goal:
            continue
        distance = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
        if minimum_cells <= distance <= maximum_cells:
            return start, goal
    raise ValueError("could not place corridor-conditioned endpoints within distance bounds")


def _candidate_from_pair(
    *,
    scene: Mapping[str, object],
    occupancy_uri: str,
    occupancy_path: Path,
    occupancy: object,
    planning_grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    difficulty: str,
    split: str,
    episode_id: str,
    seed: int,
    scene_sha256: str,
    corridor_mask: np.ndarray | None,
    corridor_mask_uri: str | None,
    corridor_width_m: float | None,
    ball_radius_m: float,
    safety_margin_m: float,
    e1_min_detour_ratio: float,
    nearby_obstacle_radius_m: float,
    minimum_nearby_obstacles: int,
    minimum_corridor_path_length_m: float,
) -> dict[str, object]:
    evidence = validate_task(
        planning_grid,
        start=start,
        goal=goal,
        difficulty=difficulty,  # type: ignore[arg-type]
        resolution_m=occupancy.resolution_m,
        ball_radius_m=ball_radius_m,
        safety_margin_m=safety_margin_m,
        e1_min_detour_ratio=e1_min_detour_ratio,
        corridor_mask=corridor_mask,
        corridor_width_m=corridor_width_m,
        nearby_obstacle_radius_m=nearby_obstacle_radius_m,
        minimum_nearby_obstacles=minimum_nearby_obstacles,
        minimum_corridor_path_length_m=minimum_corridor_path_length_m,
    )
    start_xy = occupancy.global_origin_xy + np.asarray(start) * occupancy.resolution_m
    goal_xy = occupancy.global_origin_xy + np.asarray(goal) * occupancy.resolution_m
    yaw = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
    navigation_z = float(scene.get("navigation_z_m", 0.45))
    episode = BenchmarkEpisode(
        episode_id=episode_id,
        split=split,  # type: ignore[arg-type]
        difficulty=difficulty,  # type: ignore[arg-type]
        scene_uri=str(scene["scene_uri"]),
        scene_sha256=scene_sha256,
        occupancy_sha256=_sha256_file(occupancy_path),
        start_position_world_m=(float(start_xy[0]), float(start_xy[1]), navigation_z),
        start_quaternion_world_wxyz=(
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
        ),
        goal_position_world_m=(float(goal_xy[0]), float(goal_xy[1]), navigation_z),
        euclidean_distance_m=evidence.euclidean_distance_m,
        oracle_path_length_m=evidence.oracle_path_length_m,
        seed=seed,
        human_review=HumanReview(
            status="needs_adjustment",
            reviewer="unassigned",
            notes="pending GUI review",
        ),
        classification_evidence=asdict(evidence),
    )
    payload = episode.to_dict()
    payload["occupancy_uri"] = occupancy_uri
    payload["minimum_nearby_obstacles"] = minimum_nearby_obstacles
    if corridor_mask_uri is not None:
        payload["corridor_mask_uri"] = corridor_mask_uri
        payload["corridor_width_m"] = corridor_width_m
    return payload


def _scene_by_id(
    scenes: Sequence[Mapping[str, object]], scene_id: str
) -> Mapping[str, object]:
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise ValueError("scene_id must be a non-empty string")
    matches = [scene for scene in scenes if scene.get("scene_id") == scene_id]
    if len(matches) != 1:
        raise ValueError(f"scene_id must resolve exactly once: {scene_id!r}")
    return matches[0]


def _minimum_nearby_obstacles_for_scene(
    scene: Mapping[str, object],
    *,
    default: int,
) -> int:
    """Return a non-negative per-scene override or the global default."""

    value = scene.get("minimum_nearby_obstacles", default)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("minimum_nearby_obstacles must be a non-negative integer")
    return value


def _world_xy_to_cell(
    world_xy: Sequence[float], occupancy: object
) -> tuple[int, int]:
    values = np.asarray(world_xy, dtype=np.float64)
    if values.shape != (2,) or not np.all(np.isfinite(values)):
        raise ValueError(
            "fixed start and goal must each contain two finite world coordinates"
        )
    continuous = (values - occupancy.global_origin_xy) / occupancy.resolution_m
    rounded = np.rint(continuous)
    if not np.allclose(continuous, rounded, atol=1e-5, rtol=0.0):
        raise ValueError("fixed start and goal must align with occupancy cell centers")
    cell = tuple(int(value) for value in rounded)
    if not (
        0 <= cell[0] < occupancy.grid.shape[0]
        and 0 <= cell[1] < occupancy.grid.shape[1]
    ):
        raise ValueError("fixed start or goal lies outside the frozen occupancy grid")
    return cell


def generate_fixed_candidate(
    scene_index_path: str | Path,
    *,
    scene_id: str,
    split: str,
    difficulty: str,
    start_xy_m: Sequence[float],
    goal_xy_m: Sequence[float],
    episode_id: str,
    seed: int,
    output_path: str | Path,
) -> dict[str, object]:
    """Create one candidate at exact world-frame endpoints on a frozen map."""

    if split not in ("golden", "dev"):
        raise ValueError("generated split must be golden or dev")
    if difficulty not in ("E0", "E1", "E2"):
        raise ValueError(f"unsupported difficulty: {difficulty!r}")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a non-negative integer")

    index_path = Path(scene_index_path)
    scene_index = _load_object(index_path)
    scene = _scene_by_id(_scene_records(scene_index), scene_id)
    allowed = scene.get("difficulties")
    if not isinstance(allowed, list) or difficulty not in allowed:
        raise ValueError(f"scene {scene_id!r} does not support {difficulty}")
    scene_uri = scene.get("scene_uri")
    occupancy_uri = scene.get("occupancy_uri")
    if not isinstance(scene_uri, str) or not isinstance(occupancy_uri, str):
        raise ValueError("each scene requires scene_uri and occupancy_uri")

    config = _load_config()
    data_root_value = os.environ.get("DATA_ROOT")
    data_root = (
        Path(data_root_value).expanduser().resolve()
        if data_root_value
        else index_path.parent.resolve()
    )
    occupancy_path = _resolve_data_uri(
        occupancy_uri,
        data_root=data_root,
        index_dir=index_path.parent.resolve(),
    )
    occupancy = load_occupancy_npz(occupancy_path)
    planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
    start = _world_xy_to_cell(start_xy_m, occupancy)
    goal = _world_xy_to_cell(goal_xy_m, occupancy)
    minimum_distance, maximum_distance = (
        float(value) for value in config["initial_goal_distance_m"]  # type: ignore[index]
    )
    distance_m = math.hypot(start[0] - goal[0], start[1] - goal[1]) * occupancy.resolution_m
    if distance_m < minimum_distance or distance_m > maximum_distance:
        raise ValueError(
            "fixed start-goal distance must be within "
            f"[{minimum_distance:.3f}, {maximum_distance:.3f}] m"
        )

    corridor_mask = None
    corridor_mask_uri = None
    corridor_width_m = None
    if difficulty == "E2":
        corridors = scene.get("corridors")
        if not isinstance(corridors, list) or len(corridors) != 1:
            raise ValueError("fixed E2 generation requires exactly one scene corridor")
        corridor = corridors[0]
        if not isinstance(corridor, Mapping):
            raise ValueError("E2 corridor entry must be an object")
        corridor_mask_uri_value = corridor.get("mask_uri")
        if not isinstance(corridor_mask_uri_value, str):
            raise ValueError("E2 corridor requires mask_uri")
        corridor_mask_uri = corridor_mask_uri_value
        corridor_width_m = float(corridor.get("width_m"))
        mask_path = _resolve_data_uri(
            corridor_mask_uri,
            data_root=data_root,
            index_dir=index_path.parent.resolve(),
        )
        corridor_mask = np.asarray(np.load(mask_path, allow_pickle=False), dtype=bool)

    scene_root_value = os.environ.get("SCENE_ROOT")
    if "scene_sha256" in scene:
        scene_sha256 = str(scene["scene_sha256"])
    elif scene_root_value:
        scene_path = _resolve_root_uri(
            scene_uri,
            prefix=SCENE_ROOT_PREFIX,
            root=Path(scene_root_value).expanduser().resolve(),
            field="scene_uri",
        )
        scene_sha256 = _sha256_file(scene_path)
    else:
        raise ValueError("scene_sha256 or SCENE_ROOT is required for generation")

    candidate = _candidate_from_pair(
        scene=scene,
        occupancy_uri=occupancy_uri,
        occupancy_path=occupancy_path,
        occupancy=occupancy,
        planning_grid=planning_grid,
        start=start,
        goal=goal,
        difficulty=difficulty,
        split=split,
        episode_id=episode_id,
        seed=seed,
        scene_sha256=scene_sha256,
        corridor_mask=corridor_mask,
        corridor_mask_uri=corridor_mask_uri,
        corridor_width_m=corridor_width_m,
        ball_radius_m=float(config["ball_radius_m"]),
        safety_margin_m=float(config["safety_margin_m"]),
        e1_min_detour_ratio=float(config["e1_min_detour_ratio"]),
        nearby_obstacle_radius_m=float(config["nearby_obstacle_radius_m"]),
        minimum_nearby_obstacles=_minimum_nearby_obstacles_for_scene(
            scene,
            default=int(config["minimum_nearby_obstacles"]),
        ),
        minimum_corridor_path_length_m=float(config["minimum_corridor_path_length_m"]),
    )
    payload: dict[str, object] = {
        "schema_version": CANDIDATE_SCHEMA_VERSION,
        "manifest_schema_version": "ball_pointgoal_benchmark_v1",
        "benchmark_version": str(config["benchmark_version"]),
        "action_protocol_id": str(config["action_protocol_id"]),
        "localization_source": str(config["localization_source"]),
        "generation_seed": seed,
        "episodes": [candidate],
    }
    _atomic_write_json(Path(output_path), payload)
    return payload


def generate_candidates(
    scene_index_path: str | Path,
    *,
    split: str,
    count_per_class: int,
    seed: int,
    output_path: str | Path,
) -> dict[str, object]:
    """Deterministically sample valid E0/E1/E2 candidates from frozen maps."""

    if split not in ("golden", "dev"):
        raise ValueError("generated split must be golden or dev")
    if isinstance(count_per_class, bool) or count_per_class <= 0:
        raise ValueError("count_per_class must be positive")
    index_path = Path(scene_index_path)
    scene_index = _load_object(index_path)
    scenes = _scene_records(scene_index)
    config = _load_config()
    data_root_value = os.environ.get("DATA_ROOT")
    data_root = (
        Path(data_root_value).expanduser().resolve()
        if data_root_value
        else index_path.parent.resolve()
    )
    scene_root_value = os.environ.get("SCENE_ROOT")
    scene_root = Path(scene_root_value).expanduser().resolve() if scene_root_value else None
    rng = np.random.default_rng(seed)
    minimum_distance, maximum_distance = (
        float(value) for value in config["initial_goal_distance_m"]  # type: ignore[index]
    )
    episodes: list[dict[str, object]] = []
    used: set[tuple[str, tuple[int, int], tuple[int, int]]] = set()
    occupancy_cache: dict[
        Path,
        tuple[object, np.ndarray, np.ndarray, np.ndarray],
    ] = {}
    corridor_cache: dict[Path, np.ndarray] = {}

    for difficulty in ("E0", "E1", "E2"):
        accepted = 0
        attempts = 0
        while accepted < count_per_class and attempts < 50000:
            attempts += 1
            scene = scenes[int(rng.integers(0, len(scenes)))]
            allowed = scene.get("difficulties")
            if isinstance(allowed, list) and difficulty not in allowed:
                continue
            scene_uri = scene.get("scene_uri")
            occupancy_uri = scene.get("occupancy_uri")
            if not isinstance(scene_uri, str) or not isinstance(occupancy_uri, str):
                raise ValueError("each scene requires scene_uri and occupancy_uri")
            occupancy_path = _resolve_data_uri(
                occupancy_uri,
                data_root=data_root,
                index_dir=index_path.parent.resolve(),
            )
            cached = occupancy_cache.get(occupancy_path)
            if cached is None:
                occupancy = load_occupancy_npz(occupancy_path)
                planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
                inflated = inflate_obstacles(
                    planning_grid,
                    radius_m=float(config["ball_radius_m"])
                    + float(config["safety_margin_m"]),
                    resolution_m=occupancy.resolution_m,
                )
                free_cells = np.argwhere(inflated == 0)
                cached = occupancy, planning_grid, inflated, free_cells
                occupancy_cache[occupancy_path] = cached
            occupancy, planning_grid, inflated, free_cells = cached
            if free_cells.shape[0] < 2:
                continue

            corridor_mask = None
            corridor_mask_uri = None
            corridor_width_m = None
            if difficulty == "E2":
                corridors = scene.get("corridors")
                if not isinstance(corridors, list) or not corridors:
                    continue
                corridor = corridors[int(rng.integers(0, len(corridors)))]
                if not isinstance(corridor, Mapping):
                    raise ValueError("corridor entries must be objects")
                corridor_mask_uri_value = corridor.get("mask_uri")
                if not isinstance(corridor_mask_uri_value, str):
                    raise ValueError("E2 corridor requires mask_uri")
                corridor_mask_uri = corridor_mask_uri_value
                corridor_width_m = float(corridor.get("width_m"))
                mask_path = _resolve_data_uri(
                    corridor_mask_uri,
                    data_root=data_root,
                    index_dir=index_path.parent.resolve(),
                )
                corridor_mask = corridor_cache.get(mask_path)
                if corridor_mask is None:
                    corridor_mask = np.asarray(
                        np.load(mask_path, allow_pickle=False),
                        dtype=bool,
                    )
                    corridor_cache[mask_path] = corridor_mask

            if difficulty == "E2":
                try:
                    start, goal = _sample_corridor_conditioned_pair(
                        inflated,
                        corridor_mask,
                        minimum_distance_m=minimum_distance,
                        maximum_distance_m=maximum_distance,
                        minimum_corridor_path_length_m=float(
                            config["minimum_corridor_path_length_m"]
                        ),
                        resolution_m=occupancy.resolution_m,
                        rng=rng,
                    )
                except ValueError:
                    continue
            else:
                chosen = rng.choice(free_cells.shape[0], size=2, replace=False)
                start = tuple(int(value) for value in free_cells[chosen[0]])
                goal = tuple(int(value) for value in free_cells[chosen[1]])
            distance = math.hypot(start[0] - goal[0], start[1] - goal[1]) * occupancy.resolution_m
            if distance < minimum_distance or distance > maximum_distance:
                continue
            identity = (scene_uri, start, goal)
            if identity in used:
                continue

            if "scene_sha256" in scene:
                scene_sha256 = str(scene["scene_sha256"])
            elif scene_root is not None:
                scene_path = _resolve_root_uri(
                    scene_uri,
                    prefix=SCENE_ROOT_PREFIX,
                    root=scene_root,
                    field="scene_uri",
                )
                scene_sha256 = _sha256_file(scene_path)
            else:
                raise ValueError("scene_sha256 or SCENE_ROOT is required for generation")

            episode_id = f"{Path(scene_uri).stem.lower()}_{difficulty.lower()}_{accepted + 1:04d}"
            try:
                candidate = _candidate_from_pair(
                    scene=scene,
                    occupancy_uri=occupancy_uri,
                    occupancy_path=occupancy_path,
                    occupancy=occupancy,
                    planning_grid=planning_grid,
                    start=start,
                    goal=goal,
                    difficulty=difficulty,
                    split=split,
                    episode_id=episode_id,
                    seed=seed + len(episodes),
                    scene_sha256=scene_sha256,
                    corridor_mask=corridor_mask,
                    corridor_mask_uri=corridor_mask_uri,
                    corridor_width_m=corridor_width_m,
                    ball_radius_m=float(config["ball_radius_m"]),
                    safety_margin_m=float(config["safety_margin_m"]),
                    e1_min_detour_ratio=float(config["e1_min_detour_ratio"]),
                    nearby_obstacle_radius_m=float(config["nearby_obstacle_radius_m"]),
                    minimum_nearby_obstacles=_minimum_nearby_obstacles_for_scene(
                        scene,
                        default=int(config["minimum_nearby_obstacles"]),
                    ),
                    minimum_corridor_path_length_m=float(
                        config["minimum_corridor_path_length_m"]
                    ),
                )
            except ValueError:
                continue
            episodes.append(candidate)
            used.add(identity)
            accepted += 1
        if accepted != count_per_class:
            raise RuntimeError(
                f"could not generate {count_per_class} valid {difficulty} tasks "
                f"after {attempts} attempts"
            )

    payload: dict[str, object] = {
        "schema_version": CANDIDATE_SCHEMA_VERSION,
        "manifest_schema_version": "ball_pointgoal_benchmark_v1",
        "benchmark_version": str(config["benchmark_version"]),
        "action_protocol_id": str(config["action_protocol_id"]),
        "localization_source": str(config["localization_source"]),
        "generation_seed": int(seed),
        "episodes": episodes,
    }
    _atomic_write_json(Path(output_path), payload)
    return payload


def _export_scene_with_isaac(args: argparse.Namespace) -> int:
    """Launch Isaac only for a scene-to-occupancy export."""

    scene_root_path = Path(args.scene_root).expanduser().resolve()
    scene_path = _resolve_root_uri(
        args.scene_uri,
        prefix=SCENE_ROOT_PREFIX,
        root=scene_root_path,
        field="scene_uri",
    )
    if not scene_path.is_file():
        raise FileNotFoundError(f"scene does not exist: {scene_path}")

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})
    try:
        import omni.usd
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.prims import XFormPrim
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from pxr import Usd, UsdGeom, UsdPhysics

        from scripts.bev_ground_truth import build_analytic_global_map
        from scripts.collect_data_dubins import ROBOT_ROOT, ROBOT_START, ROBOT_USD
        from scripts.collection_schema import load_contract

        contract = load_contract(BALL_WM_ROOT / "configs" / "data_contract.yaml")
        benchmark_config = _load_config()
        world = World(stage_units_in_meters=1.0, physics_dt=0.02, rendering_dt=0.1)
        world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(scene_path), prim_path="/World/Scene")
        XFormPrim("/World/Scene").set_local_pose(translation=np.zeros(3))
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        world.scene.add(
            Articulation(prim_path=ROBOT_ROOT, name="ball_robot", position=ROBOT_START)
        )
        world.reset()
        for _ in range(10):
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()
        occupancy = build_analytic_global_map(
            stage=stage,
            scene_root="/World/Scene",
            robot_root=ROBOT_ROOT,
            config=contract.bev,
            body_radius_m=float(benchmark_config["ball_radius_m"]),
            usd=Usd,
            usd_geom=UsdGeom,
            usd_physics=UsdPhysics,
        )
        output = Path(args.out)
        if output.suffix != ".npz":
            raise ValueError("export output must use the .npz suffix")
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(".partial.npz")
        np.savez(
            temporary,
            grid=occupancy.grid,
            global_origin_xy=occupancy.global_origin_xy,
            resolution_m=np.asarray(occupancy.resolution_m, dtype=np.float64),
            scene_uri=np.asarray(args.scene_uri),
            scene_sha256=np.asarray(_sha256_file(scene_path)),
            projection=np.asarray("vertical_max_body_band"),
            body_radius_m=np.asarray(float(benchmark_config["ball_radius_m"])),
        )
        temporary.replace(output)
        print(f"wrote occupancy map: {output}")
        return 0
    finally:
        simulation_app.close()


def _build_corridor_mask(args: argparse.Namespace) -> int:
    occupancy = load_occupancy_npz(args.occupancy)
    planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
    mask = build_two_sided_corridor_mask(
        planning_grid,
        resolution_m=occupancy.resolution_m,
        minimum_width_m=args.min_width_m,
        maximum_width_m=args.max_width_m,
    )
    output = Path(args.out)
    if output.suffix != ".npy":
        raise ValueError("corridor-mask output must use the .npy suffix")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".partial.npy")
    np.save(temporary, mask, allow_pickle=False)
    temporary.replace(output)
    print(f"wrote two-sided corridor mask ({int(mask.sum())} cells): {output}")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    export = subparsers.add_parser("export-scene")
    export.add_argument("--scene-uri", required=True)
    export.add_argument("--scene-root", type=Path, required=True)
    export.add_argument("--out", type=Path, required=True)

    corridor = subparsers.add_parser("build-corridor-mask")
    corridor.add_argument("--occupancy", type=Path, required=True)
    corridor.add_argument("--min-width-m", type=float, required=True)
    corridor.add_argument("--max-width-m", type=float, required=True)
    corridor.add_argument("--out", type=Path, required=True)

    generate = subparsers.add_parser("generate")
    generate.add_argument("--scene-index", type=Path, required=True)
    generate.add_argument("--split", choices=("golden", "dev"), required=True)
    generate.add_argument("--count-per-class", type=int, required=True)
    generate.add_argument("--seed", type=int, default=3072)
    generate.add_argument("--out", type=Path, required=True)

    fixed = subparsers.add_parser("generate-fixed")
    fixed.add_argument("--scene-index", type=Path, required=True)
    fixed.add_argument("--scene-id", required=True)
    fixed.add_argument("--split", choices=("golden", "dev"), required=True)
    fixed.add_argument("--difficulty", choices=("E0", "E1", "E2"), required=True)
    fixed.add_argument("--start-x", type=float, required=True)
    fixed.add_argument("--start-y", type=float, required=True)
    fixed.add_argument("--goal-x", type=float, required=True)
    fixed.add_argument("--goal-y", type=float, required=True)
    fixed.add_argument("--episode-id", required=True)
    fixed.add_argument("--seed", type=int, default=3072)
    fixed.add_argument("--out", type=Path, required=True)

    render = subparsers.add_parser("render")
    render.add_argument("--candidates", type=Path, required=True)
    render.add_argument("--cards-dir", type=Path, required=True)

    review = subparsers.add_parser("review")
    review.add_argument("--candidates", type=Path, required=True)
    review.add_argument("--episode-id", required=True)
    review.add_argument(
        "--status",
        choices=("approved", "rejected", "needs_adjustment"),
        required=True,
    )
    review.add_argument("--reviewer", required=True)
    review.add_argument("--notes", default="")

    freeze = subparsers.add_parser("freeze")
    freeze.add_argument("--candidates", type=Path, required=True)
    freeze.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "export-scene":
            return _export_scene_with_isaac(args)
        if args.command == "build-corridor-mask":
            return _build_corridor_mask(args)
        if args.command == "generate":
            payload = generate_candidates(
                args.scene_index,
                split=args.split,
                count_per_class=args.count_per_class,
                seed=args.seed,
                output_path=args.out,
            )
            print(f"generated {len(payload['episodes'])} candidates: {args.out}")
            return 0
        if args.command == "generate-fixed":
            generate_fixed_candidate(
                args.scene_index,
                scene_id=args.scene_id,
                split=args.split,
                difficulty=args.difficulty,
                start_xy_m=(args.start_x, args.start_y),
                goal_xy_m=(args.goal_x, args.goal_y),
                episode_id=args.episode_id,
                seed=args.seed,
                output_path=args.out,
            )
            print(f"generated fixed candidate: {args.out}")
            return 0
        if args.command == "render":
            rendered = render_candidates(
                args.candidates,
                args.cards_dir,
                benchmark_config=_load_config(),
            )
            print(f"rendered {len(rendered)} task cards: {args.cards_dir}")
            return 0
        if args.command == "review":
            review_candidates(
                args.candidates,
                episode_id=args.episode_id,
                status=args.status,
                reviewer=args.reviewer,
                notes=args.notes,
            )
            print(f"updated review: {args.episode_id} -> {args.status}")
            return 0
        if args.command == "freeze":
            frozen = freeze_candidates(
                args.candidates,
                args.out,
                benchmark_config=_load_config(),
            )
            print(f"froze {len(frozen.episodes)} approved tasks: {args.out}")
            return 0
    except (FileNotFoundError, KeyError, RuntimeError, TypeError, ValueError) as error:
        print(f"benchmark task command failed: {error}", file=sys.stderr)
        return 2
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
