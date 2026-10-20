"""Human-review task cards and candidate approval/freeze workflow."""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Literal, Mapping, Sequence

import numpy as np

from scripts.bev_ground_truth import GlobalOccupancy
from scripts.trajectory_map import (
    TASK_CARD_LAYERS,
    NavigationTaskOverlay,
    render_navigation_task_map,
)

from .grid_planning import astar_path, inflate_obstacles
from .manifest import (
    BenchmarkEpisode,
    BenchmarkManifest,
    HumanReview,
    freeze_manifest,
)
from .task_generation import Difficulty, TaskEvidence, validate_task


CANDIDATE_SCHEMA_VERSION = "ball_pointgoal_candidates_v1"
EXPECTED_TASK_CARD_LAYERS = TASK_CARD_LAYERS


@dataclass(frozen=True)
class TaskCard:
    episode_id: str
    difficulty: Difficulty
    occupancy: GlobalOccupancy
    inflated_grid: np.ndarray
    start_cell: tuple[int, int]
    start_yaw_rad: float
    goal_cell: tuple[int, int]
    oracle_path_cells: np.ndarray
    evidence: TaskEvidence

    def __post_init__(self) -> None:
        if not isinstance(self.episode_id, str) or not self.episode_id.strip():
            raise ValueError("episode_id must be a non-empty string")
        if self.difficulty not in ("E0", "E1", "E2"):
            raise ValueError(f"unsupported difficulty: {self.difficulty!r}")
        if not isinstance(self.occupancy, GlobalOccupancy):
            raise TypeError("occupancy must be a GlobalOccupancy")
        inflated = np.asarray(self.inflated_grid, dtype=np.uint8)
        if inflated.shape != self.occupancy.grid.shape:
            raise ValueError("inflated_grid shape must match occupancy grid")
        path = np.asarray(self.oracle_path_cells)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] == 0:
            raise ValueError("oracle_path_cells must have shape (N, 2)")
        if not isinstance(self.evidence, TaskEvidence):
            raise TypeError("evidence must be TaskEvidence")
        object.__setattr__(self, "inflated_grid", inflated.copy())
        object.__setattr__(self, "oracle_path_cells", path.astype(np.int64, copy=True))

    @classmethod
    def from_grid(
        cls,
        *,
        episode_id: str,
        difficulty: Difficulty,
        grid: np.ndarray,
        global_origin_xy: Sequence[float],
        resolution_m: float,
        start_cell: Sequence[int],
        start_yaw_rad: float,
        goal_cell: Sequence[int],
        evidence: TaskEvidence,
        ball_radius_m: float = 0.4,
        safety_margin_m: float = 0.2,
    ) -> "TaskCard":
        occupancy = GlobalOccupancy(
            grid=np.asarray(grid, dtype=np.uint8),
            global_origin_xy=np.asarray(global_origin_xy, dtype=np.float64),
            resolution_m=float(resolution_m),
        )
        planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
        inflated = inflate_obstacles(
            planning_grid,
            radius_m=ball_radius_m + safety_margin_m,
            resolution_m=occupancy.resolution_m,
            occupied=1,
        )
        start = tuple(int(value) for value in start_cell)
        goal = tuple(int(value) for value in goal_cell)
        path = astar_path(inflated, start=start, goal=goal, occupied=1)
        return cls(
            episode_id=episode_id,
            difficulty=difficulty,
            occupancy=occupancy,
            inflated_grid=inflated,
            start_cell=start,
            start_yaw_rad=float(start_yaw_rad),
            goal_cell=goal,
            oracle_path_cells=path,
            evidence=evidence,
        )


@dataclass(frozen=True)
class RenderedTaskCard:
    path: Path
    layers: Mapping[str, bool]


def render_task_card(output_path: str | Path, task: TaskCard) -> RenderedTaskCard:
    """Render a task card with all mandatory audit layers."""

    if not isinstance(task, TaskCard):
        raise TypeError("task must be a TaskCard")
    evidence = task.evidence
    metrics = (
        f"difficulty: {task.difficulty}",
        f"line free: {evidence.line_is_free}",
        f"euclidean: {evidence.euclidean_distance_m:.3f} m",
        f"A*: {evidence.oracle_path_length_m:.3f} m",
        f"detour ratio: {evidence.detour_ratio:.3f}",
        f"start clearance: {evidence.start_clearance_m:.3f} m",
        f"goal clearance: {evidence.goal_clearance_m:.3f} m",
        f"nearby obstacles: {evidence.nearby_obstacle_count}",
        f"corridor: {evidence.corridor_width_m if evidence.corridor_width_m is not None else 'n/a'}",
        f"corridor path: {evidence.corridor_path_length_m:.3f} m",
    )
    overlay = NavigationTaskOverlay(
        inflated_grid=task.inflated_grid,
        start_cell=task.start_cell,
        start_yaw_rad=task.start_yaw_rad,
        goal_cell=task.goal_cell,
        oracle_path_cells=task.oracle_path_cells,
        metrics=metrics,
    )
    path, layers = render_navigation_task_map(
        output_path,
        task.occupancy,
        overlay,
        title=f"PointGoal task {task.episode_id}",
    )
    if set(layers) != EXPECTED_TASK_CARD_LAYERS or not all(layers.values()):
        raise RuntimeError("task-card renderer omitted a mandatory review layer")
    return RenderedTaskCard(path=path, layers=MappingProxyType(dict(layers)))


def _load_json_object(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _candidate_episodes(payload: Mapping[str, object]) -> list[dict[str, object]]:
    if payload.get("schema_version") != CANDIDATE_SCHEMA_VERSION:
        raise ValueError(f"unsupported candidate schema: {payload.get('schema_version')!r}")
    episodes = payload.get("episodes")
    if not isinstance(episodes, list):
        raise ValueError("candidate episodes must be an array")
    if any(not isinstance(episode, dict) for episode in episodes):
        raise ValueError("each candidate episode must be an object")
    return episodes  # type: ignore[return-value]


def _atomic_write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def review_candidates(
    candidates_path: str | Path,
    *,
    episode_id: str,
    status: Literal["approved", "rejected", "needs_adjustment"],
    reviewer: str,
    notes: str,
) -> None:
    """Atomically update one review while retaining every candidate record."""

    review = HumanReview(status=status, reviewer=reviewer, notes=notes)
    path = Path(candidates_path)
    payload = _load_json_object(path)
    episodes = _candidate_episodes(payload)
    matches = [episode for episode in episodes if episode.get("episode_id") == episode_id]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one candidate with episode_id {episode_id!r}")
    matches[0]["human_review"] = review.to_dict()
    _atomic_write_json(path, payload)


def _resolve_data_uri(
    uri: object,
    *,
    data_root: str | Path | None,
    relative_to: Path,
) -> Path:
    if not isinstance(uri, str) or not uri:
        raise ValueError("occupancy_uri must be a non-empty string")
    prefix = "${DATA_ROOT}/"
    if uri.startswith(prefix):
        if data_root is None:
            raise ValueError("DATA_ROOT is required to resolve occupancy_uri")
        root = Path(data_root).expanduser().resolve()
        relative = Path(uri[len(prefix) :])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("occupancy_uri escapes DATA_ROOT")
        resolved = root.joinpath(relative).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as error:
            raise ValueError("occupancy_uri escapes DATA_ROOT") from error
        return resolved
    path = Path(uri).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def load_occupancy_npz(path: str | Path) -> GlobalOccupancy:
    occupancy_path = Path(path)
    with np.load(occupancy_path, allow_pickle=False) as archive:
        required = {"grid", "global_origin_xy", "resolution_m"}
        missing = required.difference(archive.files)
        if missing:
            raise ValueError(f"occupancy NPZ is missing: {', '.join(sorted(missing))}")
        grid = np.asarray(archive["grid"], dtype=np.uint8)
        origin = np.asarray(archive["global_origin_xy"], dtype=np.float64)
        resolution_values = np.asarray(archive["resolution_m"], dtype=np.float64)
    if resolution_values.size != 1:
        raise ValueError("occupancy resolution_m must be scalar")
    if np.any(~np.isin(grid, (0, 1, 2))):
        raise ValueError("occupancy grid must use labels 0, 1, and 2")
    return GlobalOccupancy(
        grid=grid,
        global_origin_xy=origin,
        resolution_m=float(resolution_values.reshape(-1)[0]),
    )


def _world_position_to_cell(position: Sequence[float], occupancy: GlobalOccupancy) -> tuple[int, int]:
    world_xy = np.asarray(position, dtype=np.float64)[:2]
    continuous = (world_xy - occupancy.global_origin_xy) / occupancy.resolution_m
    rounded = np.rint(continuous)
    if not np.allclose(continuous, rounded, atol=1e-5, rtol=0.0):
        raise ValueError("benchmark position must align with an occupancy cell center")
    return int(rounded[0]), int(rounded[1])


def _yaw_from_wxyz(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _corridor_inputs(
    episode_payload: Mapping[str, object],
    *,
    data_root: str | Path | None,
    relative_to: Path,
) -> tuple[np.ndarray | None, float | None]:
    mask_uri = episode_payload.get("corridor_mask_uri")
    width = episode_payload.get("corridor_width_m")
    if mask_uri is None:
        return None, None if width is None else float(width)
    mask_path = _resolve_data_uri(mask_uri, data_root=data_root, relative_to=relative_to)
    mask = np.asarray(np.load(mask_path, allow_pickle=False), dtype=bool)
    return mask, None if width is None else float(width)


def _minimum_nearby_obstacles(
    episode_payload: Mapping[str, object],
    *,
    benchmark_config: Mapping[str, object] | None,
) -> int:
    """Use a candidate's recorded validation threshold when it is present."""

    value = episode_payload.get(
        "minimum_nearby_obstacles",
        (benchmark_config or {}).get("minimum_nearby_obstacles", 0),
    )
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("minimum_nearby_obstacles must be a non-negative integer")
    return value


def _validated_episode_and_card(
    episode_payload: Mapping[str, object],
    *,
    candidates_dir: Path,
    data_root: str | Path | None,
    benchmark_config: Mapping[str, object] | None = None,
) -> tuple[BenchmarkEpisode, TaskCard, Path]:
    episode = BenchmarkEpisode.from_dict(episode_payload)
    occupancy_path = _resolve_data_uri(
        episode_payload.get("occupancy_uri"),
        data_root=data_root,
        relative_to=candidates_dir,
    )
    episode.verify_occupancy(occupancy_path)
    occupancy = load_occupancy_npz(occupancy_path)
    planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
    start_cell = _world_position_to_cell(episode.start_position_world_m, occupancy)
    goal_cell = _world_position_to_cell(episode.goal_position_world_m, occupancy)
    corridor_mask, corridor_width = _corridor_inputs(
        episode_payload,
        data_root=data_root,
        relative_to=candidates_dir,
    )
    evidence = validate_task(
        planning_grid,
        start=start_cell,
        goal=goal_cell,
        difficulty=episode.difficulty,
        resolution_m=occupancy.resolution_m,
        ball_radius_m=float((benchmark_config or {}).get("ball_radius_m", 0.4)),
        safety_margin_m=float((benchmark_config or {}).get("safety_margin_m", 0.2)),
        e1_min_detour_ratio=float(
            (benchmark_config or {}).get("e1_min_detour_ratio", 1.15)
        ),
        corridor_mask=corridor_mask,
        corridor_width_m=corridor_width,
        e2_extra_width_m=float(
            (benchmark_config or {}).get("e2_extra_width_range_m", [0.4, 0.4])[1]
        ),
        nearby_obstacle_radius_m=float(
            (benchmark_config or {}).get("nearby_obstacle_radius_m", 2.0)
        ),
        minimum_nearby_obstacles=_minimum_nearby_obstacles(
            episode_payload,
            benchmark_config=benchmark_config,
        ),
        minimum_corridor_path_length_m=float(
            (benchmark_config or {}).get("minimum_corridor_path_length_m", 0.0)
        ),
    )
    card = TaskCard.from_grid(
        episode_id=episode.episode_id,
        difficulty=episode.difficulty,
        grid=occupancy.grid,
        global_origin_xy=occupancy.global_origin_xy,
        resolution_m=occupancy.resolution_m,
        start_cell=start_cell,
        start_yaw_rad=_yaw_from_wxyz(episode.start_quaternion_world_wxyz),
        goal_cell=goal_cell,
        evidence=evidence,
        ball_radius_m=float((benchmark_config or {}).get("ball_radius_m", 0.4)),
        safety_margin_m=float((benchmark_config or {}).get("safety_margin_m", 0.2)),
    )
    return episode, card, occupancy_path


def render_candidates(
    candidates_path: str | Path,
    cards_dir: str | Path,
    *,
    data_root: str | Path | None = None,
    benchmark_config: Mapping[str, object] | None = None,
) -> tuple[RenderedTaskCard, ...]:
    path = Path(candidates_path)
    payload = _load_json_object(path)
    episodes = _candidate_episodes(payload)
    root = data_root if data_root is not None else os.environ.get("DATA_ROOT")
    rendered: list[RenderedTaskCard] = []
    for episode_payload in episodes:
        _, card, _ = _validated_episode_and_card(
            episode_payload,
            candidates_dir=path.parent.resolve(),
            data_root=root,
            benchmark_config=benchmark_config,
        )
        rendered.append(
            render_task_card(Path(cards_dir) / f"{card.episode_id}.png", card)
        )
    return tuple(rendered)


def freeze_candidates(
    candidates_path: str | Path,
    output_path: str | Path,
    *,
    scene_root: str | Path | None = None,
    data_root: str | Path | None = None,
    benchmark_config: Mapping[str, object] | None = None,
) -> BenchmarkManifest:
    """Revalidate approved candidates and freeze only portable episode fields."""

    path = Path(candidates_path)
    payload = _load_json_object(path)
    episode_payloads = _candidate_episodes(payload)
    unapproved = [
        str(episode.get("episode_id", "<missing>"))
        for episode in episode_payloads
        if not isinstance(episode.get("human_review"), Mapping)
        or episode["human_review"].get("status") != "approved"  # type: ignore[union-attr]
    ]
    if unapproved:
        raise ValueError(
            "all candidate tasks must be approved before freeze; "
            f"not approved: {', '.join(unapproved)}"
        )

    resolved_scene_root = scene_root if scene_root is not None else os.environ.get("SCENE_ROOT")
    resolved_data_root = data_root if data_root is not None else os.environ.get("DATA_ROOT")
    if resolved_scene_root is None:
        raise ValueError("SCENE_ROOT is required to verify approved benchmark scenes")

    episodes: list[BenchmarkEpisode] = []
    for episode_payload in episode_payloads:
        episode, card, _ = _validated_episode_and_card(
            episode_payload,
            candidates_dir=path.parent.resolve(),
            data_root=resolved_data_root,
            benchmark_config=benchmark_config,
        )
        episode.verify_scene(resolved_scene_root)
        evidence = card.evidence
        episodes.append(
            replace(
                episode,
                euclidean_distance_m=evidence.euclidean_distance_m,
                oracle_path_length_m=evidence.oracle_path_length_m,
                classification_evidence=asdict(evidence),
            )
        )

    manifest = BenchmarkManifest(
        schema_version=str(payload.get("manifest_schema_version", "ball_pointgoal_benchmark_v1")),
        benchmark_version=str(payload.get("benchmark_version", "")),
        action_protocol_id=str(payload.get("action_protocol_id", "")),
        localization_source=str(payload.get("localization_source", "")),
        episodes=tuple(episodes),
    )
    return freeze_manifest(manifest, output_path)
