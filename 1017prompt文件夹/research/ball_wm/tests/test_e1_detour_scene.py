"""Regression guards for the version-controlled E1 detour scene."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from benchmark.grid_planning import astar_path, inflate_obstacles, line_is_free
from benchmark.task_cards import load_occupancy_npz
from benchmark.task_generation import validate_task


ROOT = Path(__file__).resolve().parents[1]
SOURCE_SCENE = ROOT / "data/benchmark_v1/scenes/e1_detour.usda"
SCENE_INDEX = ROOT / "configs/benchmark_v1/scene_index.json"
OCCUPANCY = ROOT / "data/benchmark_v1/occupancy/e1_detour_bodyband.npz"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_e1_detour_source_uses_collision_cylinders_with_double_dimensions():
    """Collision geometry must match the cylinder dimensions rendered by USD."""

    source = SOURCE_SCENE.read_text(encoding="utf-8")

    assert "def Cylinder" in source
    assert "def Sphere" not in source
    assert "float radius" not in source
    assert "float height" not in source
    assert source.count("double radius = 0.35") == 71
    assert source.count("double height = 1.0") == 71
    assert source.count('prepend apiSchemas = ["PhysicsCollisionAPI"]') == 71
    for name in (
        "BoundaryLeft",
        "BoundaryRight",
        "BoundaryBottom",
        "BoundaryTop",
        "Divider",
    ):
        assert f'def Xform "{name}"' in source


def test_e1_detour_frozen_map_is_a_valid_lower_only_e1_detour():
    """The registered E1 task must require the designed lower detour."""

    index = json.loads(SCENE_INDEX.read_text(encoding="utf-8"))
    scene = next(item for item in index["scenes"] if item["scene_id"] == "e1_detour")
    assert scene["scene_uri"] == "${SCENE_ROOT}/benchmark_e1/e1_detour.usda"
    assert scene["occupancy_uri"] == "${DATA_ROOT}/benchmark_v1/occupancy/e1_detour_bodyband.npz"
    assert scene["occupancy_projection"] == "vertical_max_body_band"
    assert scene["body_radius_m"] == pytest.approx(0.4)
    assert scene["navigation_z_m"] == pytest.approx(0.45)
    assert scene["minimum_nearby_obstacles"] == 0
    assert scene["difficulties"] == ["E1"]
    assert scene["scene_sha256"] == _sha256(SOURCE_SCENE)
    assert scene["occupancy_sha256"] == _sha256(OCCUPANCY)

    with np.load(OCCUPANCY, allow_pickle=False) as archive:
        assert archive["scene_uri"].item() == scene["scene_uri"]
        assert archive["scene_sha256"].item() == scene["scene_sha256"]
        assert archive["projection"].item() == scene["occupancy_projection"]
        assert float(archive["body_radius_m"].item()) == pytest.approx(
            scene["body_radius_m"]
        )

    occupancy = load_occupancy_npz(OCCUPANCY)
    planning = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
    start = tuple(
        np.rint(
            (np.asarray((-3.3, 0.0)) - occupancy.global_origin_xy)
            / occupancy.resolution_m
        ).astype(int)
    )
    goal = tuple(
        np.rint(
            (np.asarray((3.3, 0.0)) - occupancy.global_origin_xy)
            / occupancy.resolution_m
        ).astype(int)
    )
    evidence = validate_task(
        planning,
        start=start,
        goal=goal,
        difficulty="E1",
        resolution_m=occupancy.resolution_m,
        ball_radius_m=0.4,
        safety_margin_m=0.1,
        e1_min_detour_ratio=1.3,
        nearby_obstacle_radius_m=2.0,
        minimum_nearby_obstacles=int(scene["minimum_nearby_obstacles"]),
    )
    assert evidence.line_is_free is False
    assert evidence.detour_ratio >= 1.3

    inflated = inflate_obstacles(
        planning,
        radius_m=0.5,
        resolution_m=occupancy.resolution_m,
    )
    path = astar_path(inflated, start=start, goal=goal)
    path_y = occupancy.global_origin_xy[1] + path[:, 1] * occupancy.resolution_m
    assert path_y.min() <= -2.5
    blocked_lower = inflated.copy()
    lower_columns = (
        occupancy.global_origin_xy[1]
        + np.arange(inflated.shape[1]) * occupancy.resolution_m
        <= -2.5
    )
    blocked_lower[:, lower_columns] = 1
    with pytest.raises(ValueError, match="unreachable"):
        astar_path(blocked_lower, start=start, goal=goal)
    assert line_is_free(planning, start=start, goal=goal) is False


def test_e1_detour_golden_manifest_preserves_the_approved_candidate():
    """The reviewed candidate and immutable Golden manifest must agree."""

    candidate_path = ROOT / "configs/benchmark_v1/e1_detour_golden_candidate.json"
    manifest_path = ROOT / "configs/benchmark_v1/e1_detour_golden_manifest.json"
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert candidate["schema_version"] == "ball_pointgoal_candidates_v1"
    assert len(candidate["episodes"]) == 1
    reviewed = candidate["episodes"][0]
    assert reviewed["episode_id"] == "e1_detour_golden_0001"
    assert reviewed["human_review"]["status"] == "approved"
    assert reviewed["classification_evidence"]["line_is_free"] is False
    assert reviewed["classification_evidence"]["detour_ratio"] >= 1.3

    assert len(manifest["episodes"]) == 1
    frozen = manifest["episodes"][0]
    assert frozen["episode_id"] == reviewed["episode_id"]
    assert frozen["scene_sha256"] == reviewed["scene_sha256"]
    assert frozen["occupancy_sha256"] == reviewed["occupancy_sha256"]
    assert manifest["manifest_sha256"]
