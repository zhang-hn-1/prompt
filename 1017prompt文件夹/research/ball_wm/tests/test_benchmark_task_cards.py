"""Tests for task-card rendering and the human review/freeze workflow."""

from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from benchmark import task_cards as task_cards_module
from benchmark.manifest import load_manifest
from benchmark.task_cards import (
    EXPECTED_TASK_CARD_LAYERS,
    TaskCard,
    freeze_candidates,
    render_candidates,
    render_task_card,
    review_candidates,
)
from benchmark.task_generation import validate_task
from scripts import generate_benchmark_tasks as generate_benchmark_tasks_module
from scripts.generate_benchmark_tasks import _load_config, generate_candidates, main


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _open_task_card() -> TaskCard:
    grid = np.zeros((90, 90), dtype=np.uint8)
    grid[20:70, 45] = 1
    start = (45, 10)
    goal = (45, 80)
    evidence = validate_task(
        grid,
        start=start,
        goal=goal,
        difficulty="E1",
        resolution_m=0.1,
    )
    return TaskCard.from_grid(
        episode_id="scene_e1_0001",
        difficulty="E1",
        grid=grid,
        global_origin_xy=(0.0, 0.0),
        resolution_m=0.1,
        start_cell=start,
        start_yaw_rad=0.0,
        goal_cell=goal,
        evidence=evidence,
    )


def _candidate_payload(
    root: Path,
    *,
    statuses: tuple[str, ...] = ("needs_adjustment",),
) -> tuple[dict[str, object], Path, Path]:
    scene_root = root / "scenes"
    data_root = root / "data"
    scene_root.mkdir()
    data_root.mkdir()
    scene = scene_root / "scene_01.usd"
    scene.write_bytes(b"scene-v1")
    occupancy = data_root / "scene_01.npz"
    grid = np.zeros((90, 90), dtype=np.uint8)
    np.savez(
        occupancy,
        grid=grid,
        global_origin_xy=np.asarray((0.0, 0.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )

    episodes: list[dict[str, object]] = []
    for index, status in enumerate(statuses, start=1):
        episodes.append(
            {
                "schema_version": "benchmark_episode_v1",
                "episode_id": f"easy_e0_{index:04d}",
                "split": "golden",
                "difficulty": "E0",
                "scene_uri": "${SCENE_ROOT}/scene_01.usd",
                "scene_sha256": _sha256(scene),
                "occupancy_sha256": _sha256(occupancy),
                "occupancy_uri": "${DATA_ROOT}/scene_01.npz",
                "start_position_world_m": [4.5, 1.0 + index * 0.1, 0.45],
                "start_quaternion_world_wxyz": [1.0, 0.0, 0.0, 0.0],
                "goal_position_world_m": [4.5, 4.0 + index * 0.1, 0.45],
                "euclidean_distance_m": 3.0,
                "oracle_path_length_m": 3.0,
                "seed": 3072 + index,
                "human_review": {
                    "status": status,
                    "reviewer": "project_owner",
                    "notes": "review fixture",
                },
                "classification_evidence": {
                    "line_is_free": True,
                    "detour_ratio": 1.0,
                },
            }
        )
    payload: dict[str, object] = {
        "schema_version": "ball_pointgoal_candidates_v1",
        "benchmark_version": "v1",
        "action_protocol_id": "ball_action_v2_aux30",
        "localization_source": "perfect_localization",
        "episodes": episodes,
    }
    return payload, scene_root, data_root


def _write_candidates(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def test_task_card_contains_every_required_review_layer(tmp_path: Path):
    output = tmp_path / "task_card.png"

    rendered = render_task_card(output, _open_task_card())

    assert rendered.path == output
    assert set(rendered.layers) == EXPECTED_TASK_CARD_LAYERS
    assert all(rendered.layers.values())
    with Image.open(output) as image:
        assert image.size == (1200, 900)
        assert image.mode == "RGB"
        assert len(np.unique(np.asarray(image).reshape(-1, 3), axis=0)) > 8


def test_task_card_displays_obstacle_and_corridor_path_metrics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    captured_metrics: tuple[str, ...] = ()

    def fake_render(output_path, occupancy, overlay, *, title):
        nonlocal captured_metrics
        captured_metrics = tuple(overlay.metrics)
        return Path(output_path), dict.fromkeys(EXPECTED_TASK_CARD_LAYERS, True)

    monkeypatch.setattr(task_cards_module, "render_navigation_task_map", fake_render)

    render_task_card(tmp_path / "task_card.png", _open_task_card())

    assert any(line.startswith("nearby obstacles:") for line in captured_metrics)
    assert any(line.startswith("corridor path:") for line in captured_metrics)


def test_review_updates_one_episode_atomically_and_preserves_rejected_records(
    tmp_path: Path,
):
    payload, _, _ = _candidate_payload(
        tmp_path,
        statuses=("needs_adjustment", "rejected"),
    )
    candidates = tmp_path / "candidates.json"
    _write_candidates(candidates, payload)

    review_candidates(
        candidates,
        episode_id="easy_e0_0001",
        status="approved",
        reviewer="project_owner",
        notes="GUI path and goal checked",
    )

    updated = json.loads(candidates.read_text(encoding="utf-8"))
    assert updated["episodes"][0]["human_review"] == {
        "status": "approved",
        "reviewer": "project_owner",
        "notes": "GUI path and goal checked",
    }
    assert updated["episodes"][1]["human_review"]["status"] == "rejected"
    assert not candidates.with_suffix(".json.partial").exists()


def test_freeze_cli_rejects_unapproved_candidates_without_output(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
):
    payload, _, _ = _candidate_payload(tmp_path)
    candidates = tmp_path / "candidates.json"
    output = tmp_path / "manifest.json"
    _write_candidates(candidates, payload)

    result = main(["freeze", "--candidates", str(candidates), "--out", str(output)])

    assert result == 2
    assert "approved" in capsys.readouterr().err
    assert not output.exists()


def test_approved_candidates_freeze_after_scene_and_occupancy_verification(
    tmp_path: Path,
):
    payload, scene_root, data_root = _candidate_payload(
        tmp_path,
        statuses=("approved",),
    )
    candidates = tmp_path / "candidates.json"
    output = tmp_path / "manifest.json"
    _write_candidates(candidates, payload)

    frozen = freeze_candidates(
        candidates,
        output,
        scene_root=scene_root,
        data_root=data_root,
    )

    assert load_manifest(output) == frozen
    episode = frozen.episodes[0]
    assert episode.human_review.status == "approved"
    assert episode.classification_evidence["line_is_free"] is True
    assert episode.classification_evidence["detour_ratio"] == pytest.approx(1.0)


def test_render_cli_creates_one_named_card_per_candidate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    payload, _, data_root = _candidate_payload(tmp_path)
    candidates = tmp_path / "candidates.json"
    cards = tmp_path / "cards"
    _write_candidates(candidates, payload)
    monkeypatch.setenv("DATA_ROOT", str(data_root))
    relaxed_config = dict(_load_config())
    relaxed_config["minimum_nearby_obstacles"] = 0
    monkeypatch.setattr(
        generate_benchmark_tasks_module,
        "_load_config",
        lambda: relaxed_config,
    )

    result = main(
        ["render", "--candidates", str(candidates), "--cards-dir", str(cards)]
    )

    assert result == 0
    assert (cards / "easy_e0_0001.png").is_file()


def test_export_scene_dispatch_is_lazy_and_writes_no_file_itself(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    from scripts import generate_benchmark_tasks

    calls: list[object] = []
    monkeypatch.setattr(
        generate_benchmark_tasks,
        "_export_scene_with_isaac",
        lambda args: calls.append(args) or 0,
    )

    result = main(
        [
            "export-scene",
            "--scene-uri",
            "${SCENE_ROOT}/scene.usd",
            "--scene-root",
            str(tmp_path),
            "--out",
            str(tmp_path / "occupancy.npz"),
        ]
    )

    assert result == 0
    assert len(calls) == 1


def test_build_corridor_mask_cli_writes_two_sided_annotation(tmp_path: Path):
    occupancy = tmp_path / "occupancy.npz"
    output = tmp_path / "corridor.npy"
    grid = np.zeros((80, 80), dtype=np.uint8)
    grid[25, 10:70] = 1
    grid[39, 10:70] = 1
    np.savez(
        occupancy,
        grid=grid,
        global_origin_xy=np.asarray((0.0, 0.0)),
        resolution_m=np.asarray(0.1),
    )

    result = main(
        [
            "build-corridor-mask",
            "--occupancy",
            str(occupancy),
            "--min-width-m",
            "1.2",
            "--max-width-m",
            "1.6",
            "--out",
            str(output),
        ]
    )

    assert result == 0
    mask = np.load(output, allow_pickle=False)
    assert mask.dtype == np.bool_
    assert mask[32, 20:60].all()


def test_benchmark_generation_config_requires_challenging_golden_tasks():
    config = _load_config()

    assert config["initial_goal_distance_m"] == [6.0, 10.0]
    assert config["safety_margin_m"] == pytest.approx(0.1)
    assert config["e1_min_detour_ratio"] == pytest.approx(1.3)
    assert config["nearby_obstacle_radius_m"] == pytest.approx(2.0)
    assert config["minimum_nearby_obstacles"] == 3
    assert config["minimum_corridor_path_length_m"] == pytest.approx(0.8)


def test_e2_conditioned_sampler_places_endpoints_on_opposite_corridor_sides():
    assert hasattr(generate_benchmark_tasks_module, "_sample_corridor_conditioned_pair")
    assert "minimum_corridor_path_length_m" in inspect.signature(
        generate_benchmark_tasks_module._sample_corridor_conditioned_pair
    ).parameters
    inflated = np.zeros((120, 120), dtype=np.uint8)
    corridor = np.zeros_like(inflated, dtype=bool)
    corridor[50:71, 60] = True

    start, goal = generate_benchmark_tasks_module._sample_corridor_conditioned_pair(
        inflated,
        corridor,
        minimum_distance_m=6.0,
        maximum_distance_m=10.0,
        minimum_corridor_path_length_m=0.8,
        resolution_m=0.1,
        rng=np.random.default_rng(7),
    )

    center = np.array([60.0, 60.0])
    assert (np.asarray(start) - center)[0] < 0.0
    assert (np.asarray(goal) - center)[0] > 0.0
    distance_m = np.linalg.norm(np.asarray(goal) - np.asarray(start)) * 0.1
    assert 6.0 <= distance_m <= 10.0


def test_generate_fixed_cli_writes_the_requested_single_e1_candidate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A fixed Golden task must use the requested scene and endpoints."""

    data_root = tmp_path / "data"
    data_root.mkdir()
    grid = np.zeros((100, 100), dtype=np.uint8)
    grid[10:90, 50] = 1
    occupancy = data_root / "e1.npz"
    np.savez(
        occupancy,
        grid=grid,
        global_origin_xy=np.asarray((0.0, 0.0)),
        resolution_m=np.asarray(0.1),
    )
    index = tmp_path / "scene_index.json"
    _write_candidates(
        index,
        {
            "scenes": [
                {
                    "scene_id": "fixed_e1",
                    "scene_uri": "${SCENE_ROOT}/fixed_e1.usda",
                    "scene_sha256": "ab" * 32,
                    "occupancy_uri": "${DATA_ROOT}/e1.npz",
                    "navigation_z_m": 0.45,
                    "difficulties": ["E1"],
                    "minimum_nearby_obstacles": 0,
                }
            ]
        },
    )
    monkeypatch.setenv("DATA_ROOT", str(data_root))
    relaxed_config = dict(_load_config())
    relaxed_config.update(
        {
            "initial_goal_distance_m": [6.0, 10.0],
            "e1_min_detour_ratio": 1.15,
            "minimum_nearby_obstacles": 3,
        }
    )
    monkeypatch.setattr(
        generate_benchmark_tasks_module,
        "_load_config",
        lambda: relaxed_config,
    )
    output = tmp_path / "fixed.json"

    assert main(
        [
            "generate-fixed",
            "--scene-index",
            str(index),
            "--scene-id",
            "fixed_e1",
            "--split",
            "golden",
            "--difficulty",
            "E1",
            "--start-x",
            "5.0",
            "--start-y",
            "2.0",
            "--goal-x",
            "5.0",
            "--goal-y",
            "8.0",
            "--episode-id",
            "fixed_e1_golden_0001",
            "--seed",
            "9101",
            "--out",
            str(output),
        ]
    ) == 0

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert len(payload["episodes"]) == 1
    episode = payload["episodes"][0]
    assert episode["episode_id"] == "fixed_e1_golden_0001"
    assert episode["difficulty"] == "E1"
    assert episode["scene_uri"] == "${SCENE_ROOT}/fixed_e1.usda"
    assert episode["start_position_world_m"] == [5.0, 2.0, 0.45]
    assert episode["goal_position_world_m"] == [5.0, 8.0, 0.45]
    assert episode["human_review"]["status"] == "needs_adjustment"
    assert episode["classification_evidence"]["line_is_free"] is False
    assert episode["minimum_nearby_obstacles"] == 0

    rendered = render_candidates(
        output,
        tmp_path / "cards",
        data_root=data_root,
        benchmark_config=relaxed_config,
    )
    assert [card.path.name for card in rendered] == ["fixed_e1_golden_0001.png"]


def test_generation_is_seeded_and_produces_one_valid_candidate_per_class(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    data_root = tmp_path / "data"
    data_root.mkdir()

    e0 = np.zeros((90, 90), dtype=np.uint8)
    e1 = np.zeros((90, 90), dtype=np.uint8)
    e1[10:80, 45] = 1
    e2 = np.zeros((100, 100), dtype=np.uint8)
    e2[:43, 50] = 1
    e2[57:, 50] = 1
    corridor = np.zeros_like(e2, dtype=bool)
    corridor[43:57, 45:56] = True
    for name, grid in (("e0", e0), ("e1", e1), ("e2", e2)):
        np.savez(
            data_root / f"{name}.npz",
            grid=grid,
            global_origin_xy=np.asarray((0.0, 0.0)),
            resolution_m=np.asarray(0.1),
        )
    np.save(data_root / "e2_corridor.npy", corridor)

    scene_index = {
        "scenes": [
            {
                "scene_uri": "${SCENE_ROOT}/e0.usd",
                "scene_sha256": "01" * 32,
                "occupancy_uri": "${DATA_ROOT}/e0.npz",
                "difficulties": ["E0"],
            },
            {
                "scene_uri": "${SCENE_ROOT}/e1.usd",
                "scene_sha256": "02" * 32,
                "occupancy_uri": "${DATA_ROOT}/e1.npz",
                "difficulties": ["E1"],
            },
            {
                "scene_uri": "${SCENE_ROOT}/e2.usd",
                "scene_sha256": "03" * 32,
                "occupancy_uri": "${DATA_ROOT}/e2.npz",
                "difficulties": ["E2"],
                "corridors": [
                    {
                        "mask_uri": "${DATA_ROOT}/e2_corridor.npy",
                        "width_m": 1.4,
                    }
                ],
            },
        ]
    }
    index_path = tmp_path / "scene_index.json"
    _write_candidates(index_path, scene_index)
    monkeypatch.setenv("DATA_ROOT", str(data_root))
    relaxed_config = dict(_load_config())
    relaxed_config.update(
        {
            "initial_goal_distance_m": [1.5, 4.0],
            "e1_min_detour_ratio": 1.15,
            "minimum_nearby_obstacles": 0,
            "minimum_corridor_path_length_m": 0.0,
        }
    )
    monkeypatch.setattr(
        generate_benchmark_tasks_module,
        "_load_config",
        lambda: relaxed_config,
    )

    first = generate_candidates(
        index_path,
        split="golden",
        count_per_class=1,
        seed=3072,
        output_path=tmp_path / "first.json",
    )
    second = generate_candidates(
        index_path,
        split="golden",
        count_per_class=1,
        seed=3072,
        output_path=tmp_path / "second.json",
    )

    assert first == second
    episodes = first["episodes"]
    assert [episode["difficulty"] for episode in episodes] == ["E0", "E1", "E2"]
    assert all(
        episode["human_review"]["status"] == "needs_adjustment"
        for episode in episodes
    )
