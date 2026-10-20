"""Pure-Python contract tests for immutable Benchmark V1 manifests."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
import yaml

from benchmark.manifest import (
    BenchmarkEpisode,
    BenchmarkManifest,
    HumanReview,
    freeze_manifest,
    load_manifest,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / "configs" / "benchmark_v1.yaml"
VALID_SCENE_SHA = "01" * 32
VALID_OCCUPANCY_SHA = "ab" * 32


def candidate_episode(**overrides: object) -> BenchmarkEpisode:
    values: dict[str, object] = {
        "episode_id": "easy_e1_0001",
        "split": "dev",
        "difficulty": "E1",
        "scene_uri": "${SCENE_ROOT}/benchmark_easy/scene_01.usd",
        "scene_sha256": VALID_SCENE_SHA,
        "occupancy_sha256": VALID_OCCUPANCY_SHA,
        "start_position_world_m": (0.0, 0.0, 0.45),
        "start_quaternion_world_wxyz": (1.0, 0.0, 0.0, 0.0),
        "goal_position_world_m": (3.0, 0.0, 0.45),
        "euclidean_distance_m": 3.0,
        "oracle_path_length_m": 3.6,
        "seed": 3072,
        "human_review": HumanReview(
            status="approved",
            reviewer="project_owner",
            notes="single-obstacle detour",
        ),
        "classification_evidence": {
            "line_is_free": False,
            "detour_ratio": 1.2,
        },
    }
    values.update(overrides)
    return BenchmarkEpisode(**values)


def candidate_manifest(
    *, episodes: tuple[BenchmarkEpisode, ...] | None = None
) -> BenchmarkManifest:
    return BenchmarkManifest(
        schema_version="ball_pointgoal_benchmark_v1",
        benchmark_version="v1",
        action_protocol_id="ball_action_v2_aux30",
        localization_source="perfect_localization",
        episodes=episodes or (candidate_episode(),),
    )


def test_freeze_writes_canonical_self_verifying_manifest(tmp_path: Path):
    output = tmp_path / "golden_manifest.json"

    frozen = freeze_manifest(candidate_manifest(), output)
    loaded = load_manifest(output)

    assert frozen.manifest_sha256 is not None
    assert loaded == frozen
    assert output.read_bytes().endswith(b"\n")
    assert b"\n  " not in output.read_bytes()


def test_manifest_requires_approved_tasks_before_freeze(tmp_path: Path):
    episode = candidate_episode(
        human_review=HumanReview(
            status="needs_adjustment",
            reviewer="project_owner",
            notes="move the start pose",
        )
    )

    with pytest.raises(ValueError, match="approved"):
        freeze_manifest(candidate_manifest(episodes=(episode,)), tmp_path / "manifest.json")


def test_manifest_rejects_duplicate_episode_ids():
    episode = candidate_episode()

    with pytest.raises(ValueError, match="duplicate"):
        candidate_manifest(episodes=(episode, episode))


@pytest.mark.parametrize(
    "scene_uri",
    (
        "/absolute/private.usd",
        "${SCENE_ROOT}/../private.usd",
        "${SCENE_ROOT}/safe/../../private.usd",
        "${DATA_ROOT}/scene.usd",
    ),
)
def test_scene_uri_rejects_absolute_wrong_root_and_escape_paths(
    tmp_path: Path, scene_uri: str
):
    with pytest.raises(ValueError, match="SCENE_ROOT|absolute|escapes"):
        candidate_episode(scene_uri=scene_uri).resolve_scene(tmp_path)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("start_position_world_m", (0.0, math.nan, 0.45)),
        ("start_quaternion_world_wxyz", (1.0, 0.0, math.inf, 0.0)),
        ("goal_position_world_m", (math.inf, 0.0, 0.45)),
        ("euclidean_distance_m", math.nan),
        ("oracle_path_length_m", math.inf),
        ("classification_evidence", {"detour_ratio": math.nan}),
    ),
)
def test_episode_rejects_non_finite_numeric_data(field: str, value: object):
    with pytest.raises(ValueError, match="finite"):
        candidate_episode(**{field: value})


def test_scene_hash_mismatch_is_rejected(tmp_path: Path):
    scene_root = tmp_path / "scenes"
    scene = scene_root / "benchmark_easy" / "scene_01.usd"
    scene.parent.mkdir(parents=True)
    scene.write_bytes(b"scene-v1")

    with pytest.raises(ValueError, match="scene SHA256 mismatch"):
        candidate_episode().verify_scene(scene_root)


def test_occupancy_hash_mismatch_is_rejected(tmp_path: Path):
    occupancy = tmp_path / "scene_01_occupancy.npz"
    occupancy.write_bytes(b"occupancy-v1")

    with pytest.raises(ValueError, match="occupancy SHA256 mismatch"):
        candidate_episode().verify_occupancy(occupancy)


def test_load_rejects_manifest_whose_contents_changed_after_freeze(tmp_path: Path):
    output = tmp_path / "manifest.json"
    freeze_manifest(candidate_manifest(), output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    payload["episodes"][0]["goal_position_world_m"][0] = 3.5
    output.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="manifest SHA256 mismatch"):
        load_manifest(output)


def test_freeze_refuses_to_replace_existing_manifest(tmp_path: Path):
    output = tmp_path / "manifest.json"
    output.write_text("keep-me", encoding="utf-8")

    with pytest.raises(FileExistsError):
        freeze_manifest(candidate_manifest(), output)

    assert output.read_text(encoding="utf-8") == "keep-me"


def test_benchmark_v1_config_freezes_protocol_and_rates():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    assert config["schema_version"] == "ball_pointgoal_benchmark_config_v1"
    assert config["benchmark_version"] == "v1"
    assert config["evaluation_protocol_id"] == "ball_pointgoal_v1_1_35s"
    assert config["action_protocol_id"] == "ball_action_v2_aux30"
    assert config["localization_source"] == "perfect_localization"
    assert config["ball_radius_m"] == pytest.approx(0.4)
    assert config["safety_margin_m"] == pytest.approx(0.1)
    assert config["occupancy_resolution_m"] == pytest.approx(0.1)
    assert config["goal_tolerance_m"] == pytest.approx(0.2)
    assert config["episode_budget_s"] == pytest.approx(35.0)
    assert [config[name] for name in ("physics_hz", "control_hz", "stereo_hz", "replan_hz")] == [
        50,
        50,
        10,
        10,
    ]
