"""Regression guards for the version-controlled E2 corridor scene."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SOURCE_SCENE = ROOT / "data/benchmark_v1/scenes/benchmark_e2/e2_corridor.usda"
SCENE_INDEX = ROOT / "configs/benchmark_v1/scene_index.json"
OCCUPANCY = ROOT / "data/benchmark_v1/occupancy/e2_corridor_bodyband.npz"
GOLDEN_MANIFEST = ROOT / "configs/benchmark_v1/golden_v1_manifest.json"


def test_e2_corridor_primitives_have_explicit_physics_extents():
    """USD collision bounds must match the declared primitive dimensions."""

    source = SOURCE_SCENE.read_text(encoding="utf-8")

    assert source.count("def Cylinder") == 26
    assert source.count("def Sphere") == 4
    assert source.count("double radius = 0.45") == 26
    assert source.count("double height = 1.0") == 26
    assert source.count("double radius = 0.30") == 2
    assert source.count("double radius = 0.15") == 2
    assert source.count("float3[] extent = [(-0.45, -0.45, -0.5), (0.45, 0.45, 0.5)]") == 26
    assert source.count("float3[] extent = [(-0.30, -0.30, -0.30), (0.30, 0.30, 0.30)]") == 2
    assert source.count("float3[] extent = [(-0.15, -0.15, -0.15), (0.15, 0.15, 0.15)]") == 2


def test_e2_corridor_artifact_hashes_are_synchronized():
    """The frozen E2 scene, occupancy metadata, and manifest must agree."""

    scene_sha = hashlib.sha256(SOURCE_SCENE.read_bytes()).hexdigest()
    occupancy_sha = hashlib.sha256(OCCUPANCY.read_bytes()).hexdigest()
    scene_index = json.loads(SCENE_INDEX.read_text(encoding="utf-8"))
    scene = next(item for item in scene_index["scenes"] if item["scene_id"] == "e2_corridor")
    assert scene["scene_sha256"] == scene_sha
    assert scene["occupancy_sha256"] == occupancy_sha
    with np.load(OCCUPANCY, allow_pickle=False) as archive:
        assert archive["scene_sha256"].item() == scene_sha

    manifest = json.loads(GOLDEN_MANIFEST.read_text(encoding="utf-8"))
    episode = next(item for item in manifest["episodes"] if item["episode_id"] == "golden_e2_0001")
    assert episode["scene_sha256"] == scene_sha
    assert episode["occupancy_sha256"] == occupancy_sha
    frozen = {key: value for key, value in manifest.items() if key != "manifest_sha256"}
    canonical = json.dumps(
        frozen, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")
    assert manifest["manifest_sha256"] == hashlib.sha256(canonical).hexdigest()
