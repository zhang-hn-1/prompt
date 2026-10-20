"""Regression guards for the registered cluttered-hard benchmark scene."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCENE_INDEX = ROOT / "configs/benchmark_v1/scene_index.json"
OCCUPANCY = ROOT / "data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_hard_0_bodyband_map_is_registered_with_reproducible_metadata():
    index = json.loads(SCENE_INDEX.read_text(encoding="utf-8"))
    scene = next(item for item in index["scenes"] if item["scene_id"] == "cluttered_hard_0")

    assert scene["scene_uri"] == "${SCENE_ROOT}/cluttered_hard/hard_0/cluttered-0.usd"
    assert scene["occupancy_uri"] == "${DATA_ROOT}/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz"
    assert scene["occupancy_projection"] == "vertical_max_body_band"
    assert scene["body_radius_m"] == pytest.approx(0.4)
    assert scene["navigation_z_m"] == pytest.approx(0.45)
    assert scene["difficulties"] == ["E0", "E1"]
    assert scene["occupancy_sha256"] == _sha256(OCCUPANCY)

    with np.load(OCCUPANCY, allow_pickle=False) as archive:
        assert archive["scene_uri"].item() == scene["scene_uri"]
        assert archive["scene_sha256"].item() == scene["scene_sha256"]
        assert archive["projection"].item() == scene["occupancy_projection"]
        assert float(archive["body_radius_m"].item()) == pytest.approx(scene["body_radius_m"])
        assert float(archive["resolution_m"].item()) == pytest.approx(0.1)
        grid = np.asarray(archive["grid"], dtype=np.uint8)

    assert grid.ndim == 2
    assert grid.shape == (620, 620)
    assert 0.05 < float((grid != 0).mean()) < 0.25
