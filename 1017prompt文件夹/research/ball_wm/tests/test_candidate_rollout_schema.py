"""Pure schema tests for same-context multi-candidate rollout artifacts."""

from pathlib import Path

import numpy as np
import pytest

from training.candidate_rollout_schema import (
    CandidateContextRef,
    validate_candidate_arrays,
    write_candidate_context_atomic,
)


def _metadata() -> dict[str, object]:
    return {
        "schema_version": "candidate_rollout_v1",
        "source_episode": "scene_easy_0/ep0000",
        "context_start": 0,
        "history_frames": 5,
        "rollout_horizon": 5,
        "action_block": 5,
        "goal_relative_xy": [1.0, 0.0],
        "scene": "scene_easy_0",
        "seed": 1,
    }


def _valid_arrays(k: int = 8) -> dict[str, np.ndarray]:
    pose = np.zeros((k, 5, 4), dtype=np.float32)
    pose[..., 3] = 1.0
    return {
        "action": np.zeros((k, 5, 10), dtype=np.float32),
        "future_relative_pose": pose,
        "goal_distance_m": np.arange(k, dtype=np.float32),
        "collision": np.zeros((k, 5), dtype=np.uint8),
        "clearance_m": np.full((k, 5), np.inf, dtype=np.float32),
    }


def test_candidate_schema_rejects_absolute_or_traversal_source_episode():
    for source_episode in ("../ep0000", "/tmp/ep0000"):
        values = {**_metadata(), "source_episode": source_episode}
        with pytest.raises(ValueError, match="source_episode"):
            CandidateContextRef.from_mapping(values)


def test_candidate_schema_rejects_mismatched_shapes_and_invalid_clearance():
    arrays = _valid_arrays()
    arrays["action"] = arrays["action"][:, :-1]
    with pytest.raises(ValueError, match="action"):
        validate_candidate_arrays(arrays)

    arrays = _valid_arrays()
    arrays["clearance_m"][0, 0] = -np.inf
    with pytest.raises(ValueError, match="clearance"):
        validate_candidate_arrays(arrays)


def test_atomic_writer_publishes_only_after_validation(tmp_path: Path):
    published = write_candidate_context_atomic(tmp_path / "context_0000", _metadata(), _valid_arrays())

    assert published.name == "context_0000"
    assert not (tmp_path / "context_0000.partial").exists()
    assert (published / "context.json").is_file()
    assert (published / "candidates.npz").is_file()


def test_candidate_schema_exposes_v2_action_protocol():
    ref = CandidateContextRef.from_mapping(
        {
            **_metadata(),
            "action_protocol_id": "ball_action_v2_aux30",
            "controls_per_block": 5,
        }
    )

    assert ref.action_protocol_id == "ball_action_v2_aux30"
    assert ref.controls_per_block == 5
