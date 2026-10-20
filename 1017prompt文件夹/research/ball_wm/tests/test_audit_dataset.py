import json
import sys
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from collection_schema import EpisodeBuffer, EpisodeWriter, StateRecord, StereoRecord
from collect_data_dubins import _audit_episode_in_subprocess


IMAGE_HEIGHT_PX = 480
IMAGE_WIDTH_PX = 640


def _state(step: int) -> StateRecord:
    timestamp = step / 50.0
    return StateRecord(
        physics_step=step,
        sim_time=timestamp,
        state_timestamp=timestamp,
        pos=np.zeros(3, dtype=np.float32),
        quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        lin_vel=np.zeros(3, dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        joint_pos=np.zeros(3, dtype=np.float32),
        joint_vel=np.zeros(3, dtype=np.float32),
        commanded_action=np.zeros(2, dtype=np.float32),
        applied_action=np.zeros(2, dtype=np.float32),
        action_command_timestamp=timestamp,
        action_applied_timestamp=timestamp,
    )


def _stereo(frame_id: int, step: int) -> StereoRecord:
    timestamp = step / 50.0
    return StereoRecord(
        frame_id=frame_id,
        physics_step=step,
        left_timestamp=timestamp,
        right_timestamp=timestamp,
        cam_left_pose=np.eye(4, dtype=np.float32),
        cam_right_pose=np.eye(4, dtype=np.float32),
    )


def write_valid_fixture(tmp_path: Path, name: str = "ep0000", save_depth: bool = False, save_elevation: bool = False) -> Path:
    """Create 10 state samples and two 10 Hz visuals; the final state tail is valid."""
    episode = tmp_path / name
    writer = EpisodeWriter(
        episode,
        metadata={
            "scene": "audit-fixture",
            "image_height_px": IMAGE_HEIGHT_PX,
            "image_width_px": IMAGE_WIDTH_PX,
        },
        save_depth=save_depth,
        save_elevation=save_elevation,
        save_preview=False,
    )
    rgb = np.zeros((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX, 3), dtype=np.uint8)
    geometry = np.zeros((80, 80), dtype=np.uint8)
    mask = np.ones((80, 80), dtype=np.uint8)
    for frame_id in range(2):
        writer.write_visual_sample(
            frame_id,
            rgb,
            rgb,
            np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), dtype=np.float32) if save_depth else None,
            np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), dtype=np.float32) if save_depth else None,
            geometry,
            mask,
            mask,
            np.full((80, 80), -0.2, dtype=np.float32) if save_elevation else None,
            np.full((80, 80), 0.8, dtype=np.float32) if save_elevation else None,
            np.ones((80, 80), dtype=np.uint8) if save_elevation else None,
        )
    buffer = EpisodeBuffer()
    for step in range(10):
        buffer.append_state(_state(step))
    buffer.append_stereo(_stereo(0, 0))
    buffer.append_stereo(_stereo(1, 5))
    writer.finalize(buffer)
    return episode


def replace_npz_field(path: Path, field: str, value: np.ndarray) -> None:
    with np.load(path) as data:
        arrays = {name: data[name] for name in data.files}
    arrays[field] = value
    np.savez_compressed(path, **arrays)


def test_audit_accepts_fixed_cadence_with_final_state_tail(tmp_path):
    """Skipping the allowed end-of-episode state tail must not become an audit error."""
    from audit_dataset import audit_episode

    assert audit_episode(write_valid_fixture(tmp_path)) == []


def test_clean_subprocess_audit_rejects_an_invalid_published_episode(tmp_path):
    """Kit's bundled Pillow modules must not contaminate the post-episode auditor."""
    episode = write_valid_fixture(tmp_path)
    (episode / "right_000000.png").unlink()

    with pytest.raises(RuntimeError, match="missing right_000000.png"):
        _audit_episode_in_subprocess(episode, python_executable=sys.executable)


def test_audit_rejects_three_dimensional_action(tmp_path):
    """Changing the action contract from two controls to three must be reported."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    replace_npz_field(episode / "states.npz", "applied_action", np.zeros((10, 3), np.float32))
    errors = audit_episode(episode)
    assert any("applied_action must have shape (N, 2)" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "replacement"),
    [
        ("sim_time", np.full(10, np.nan, dtype=np.float64)),
        ("state_timestamp", np.full(10, np.inf, dtype=np.float64)),
        ("applied_action", np.full((10, 2), np.nan, dtype=np.float32)),
        ("quat", np.full((10, 4), np.inf, dtype=np.float32)),
        ("left_timestamp", np.full(2, np.nan, dtype=np.float64)),
        ("cam_left_pose", np.full((2, 4, 4), np.inf, dtype=np.float32)),
    ],
)
def test_audit_rejects_non_finite_numeric_fields(tmp_path, field, replacement):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    replace_npz_field(episode / "states.npz", field, replacement)

    assert any(f"{field} must contain only finite values" in error for error in audit_episode(episode))


@pytest.mark.parametrize("distance", [np.nan, -np.inf])
def test_audit_rejects_invalid_minimum_obstacle_distance(tmp_path, distance):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    replace_npz_field(
        episode / "states.npz",
        "minimum_obstacle_distance",
        np.full(2, distance, dtype=np.float32),
    )

    assert any("minimum_obstacle_distance" in error for error in audit_episode(episode))


def test_audit_rejects_collision_that_disagrees_with_clearance(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    replace_npz_field(
        episode / "states.npz",
        "minimum_obstacle_distance",
        np.asarray([-0.1, np.inf], dtype=np.float32),
    )

    assert any("collision must equal" in error for error in audit_episode(episode))


def test_audit_rejects_unpaired_stereo_file(tmp_path):
    """Deleting one right camera asset must make the stereo pair invalid."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    (episode / "right_000000.png").unlink()
    assert any("missing right_000000.png" in error for error in audit_episode(episode))


@pytest.mark.parametrize(
    ("field", "replacement", "message"),
    [
        ("physics_step", np.zeros((10, 1), dtype=np.int64), "physics_step must have shape (N,)"),
        ("stereo_frame_id", np.zeros((2, 1), dtype=np.int64), "stereo_frame_id must have shape (M,)"),
    ],
)
def test_audit_returns_errors_for_malformed_required_arrays(tmp_path, field, replacement, message):
    """A corrupt required array must be reported, never raised from a dependent audit."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    replace_npz_field(episode / "states.npz", field, replacement)
    errors = audit_episode(episode)
    assert isinstance(errors, list)
    assert any(message in error for error in errors)


def test_audit_cli_continues_after_corrupt_episode(tmp_path, capsys):
    """A first corrupt episode must not stop auditing later episodes in the same directory."""
    from audit_dataset import main

    first = write_valid_fixture(tmp_path, "ep0000")
    second = write_valid_fixture(tmp_path, "ep0001")
    replace_npz_field(first / "states.npz", "physics_step", np.zeros((10, 1), dtype=np.int64))
    replace_npz_field(second / "states.npz", "stereo_frame_id", np.zeros((2, 1), dtype=np.int64))

    assert main([str(tmp_path)]) == 1
    output = capsys.readouterr().out
    assert "ep0000" in output
    assert "ep0001" in output


def test_audit_cli_recurses_through_scene_grouped_root(tmp_path):
    write_valid_fixture(tmp_path / "scene_easy_0" / "ep0000")
    write_valid_fixture(tmp_path / "scene_easy_1" / "ep0000")

    from audit_dataset import main

    assert main([str(tmp_path)]) == 0


@pytest.mark.parametrize("field", ["action_command_timestamp", "action_applied_timestamp"])
def test_audit_rejects_nonmonotonic_action_timestamps(tmp_path, field):
    """Losing a 50 Hz action timestamp interval breaks the zero-order-hold record."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    timestamps = np.arange(10, dtype=np.float64) / 50.0
    timestamps[4] = timestamps[3]
    replace_npz_field(episode / "states.npz", field, timestamps)
    assert any(f"{field} must be strictly increasing" in error for error in audit_episode(episode))


@pytest.mark.parametrize("field", ["action_command_timestamp", "action_applied_timestamp"])
def test_audit_rejects_action_timestamps_misaligned_with_state_time(tmp_path, field):
    """A shifted action timestamp cannot describe the state/control event at that step."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    timestamps = np.arange(10, dtype=np.float64) / 50.0 + 0.001
    replace_npz_field(episode / "states.npz", field, timestamps)
    assert any(f"{field} must align with state_timestamp and sim_time" in error for error in audit_episode(episode))


def test_audit_requires_paired_float32_depth_when_metadata_enables_depth(tmp_path):
    """Removing one depth side must be caught only for episodes that declared depth output."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_depth=True)
    (episode / "left_depth_000000.npy").unlink()
    assert any("missing left_depth_000000.npy" in error for error in audit_episode(episode))


def test_audit_rejects_depth_dtype_and_shape_mismatches(tmp_path):
    """Depth arrays must preserve float32 pixels at the metadata-declared RGB size."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_depth=True)
    np.save(
        episode / "left_depth_000000.npy",
        np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), dtype=np.float64),
    )
    np.save(
        episode / "right_depth_000000.npy",
        np.ones((IMAGE_HEIGHT_PX - 1, IMAGE_WIDTH_PX), dtype=np.float32),
    )
    errors = audit_episode(episode)
    assert any("left_depth_000000.npy must have dtype float32" in error for error in errors)
    assert any(
        f"right_depth_000000.npy must have shape ({IMAGE_HEIGHT_PX}, {IMAGE_WIDTH_PX})" in error
        for error in errors
    )


def test_audit_reports_npz_container_disguised_as_depth_npy(tmp_path):
    """A zip container at a depth path must become an audit error, not an AttributeError."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_depth=True)
    with (episode / "left_depth_000000.npy").open("wb") as handle:
        np.savez(
            handle,
            depth=np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), dtype=np.float32),
        )
    errors = audit_episode(episode)
    assert isinstance(errors, list)
    assert any("left_depth_000000.npy must contain a NumPy ndarray" in error for error in errors)


def test_audit_uses_decoded_rgb_dimensions_when_metadata_and_depth_are_tampered(tmp_path):
    """Changing metadata and depth together cannot mask a mismatch with the paired RGB pixels."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_depth=True)
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["image_height_px"] = 7
    metadata["image_width_px"] = 9
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    for frame_id in range(2):
        for prefix in ("left_depth", "right_depth"):
            np.save(episode / f"{prefix}_{frame_id:06d}.npy", np.ones((7, 9), dtype=np.float32))

    errors = audit_episode(episode)
    assert any("metadata.json image dimensions must match left_000000.png" in error for error in errors)
    assert any(
        f"left_depth_000000.npy must have shape ({IMAGE_HEIGHT_PX}, {IMAGE_WIDTH_PX})" in error
        for error in errors
    )


def test_audit_rejects_wrong_rgb_dimensions_when_depth_is_disabled(tmp_path):
    from PIL import Image

    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_depth=False)
    Image.fromarray(np.zeros((8, 10, 3), dtype=np.uint8)).save(
        episode / "left_000000.png", format="PNG"
    )

    assert any(
        "metadata.json image dimensions must match left_000000.png" in error
        for error in audit_episode(episode)
    )


def test_audit_rejects_orphan_visual_file(tmp_path):
    """A right image with no stereo record is an incomplete or misindexed sample."""
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    (episode / "right_000002.png").write_bytes((episode / "right_000000.png").read_bytes())
    assert any("orphan right_000002.png" in error for error in audit_episode(episode))


def test_audit_accepts_valid_elevation_assets(tmp_path):
    from audit_dataset import audit_episode

    assert audit_episode(write_valid_fixture(tmp_path, save_elevation=True)) == []


def test_audit_requires_all_elevation_assets_for_each_frame(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=True)
    (episode / "height_min_bev_000001.npy").unlink()
    assert any("missing height_min_bev_000001.npy" in error for error in audit_episode(episode))


@pytest.mark.parametrize(
    ("filename", "replacement", "message"),
    [
        ("height_min_bev_000000.npy", np.zeros((80, 80), dtype=np.float64), "dtype float32"),
        ("height_max_bev_000000.npy", np.zeros((79, 80), dtype=np.float32), "shape (80, 80)"),
        ("height_min_bev_000000.npy", np.full((80, 80), np.nan, dtype=np.float32), "finite"),
    ],
)
def test_audit_rejects_invalid_elevation_arrays(tmp_path, filename, replacement, message):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=True)
    np.save(episode / filename, replacement)
    assert any(filename in error and message in error for error in audit_episode(episode))


def test_audit_rejects_illegal_elevation_mask_values(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=True)
    from PIL import Image
    Image.fromarray(np.full((80, 80), 2, dtype=np.uint8)).save(episode / "height_valid_bev_000000.png")
    assert any("height_valid_bev_000000.png" in error for error in audit_episode(episode))


def test_audit_rejects_reversed_valid_elevation_bounds(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=True)
    np.save(episode / "height_min_bev_000000.npy", np.full((80, 80), 2.0, dtype=np.float32))
    assert any("height_min_bev_000000.npy" in error and "height_min" in error for error in audit_episode(episode))


def test_audit_rejects_nonzero_invalid_elevation_values(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=True)
    np.save(episode / "height_min_bev_000000.npy", np.zeros((80, 80), dtype=np.float32) + 1.0)
    from PIL import Image
    Image.fromarray(np.zeros((80, 80), dtype=np.uint8)).save(episode / "height_valid_bev_000000.png")
    assert any("invalid" in error and "height_min_bev_000000.npy" in error for error in audit_episode(episode))


def test_audit_reports_orphan_elevation_files_when_disabled(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=False)
    np.save(episode / "height_min_bev_000000.npy", np.zeros((80, 80), dtype=np.float32))
    assert any("orphan height_min_bev_000000.npy" in error for error in audit_episode(episode))


def test_audit_treats_missing_elevation_metadata_as_disabled(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path, save_elevation=False)
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata.pop("elevation_enabled", None)
    metadata_path.write_text(json.dumps(metadata))
    np.save(episode / "height_min_bev_000000.npy", np.zeros((80, 80), dtype=np.float32))
    assert any("orphan height_min_bev_000000.npy" in error for error in audit_episode(episode))


def test_audit_requires_strict_boolean_elevation_metadata(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["elevation_enabled"] = 1
    metadata_path.write_text(json.dumps(metadata))
    assert any("elevation_enabled must be a boolean" in error for error in audit_episode(episode))


def _label_aux30_episode(episode: Path, auxiliary_rad: float) -> None:
    metadata_path = episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["action_protocol_id"] = "ball_action_v2_aux30"
    metadata["controls_per_block"] = 5
    metadata_path.write_text(json.dumps(metadata))
    with np.load(episode / "states.npz", allow_pickle=False) as loaded:
        commanded = loaded["commanded_action"].copy()
        applied = loaded["applied_action"].copy()
    commanded[:, 1] = auxiliary_rad
    applied[:, 1] = auxiliary_rad
    replace_npz_field(episode / "states.npz", "commanded_action", commanded)
    replace_npz_field(episode / "states.npz", "applied_action", applied)


def test_auxiliary_statistics_use_fixed_five_degree_bins_and_report_high_angles():
    from audit_dataset import auxiliary_action_statistics

    actions = np.zeros((4, 2), dtype=np.float32)
    actions[:, 1] = np.deg2rad([-30.0, -10.0, 0.0, 20.0])

    result = auxiliary_action_statistics(actions)

    assert result["histogram_bin_edges_deg"] == list(range(-30, 35, 5))
    assert sum(result["histogram_counts"]) == 4
    assert result["high_angle_sample_count"] == 2
    assert result["high_angle_fraction"] == pytest.approx(0.5)


def test_v2_audit_rejects_collection_without_high_angle_samples(tmp_path):
    from audit_dataset import audit_episode

    episode = write_valid_fixture(tmp_path)
    _label_aux30_episode(episode, 0.0)

    assert any("high-angle" in error for error in audit_episode(episode))


def test_v2_audit_accepts_collection_with_high_angle_samples(tmp_path):
    from audit_dataset import audit_episode_report

    episode = write_valid_fixture(tmp_path)
    _label_aux30_episode(episode, float(np.deg2rad(20.0)))

    report = audit_episode_report(episode)

    assert report["errors"] == []
    assert report["action_protocol_id"] == "ball_action_v2_aux30"
    assert report["auxiliary_actions"]["high_angle_sample_count"] == 10
    assert report["auxiliary_actions"]["high_angle_fraction"] == pytest.approx(1.0)


def test_audit_cli_rejects_mixed_legacy_and_v2_protocols(tmp_path, capsys):
    from audit_dataset import main

    write_valid_fixture(tmp_path, "ep0000")
    v2_episode = write_valid_fixture(tmp_path, "ep0001")
    _label_aux30_episode(v2_episode, float(np.deg2rad(20.0)))

    assert main([str(tmp_path)]) == 1
    assert "mixed action protocols" in capsys.readouterr().out
