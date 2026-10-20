from dataclasses import replace
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from collection_schema import (
    EpisodeBuffer,
    RateSchedule,
    StateRecord,
    StereoRecord,
    collection_step,
    derive_stereo_extrinsics,
    load_contract,
    validate_action,
)


CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


def write_contract_with_action_changes(tmp_path, **changes):
    raw = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))
    raw["action"].update(changes)
    path = tmp_path / "data_contract.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def test_v2_contract_declares_full_auxiliary_range():
    """A legacy range or missing protocol ID would silently mix V1 and V2 actions."""
    contract = load_contract(CONTRACT_PATH)

    assert contract.action_protocol_id == "ball_action_v2_aux30"
    assert contract.controls_per_block == 5
    assert contract.action_minimum == pytest.approx((-2.0, -0.5235987756))
    assert contract.action_maximum == pytest.approx((2.0, 0.5235987756))


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"protocol_id": ""}, "protocol_id"),
        ({"controls_per_block": 0}, "controls_per_block"),
        ({"controls_per_block": True}, "controls_per_block"),
    ],
)
def test_contract_rejects_invalid_action_protocol_metadata(tmp_path, changes, message):
    """Invalid version metadata would make dataset and checkpoint compatibility ambiguous."""
    path = write_contract_with_action_changes(tmp_path, **changes)

    with pytest.raises(ValueError, match=message):
        load_contract(path)


@pytest.mark.parametrize(
    ("minimum", "maximum"),
    [
        (np.nan, 2.0),
        (-2.0, np.inf),
        (2.0, 2.0),
        (3.0, 2.0),
    ],
)
def test_contract_rejects_non_finite_or_unordered_action_bounds(
    tmp_path, minimum, maximum
):
    """Invalid component bounds would make clipping or sampling unsafe."""
    raw = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))
    raw["action"]["fields"][0]["minimum"] = minimum
    raw["action"]["fields"][0]["maximum"] = maximum
    path = tmp_path / "data_contract.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")

    with pytest.raises(ValueError, match="action field bounds"):
        load_contract(path)


def test_load_contract_builds_runtime_schedule_and_bev_configuration():
    """Ignoring parsed action/BEV fields would let runtime metadata drift from YAML."""
    contract = load_contract(CONTRACT_PATH)

    assert contract.schema_version == "2.0.0"
    assert contract.elevation_enabled_default is True
    assert contract.elevation_dtype == "float32"
    assert contract.elevation_invalid_value == pytest.approx(0.0)
    assert contract.elevation_teacher_modes == ("auto", "analytic", "raycast")
    assert contract.action_names == (
        "main_joint_velocity_cmd",
        "auxiliary_joint_angle_cmd",
    )
    assert (
        contract.schedule.physics_hz,
        contract.schedule.control_hz,
        contract.schedule.state_hz,
        contract.schedule.stereo_hz,
    ) == (50, 50, 50, 10)
    assert contract.bev.shape == (80, 80)
    assert (contract.bev.free, contract.bev.occupied, contract.bev.unknown) == (0, 1, 2)
    assert contract.action_minimum == pytest.approx((-2.0, -0.5235987756))
    assert contract.action_maximum == pytest.approx((2.0, 0.5235987756))
    assert (contract.image_width_px, contract.image_height_px) == (640, 480)
    assert contract.visual_history_frames == 5
    assert contract.extrinsic_matrix_convention == (
        "column vectors; source_to_target maps p_target = T_source_to_target @ p_source; "
        "left/right use OpenCV optical x_right_y_down_z_forward"
    )


def test_stereo_extrinsics_follow_source_to_target_column_vector_directions():
    """Wrong inversion/composition order would map a left point to the wrong frame."""
    left_optical_to_base = np.eye(4)
    left_optical_to_base[:3, 3] = [1.0, 2.0, 3.0]
    right_optical_to_base = np.eye(4)
    right_optical_to_base[:3, 3] = [4.0, 6.0, 3.0]

    extrinsics = derive_stereo_extrinsics(left_optical_to_base, right_optical_to_base)

    np.testing.assert_allclose(extrinsics.base_to_left_extrinsic[:3, 3], [-1.0, -2.0, -3.0])
    np.testing.assert_allclose(extrinsics.base_to_right_extrinsic[:3, 3], [-4.0, -6.0, -3.0])
    point_left = np.array([0.5, -0.5, 2.0, 1.0])
    np.testing.assert_allclose(
        extrinsics.left_to_right_extrinsic @ point_left,
        extrinsics.base_to_right_extrinsic @ (left_optical_to_base @ point_left),
    )
    np.testing.assert_allclose(
        extrinsics.left_to_right_extrinsic,
        extrinsics.base_to_right_extrinsic @ left_optical_to_base,
    )


def test_collection_step_schedules_stereo_after_five_completed_physics_steps():
    """Scheduling from zero before stepping would incorrectly emit frame zero at time zero."""
    decisions = [collection_step(index, RateSchedule()) for index in range(10)]

    assert [decision.physics_step for decision in decisions] == list(range(1, 11))
    assert [decision.sim_time for decision in decisions] == pytest.approx(
        [0.02 * step for step in range(1, 11)]
    )
    assert [decision.physics_step for decision in decisions if decision.is_stereo] == [5, 10]
    assert all(decision.is_control for decision in decisions)


def test_schedule_emits_exact_50_50_10_boundaries():
    schedule = RateSchedule(physics_hz=50, control_hz=50, stereo_hz=10, state_hz=50)
    assert [s for s in range(10) if schedule.is_control_step(s)] == list(range(10))
    assert [s for s in range(10) if schedule.is_stereo_step(s)] == [0, 5]


@pytest.mark.parametrize(
    "rates",
    [
        (100, 50, 20, 100),
        (50, 25, 10, 50),
        (50, 50, 20, 50),
        (50, 50, 10, 100),
    ],
)
def test_schedule_rejects_noncontract_rates(rates):
    with pytest.raises(ValueError, match="exactly 50/50/50/10"):
        RateSchedule(*rates)


@pytest.mark.parametrize("step", [-1, 1.0, "1", True])
def test_schedule_rejects_invalid_step(step):
    with pytest.raises(ValueError, match="non-negative integer"):
        RateSchedule().is_control_step(step)


def test_validate_action_rejects_legacy_three_dimensions():
    with pytest.raises(ValueError, match="exactly 2"):
        validate_action(np.zeros(3, dtype=np.float32))


def test_validate_action_returns_float32_copy():
    source = np.array([1.0, -0.2], dtype=np.float32)
    result = validate_action(source)
    source[0] = 9.0
    assert result.dtype == np.float32
    np.testing.assert_array_equal(result, np.array([1.0, -0.2], dtype=np.float32))


def test_validate_action_rejects_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        validate_action([np.nan, 0.0])


def make_state(step, time):
    return StateRecord(
        physics_step=step,
        sim_time=time,
        state_timestamp=time,
        pos=np.zeros(3),
        quat=np.array([1.0, 0.0, 0.0, 0.0]),
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        joint_pos=np.zeros(3),
        joint_vel=np.zeros(3),
        commanded_action=np.zeros(2),
        applied_action=np.zeros(2),
        action_command_timestamp=time,
        action_applied_timestamp=time,
    )


def make_stereo(frame_id, step, time):
    return StereoRecord(
        frame_id=frame_id,
        physics_step=step,
        left_timestamp=time,
        right_timestamp=time,
        cam_left_pose=np.eye(4),
        cam_right_pose=np.eye(4),
    )


def test_episode_buffer_preserves_state_and_stereo_time_axes():
    buf = EpisodeBuffer()
    buf.append_state(make_state(step=0, time=0.00))
    buf.append_state(make_state(step=1, time=0.02))
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    arrays = buf.as_arrays()
    assert arrays["physics_step"].shape == (2,)
    assert arrays["applied_action"].shape == (2, 2)
    assert arrays["stereo_frame_id"].shape == (1,)
    assert arrays["cam_left_pose"].shape == (1, 4, 4)


def test_episode_buffer_rejects_nonmonotonic_state_time():
    buf = EpisodeBuffer()
    buf.append_state(make_state(step=0, time=0.00))
    with pytest.raises(ValueError, match="strictly increasing"):
        buf.append_state(make_state(step=1, time=0.00))


@pytest.mark.parametrize(
    "bad_state",
    [
        (2, 0.04),  # skips a physics step
        (1, 0.01),  # wrong fixed state period
        (1, 0.03),  # wrong fixed state period
    ],
)
def test_episode_buffer_rejects_invalid_state_progression(bad_state):
    buf = EpisodeBuffer()
    buf.append_state(make_state(step=0, time=0.00))
    with pytest.raises(ValueError, match="state progression"):
        buf.append_state(make_state(step=bad_state[0], time=bad_state[1]))


def test_episode_buffer_rejects_nonincreasing_sim_time_and_step():
    for step, time, message in [(1, 0.00, "sim_time"), (0, 0.02, "physics_step")]:
        buf = EpisodeBuffer()
        buf.append_state(make_state(step=0, time=0.00))
        with pytest.raises(ValueError, match=message):
            buf.append_state(make_state(step=step, time=time))


@pytest.mark.parametrize(
    "record",
    [
        make_stereo(frame_id=0, step=1, time=0.00),  # not a 10 Hz boundary
        make_stereo(frame_id=0, step=-5, time=0.00),  # negative step
    ],
)
def test_episode_buffer_rejects_invalid_stereo_step(record):
    buf = EpisodeBuffer()
    with pytest.raises(ValueError, match="stereo physics_step"):
        buf.append_stereo(record)


def test_episode_buffer_rejects_nonconsecutive_stereo_rows():
    buf = EpisodeBuffer()
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    with pytest.raises(ValueError, match="stereo_frame_id"):
        buf.append_stereo(make_stereo(frame_id=2, step=10, time=0.20))
    with pytest.raises(ValueError, match="stereo_physics_step"):
        buf.append_stereo(make_stereo(frame_id=1, step=0, time=0.20))


def test_episode_buffer_rejects_skipped_stereo_boundary():
    buf = EpisodeBuffer()
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    with pytest.raises(ValueError, match="exactly 5 physics steps"):
        buf.append_stereo(make_stereo(frame_id=1, step=10, time=0.20))


def test_episode_buffer_rejects_wrong_stereo_time_period():
    buf = EpisodeBuffer()
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    with pytest.raises(ValueError, match="0.1 s"):
        buf.append_stereo(make_stereo(frame_id=1, step=5, time=0.11))


def test_episode_buffer_rejects_nonrecords():
    buf = EpisodeBuffer()
    with pytest.raises(TypeError, match="StateRecord"):
        buf.append_state(object())
    with pytest.raises(TypeError, match="StereoRecord"):
        buf.append_stereo(object())


def test_records_reject_wrong_shapes():
    with pytest.raises(ValueError, match="pos must have shape"):
        replace(make_state(step=0, time=0.00), pos=np.zeros(2))
    with pytest.raises(ValueError, match="cam_left_pose must have shape"):
        replace(make_stereo(frame_id=0, step=0, time=0.00), cam_left_pose=np.eye(3))


@pytest.mark.parametrize(
    ("record", "changes", "message"),
    [
        (make_state(0, 0.0), {"sim_time": np.inf}, "sim_time"),
        (
            make_state(0, 0.0),
            {"action_applied_timestamp": np.nan},
            "action_applied_timestamp",
        ),
        (make_state(0, 0.0), {"quat": [1.0, 0.0, np.nan, 0.0]}, "quat"),
        (make_stereo(0, 0, 0.0), {"left_timestamp": -np.inf}, "left_timestamp"),
        (make_stereo(0, 0, 0.0), {"cam_right_pose": np.full((4, 4), np.nan)}, "cam_right_pose"),
    ],
)
def test_records_reject_non_finite_state_timestamp_and_pose_values(record, changes, message):
    with pytest.raises(ValueError, match=message):
        replace(record, **changes)


@pytest.mark.parametrize("distance", [np.nan, -np.inf])
def test_stereo_record_rejects_invalid_obstacle_distance(distance):
    with pytest.raises(ValueError, match="minimum_obstacle_distance"):
        replace(make_stereo(0, 0, 0.0), minimum_obstacle_distance=distance)


def test_stereo_record_allows_positive_infinity_for_no_obstacle():
    record = replace(
        make_stereo(0, 0, 0.0),
        collision=False,
        minimum_obstacle_distance=np.inf,
    )

    assert np.isposinf(record.minimum_obstacle_distance)


def test_stereo_record_allows_negative_clearance_only_with_collision():
    record = replace(
        make_stereo(0, 0, 0.0),
        collision=True,
        minimum_obstacle_distance=-0.1,
    )
    assert record.minimum_obstacle_distance == pytest.approx(-0.1)

    with pytest.raises(ValueError, match="collision must equal"):
        replace(record, collision=False)


def test_record_arrays_are_copied_from_callers():
    position = np.ones(3, dtype=np.float32)
    pose = np.eye(4, dtype=np.float32)
    state = replace(make_state(step=0, time=0.00), pos=position)
    stereo = replace(make_stereo(frame_id=0, step=0, time=0.00), cam_left_pose=pose)
    position[0] = 99.0
    pose[0, 0] = 99.0
    assert state.pos[0] == 1.0
    assert stereo.cam_left_pose[0, 0] == 1.0


def test_episode_buffer_rejects_unsynchronized_or_nonincreasing_stereo_time():
    buf = EpisodeBuffer()
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    unsynchronized = make_stereo(frame_id=1, step=5, time=0.10)
    unsynchronized.right_timestamp += 0.01
    with pytest.raises(ValueError, match="timestamps"):
        buf.append_stereo(unsynchronized)

    repeated = make_stereo(frame_id=1, step=5, time=0.00)
    with pytest.raises(ValueError, match="timestamps"):
        buf.append_stereo(repeated)


def test_episode_buffer_empty_arrays_have_contract_shapes():
    arrays = EpisodeBuffer().as_arrays()
    assert arrays["physics_step"].shape == (0,)
    assert arrays["applied_action"].shape == (0, 2)
    assert arrays["cam_left_pose"].shape == (0, 4, 4)


def test_writer_uses_lossless_paired_stereo_and_contract_arrays(tmp_path):
    """Removing a paired PNG or writing a legacy three-action array must break this."""
    from collection_schema import EpisodeWriter

    episode = tmp_path / "ep0000"
    writer = EpisodeWriter(
        episode,
        metadata={"scene": "unit-test", "image_height_px": 4, "image_width_px": 6},
        save_depth=True,
        save_preview=False,
    )
    rgb = np.full((4, 6, 3), 17, dtype=np.uint8)
    depth = np.full((4, 6), 1.5, dtype=np.float32)
    bev = np.zeros((80, 80), dtype=np.uint8)
    mask = np.ones((80, 80), dtype=np.uint8)
    writer.write_visual_sample(
        frame_id=0,
        left_rgb=rgb,
        right_rgb=rgb,
        left_depth=depth,
        right_depth=depth,
        geometry_bev=bev,
        current_visible=mask,
        history_observed=mask,
    )
    buffer = EpisodeBuffer()
    buffer.append_state(make_state(0, 0.0))
    buffer.append_stereo(make_stereo(0, 0, 0.0))
    writer.finalize(buffer)

    assert (episode / "left_000000.png").exists()
    assert (episode / "right_000000.png").exists()
    assert (episode / "left_depth_000000.npy").exists()
    assert not (tmp_path / "ep0000.partial").exists()
    with np.load(episode / "states.npz") as data:
        assert data["applied_action"].shape == (1, 2)


def test_writer_keeps_partial_data_when_finalize_fails(tmp_path):
    """Publishing a mismatched visual/state episode would expose incomplete data."""
    from collection_schema import EpisodeWriter

    episode = tmp_path / "ep0000"
    writer = EpisodeWriter(
        episode,
        metadata={"image_height_px": 4, "image_width_px": 6},
        save_depth=False,
        save_preview=False,
    )
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    bev = np.zeros((80, 80), dtype=np.uint8)
    mask = np.zeros((80, 80), dtype=np.uint8)
    writer.write_visual_sample(0, rgb, rgb, None, None, bev, mask, mask)
    with pytest.raises(ValueError, match="visual frame IDs"):
        writer.finalize(EpisodeBuffer())

    assert not episode.exists()
    assert (tmp_path / "ep0000.partial" / "left_000000.png").exists()


def test_writer_preview_is_a_five_panel_horizontal_mosaic(tmp_path):
    """Dropping any visual panel or stacking the preview vertically must fail this."""
    from PIL import Image

    from collection_schema import EpisodeWriter

    writer = EpisodeWriter(
        tmp_path / "ep0000",
        metadata={"image_height_px": 4, "image_width_px": 6},
        save_depth=False,
        save_preview=True,
    )
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    bev = np.zeros((80, 80), dtype=np.uint8)
    mask = np.zeros((80, 80), dtype=np.uint8)
    writer.write_visual_sample(0, rgb, rgb, None, None, bev, mask, mask)
    with Image.open(tmp_path / "ep0000.partial" / "preview_000000.png") as preview:
        assert preview.mode == "RGB"
        assert preview.size == (30, 4)


def test_writer_refuses_to_overwrite_a_final_episode(tmp_path):
    """Replacing an existing finalized episode would silently destroy user data."""
    from collection_schema import EpisodeWriter

    episode = tmp_path / "ep0000"
    episode.mkdir()
    (episode / "sentinel.txt").write_text("keep", encoding="utf-8")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        EpisodeWriter(episode, metadata={}, save_depth=False, save_preview=False)
    assert (episode / "sentinel.txt").read_text(encoding="utf-8") == "keep"


def test_writer_requires_declared_image_dimensions(tmp_path):
    from collection_schema import EpisodeWriter

    with pytest.raises(ValueError, match="image_height_px"):
        EpisodeWriter(tmp_path / "ep0000", metadata={}, save_depth=False, save_preview=False)


def test_writer_rejects_rgb_that_does_not_match_declared_dimensions(tmp_path):
    from collection_schema import EpisodeWriter

    writer = EpisodeWriter(
        tmp_path / "ep0000",
        metadata={"image_height_px": 4, "image_width_px": 6},
        save_depth=False,
        save_preview=False,
    )
    wrong_rgb = np.zeros((3, 6, 3), dtype=np.uint8)
    bev = np.zeros((80, 80), dtype=np.uint8)
    mask = np.zeros((80, 80), dtype=np.uint8)

    with pytest.raises(ValueError, match="declared image dimensions"):
        writer.write_visual_sample(0, wrong_rgb, wrong_rgb, None, None, bev, mask, mask)


def _elevation_writer(tmp_path, **kwargs):
    from collection_schema import EpisodeWriter

    return EpisodeWriter(
        tmp_path / "ep0000",
        metadata={"image_height_px": 4, "image_width_px": 6, **kwargs.pop("metadata", {})},
        save_depth=False,
        save_preview=False,
        save_elevation=True,
        **kwargs,
    )


def _write_elevation(writer, minimum=None, maximum=None, valid=None):
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    bev = np.zeros((80, 80), dtype=np.uint8)
    mask = np.zeros((80, 80), dtype=np.uint8)
    writer.write_visual_sample(0, rgb, rgb, None, None, bev, mask, mask,
                               minimum, maximum, valid)


def test_writer_saves_elevation_assets_and_metadata_when_enabled(tmp_path):
    writer = _elevation_writer(tmp_path)
    minimum = np.full((80, 80), 0.2, dtype=np.float32)
    maximum = np.full((80, 80), 1.0, dtype=np.float32)
    valid = np.ones((80, 80), dtype=np.uint8)
    _write_elevation(writer, minimum, maximum, valid)
    assert (tmp_path / "ep0000.partial" / "height_min_bev_000000.npy").exists()
    assert (tmp_path / "ep0000.partial" / "height_max_bev_000000.npy").exists()
    assert (tmp_path / "ep0000.partial" / "height_valid_bev_000000.png").exists()
    writer._write_metadata()
    assert '"elevation_enabled": true' in (tmp_path / "ep0000.partial" / "metadata.json").read_text()


@pytest.mark.parametrize(
    ("minimum", "maximum", "valid", "message"),
    [
        (None, np.ones((80, 80), dtype=np.float32), np.ones((80, 80), dtype=np.uint8), "required"),
        (np.zeros((79, 80), dtype=np.float32), np.zeros((80, 80), dtype=np.float32), np.ones((80, 80), dtype=np.uint8), "shape"),
        (np.zeros((80, 80), dtype=np.float64), np.zeros((80, 80), dtype=np.float32), np.ones((80, 80), dtype=np.uint8), "dtype"),
        (np.full((80, 80), np.nan, dtype=np.float32), np.ones((80, 80), dtype=np.float32), np.ones((80, 80), dtype=np.uint8), "finite"),
        (np.zeros((80, 80), dtype=np.float32), np.ones((80, 80), dtype=np.float32), np.full((80, 80), 2, dtype=np.uint8), "binary"),
        (np.full((80, 80), 0.1, dtype=np.float32), np.full((80, 80), 0.2, dtype=np.float32), np.zeros((80, 80), dtype=np.uint8), "invalid"),
        (np.full((80, 80), 1.0, dtype=np.float32), np.full((80, 80), 0.2, dtype=np.float32), np.ones((80, 80), dtype=np.uint8), "minimum.*maximum"),
    ],
)
def test_writer_rejects_invalid_elevation_assets(tmp_path, minimum, maximum, valid, message):
    writer = _elevation_writer(tmp_path)
    with pytest.raises(ValueError, match=message):
        _write_elevation(writer, minimum, maximum, valid)


def test_writer_rejects_contradictory_elevation_metadata(tmp_path):
    with pytest.raises(ValueError, match="elevation_enabled"):
        _elevation_writer(tmp_path, metadata={"elevation_enabled": False})


def test_no_clobber_publication_keeps_empty_final_and_partial_directory(tmp_path):
    """A final directory created after staging must never be replaced, even when empty."""
    from collection_schema import _publish_noreplace

    partial = tmp_path / "ep0000.partial"
    partial.mkdir()
    (partial / "staged.txt").write_text("staged", encoding="utf-8")
    final = tmp_path / "ep0000"
    final.mkdir()

    with pytest.raises(FileExistsError):
        _publish_noreplace(partial, final)
    assert final.is_dir()
    assert not any(final.iterdir())
    assert (partial / "staged.txt").read_text(encoding="utf-8") == "staged"
