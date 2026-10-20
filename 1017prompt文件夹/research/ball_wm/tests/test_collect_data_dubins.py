from pathlib import Path
from io import StringIO
import math
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from collect_data_dubins import (
    _assert_planar_elevation_pose,
    _collection_runtime,
    _episode_metadata,
    _load_collection_scene,
    _parse_args,
    _prime_stereo_cameras,
    _reset_articulation_after_occupancy,
    _warmup_physics_scene,
)
from collection_schema import load_contract


CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


@pytest.mark.parametrize(
    ("scene_path", "expected_scene_root"),
    [("/tmp/clutter.usd", "/World/Scene"), ("", None)],
)
def test_scene_setup_adds_one_ground_collision_for_every_collection_mode(
    scene_path, expected_scene_root
):
    """Removing the shared ground setup would leave external scenes without a floor collider."""

    events = []

    class Scene:
        def add_default_ground_plane(self):
            events.append(("ground",))

    class World:
        scene = Scene()

    class XForm:
        def __init__(self, path):
            events.append(("xform", path))

        def set_local_pose(self, translation):
            np.testing.assert_array_equal(translation, np.zeros(3))
            events.append(("pose",))

    result = _load_collection_scene(
        World(),
        scene_path,
        lambda usd_path, prim_path: events.append(("reference", usd_path, prim_path)),
        XForm,
        print_fn=lambda _message: None,
    )

    assert result == expected_scene_root
    assert events.count(("ground",)) == 1
    if scene_path:
        assert ("reference", scene_path, "/World/Scene") in events
        assert ("xform", "/World/Scene") in events
    else:
        assert events == [("ground",)]


def test_collection_runtime_reports_failure_before_isaac_cleanup():
    """A runtime exception must remain observable even when Kit cleanup silences logging."""

    class FakeSimulationApp:
        closed = False

        def close(self):
            self.closed = True

    simulation_app = FakeSimulationApp()
    stderr = StringIO()

    with pytest.raises(RuntimeError, match="OMap failed"):
        with _collection_runtime(simulation_app, stderr):
            raise RuntimeError("OMap failed")

    assert simulation_app.closed
    assert "Collection failed: RuntimeError: OMap failed" in stderr.getvalue()
    assert "RuntimeError: OMap failed" in stderr.getvalue()


def test_collection_runtime_failure_stays_nonzero_when_close_exits_zero():
    """Kit's successful SystemExit during close must not erase an active failure."""

    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    program = f"""
import sys
sys.path.insert(0, {str(scripts_dir)!r})
from collect_data_dubins import _collection_runtime

class SimulationApp:
    def close(self):
        raise SystemExit(0)

with _collection_runtime(SimulationApp()):
    raise RuntimeError("original collection failure")
"""

    completed = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 1
    assert "Collection failed: RuntimeError: original collection failure" in completed.stderr
    assert completed.stderr.rstrip().endswith("RuntimeError: original collection failure")


def test_collection_runtime_posts_failure_status_before_nonreturning_close():
    """Real Kit shutdown never returns, so its process status must be set first."""

    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    program = f"""
import os
import sys
sys.path.insert(0, {str(scripts_dir)!r})
from collect_data_dubins import _collection_runtime

class KitApp:
    return_code = 0

    def post_quit(self, return_code):
        self.return_code = return_code

class SimulationApp:
    app = KitApp()

    def close(self):
        os._exit(self.app.return_code)

with _collection_runtime(SimulationApp()):
    raise RuntimeError("original collection failure")
"""

    completed = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 1
    assert "Collection failed: RuntimeError: original collection failure" in completed.stderr


def test_articulation_is_reset_and_regained_after_occupancy_mutates_collision_schema():
    """OMap collision exclusion invalidates the existing Isaac tensor/controller view."""
    events = []

    class Controller:
        def set_gains(self, kps, kds):
            events.append(("gains", kps.copy(), kds.copy()))

    controller = Controller()

    class World:
        def reset(self):
            events.append(("reset",))

    class Robot:
        def get_articulation_controller(self):
            events.append(("controller",))
            return controller

    kps = np.array([0.0, 500.0, 0.0])
    kds = np.array([80.0, 50.0, 0.0])

    result = _reset_articulation_after_occupancy(World(), Robot(), kps, kds)

    assert result is controller
    assert events[0] == ("reset",)
    assert events[1] == ("controller",)
    np.testing.assert_array_equal(events[2][1], kps)
    np.testing.assert_array_equal(events[2][2], kds)


def test_stereo_priming_waits_until_both_depth_annotators_are_images():
    """Isaac annotators initially return scalar placeholders despite an available RGB frame."""

    class Camera:
        prim_path = "/World/camera"

        def __init__(self):
            self.frame = {"distance_to_image_plane": None}

        def get_current_frame(self):
            return self.frame

    left, right = Camera(), Camera()

    class World:
        steps = 0

        def step(self, render):
            assert render
            self.steps += 1
            if self.steps == 3:
                left.frame["distance_to_image_plane"] = np.ones((2, 3), dtype=np.float32)
                right.frame["distance_to_image_plane"] = np.ones((2, 3), dtype=np.float32)

    world = World()

    _prime_stereo_cameras(world, left, right, max_steps=5)

    assert world.steps == 3


def test_stereo_priming_fails_after_its_explicit_bound():
    """A broken annotator must not cause an unbounded simulator warmup."""

    class Camera:
        prim_path = "/World/camera"

        def get_current_frame(self):
            return {"distance_to_image_plane": None}

    class World:
        steps = 0

        def step(self, render):
            self.steps += 1

    world = World()

    with pytest.raises(RuntimeError, match="after 2 rendering warmup steps"):
        _prime_stereo_cameras(world, Camera(), Camera(), max_steps=2)

    assert world.steps == 2


def test_physics_warmup_steps_without_rendering_and_requires_positive_count():
    """OMap must run after PhysX colliders are registered, before the formal loop."""
    events = []

    class World:
        def step(self, render):
            assert render is False
            events.append(1)

    _warmup_physics_scene(World(), steps=4)
    assert events == [1, 1, 1, 1]

    with pytest.raises(ValueError, match="positive"):
        _warmup_physics_scene(World(), steps=0)


def test_metadata_emits_required_source_to_target_stereo_extrinsics():
    """Missing or ambiguously directed calibration matrices violate the stereo contract."""
    contract = load_contract(CONTRACT_PATH)
    args = SimpleNamespace(
        contract=CONTRACT_PATH,
        scene="",
        img_w=640,
        img_h=480,
        ball_radius=0.4,
        bev_teacher="analytic",
        elevation_teacher="auto",
        no_save_elevation=False,
        sign_probe="none",
        sign_probe_steps=100,
    )
    left_optical_to_base = np.eye(4)
    left_optical_to_base[:3, 3] = [1.0, 2.0, 3.0]
    right_optical_to_base = np.eye(4)
    right_optical_to_base[:3, 3] = [4.0, 6.0, 3.0]

    metadata = _episode_metadata(
        args,
        contract,
        episode=0,
        waypoints=[],
        left_intrinsics=np.eye(3),
        right_intrinsics=np.eye(3),
        left_optical_to_base=left_optical_to_base,
        right_optical_to_base=right_optical_to_base,
    )

    assert metadata["action_protocol_id"] == contract.action_protocol_id
    assert metadata["controls_per_block"] == contract.controls_per_block
    assert metadata["action_semantics"] == (
        "slew_limited_target; recorded after completed physics step"
    )
    execution = metadata["action_execution"]
    assert execution["main_acceleration_limit_rad_s2"] == pytest.approx(0.5)
    assert execution["auxiliary_target_rate_limit_rad_s"] == pytest.approx(
        math.radians(30.0)
    )
    assert execution["articulation_gains"] == {
        "main_kp": 0.0,
        "main_kd": 80.0,
        "auxiliary_kp": 300.0,
        "auxiliary_kd": 60.0,
    }
    assert metadata["extrinsic_matrix_convention"] == (
        "column vectors; source_to_target maps p_target = T_source_to_target @ p_source; "
        "left/right use OpenCV optical x_right_y_down_z_forward"
    )
    assert metadata["intrinsics_matrix_convention"] == (
        "column vectors; p_image_homogeneous ~ K_camera_optical_to_image @ "
        "p_camera_optical"
    )
    assert metadata["camera_pose_storage"] == (
        "column vectors; camera USD local to world, "
        "p_world = T_camera_usd_to_world @ p_camera_usd"
    )
    np.testing.assert_allclose(metadata["base_to_left_extrinsic"][:3, 3], [-1.0, -2.0, -3.0])
    np.testing.assert_allclose(metadata["base_to_right_extrinsic"][:3, 3], [-4.0, -6.0, -3.0])
    np.testing.assert_allclose(metadata["left_to_right_extrinsic"][:3, 3], [-3.0, -4.0, 0.0])
    np.testing.assert_array_equal(metadata["left_optical_to_base_extrinsic"], left_optical_to_base)
    np.testing.assert_array_equal(metadata["right_optical_to_base_extrinsic"], right_optical_to_base)


def test_cli_defaults_come_from_fixed_contract():
    args, contract = _parse_args([])

    assert (args.img_w, args.img_h) == (contract.image_width_px, contract.image_height_px)
    assert args.bev_extent_m == contract.bev.extent_m
    assert args.bev_resolution_m == contract.bev.resolution_m
    assert args.min_turn_r == pytest.approx(
        args.ball_radius / math.tan(contract.action_maximum[1])
    )
    assert args.main_accel_rad_s2 == pytest.approx(0.5)
    assert args.aux_kp == pytest.approx(300.0)
    assert args.aux_kd == pytest.approx(60.0)
    assert args.aux_rate_deg_s == pytest.approx(30.0)


def test_cli_enables_contract_elevation_by_default():
    args, contract = _parse_args([])

    assert args.elevation_teacher == "auto"
    assert args.elevation_teacher in contract.elevation_teacher_modes
    assert args.no_save_elevation is False


def test_metadata_records_elevation_contract_and_files_when_enabled():
    contract = load_contract(CONTRACT_PATH)
    args = SimpleNamespace(
        contract=CONTRACT_PATH,
        scene="",
        img_w=640,
        img_h=480,
        ball_radius=0.4,
        bev_teacher="analytic",
        elevation_teacher="raycast",
        no_save_elevation=False,
        sign_probe="none",
        sign_probe_steps=100,
    )

    metadata = _episode_metadata(
        args,
        contract,
        episode=0,
        waypoints=[],
        left_intrinsics=np.eye(3),
        right_intrinsics=np.eye(3),
        left_optical_to_base=np.eye(4),
        right_optical_to_base=np.eye(4),
    )

    assert metadata["elevation_enabled"] is True
    assert metadata["elevation_teacher"] == "raycast"
    assert metadata["elevation_frame"] == "base_xy_yaw_with_world_gravity_height"
    assert metadata["elevation_pose_policy"] == "yaw_only; body_roll_pitch_ignored"
    assert metadata["elevation_shape"] == list(contract.bev.shape)
    assert metadata["elevation_unit"] == "m"
    assert metadata["elevation_height_reference"] == "z_world - z_base_world"
    assert metadata["elevation_dtype"] == contract.elevation_dtype
    assert metadata["elevation_invalid_value"] == contract.elevation_invalid_value
    assert metadata["elevation_file_naming"] == {
        "height_min_bev": "height_min_bev_{stereo_frame_id:06d}.npy",
        "height_max_bev": "height_max_bev_{stereo_frame_id:06d}.npy",
        "height_valid_bev": "height_valid_bev_{stereo_frame_id:06d}.png",
    }
    assert metadata["file_naming"]["height_min_bev"] == "height_min_bev_{stereo_frame_id:06d}.npy"
    assert metadata["file_naming"]["height_max_bev"] == "height_max_bev_{stereo_frame_id:06d}.npy"
    assert metadata["file_naming"]["height_valid_bev"] == "height_valid_bev_{stereo_frame_id:06d}.png"


def test_metadata_disables_elevation_when_no_save_elevation_is_requested():
    contract = load_contract(CONTRACT_PATH)
    args = SimpleNamespace(
        contract=CONTRACT_PATH,
        scene="",
        img_w=640,
        img_h=480,
        ball_radius=0.4,
        bev_teacher="analytic",
        elevation_teacher="auto",
        no_save_elevation=True,
        sign_probe="none",
        sign_probe_steps=100,
    )

    metadata = _episode_metadata(
        args,
        contract,
        episode=0,
        waypoints=[],
        left_intrinsics=np.eye(3),
        right_intrinsics=np.eye(3),
        left_optical_to_base=np.eye(4),
        right_optical_to_base=np.eye(4),
    )

    assert metadata["elevation_enabled"] is False


def test_elevation_accepts_expected_rolling_tilt_and_returns_gravity_aligned_yaw():
    """Rolling tilt is valid; elevation crops use only the horizontal yaw."""
    yaw = 0.3
    roll = math.radians(11.349)
    half_yaw = yaw / 2.0
    half_roll = roll / 2.0
    # q_yaw * q_roll: a finite tilted body orientation with known yaw.
    quaternion = np.array(
        [
            math.cos(half_yaw) * math.cos(half_roll),
            math.cos(half_yaw) * math.sin(half_roll),
            math.sin(half_yaw) * math.sin(half_roll),
            math.sin(half_yaw) * math.cos(half_roll),
        ]
    )

    result_yaw = _assert_planar_elevation_pose(np.array([1.0, -2.0, 0.45]), quaternion)

    assert result_yaw == pytest.approx(yaw)


@pytest.mark.parametrize(
    "position",
    [
        np.array([1.0, 2.0]),
        np.array([1.0, np.nan, 0.0]),
        np.array([1.0, np.inf, 0.0]),
    ],
)
def test_elevation_rejects_malformed_base_position(position):
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    with pytest.raises(RuntimeError, match="finite base position"):
        _assert_planar_elevation_pose(position, quaternion)


@pytest.mark.parametrize(
    "quaternion",
    [
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, np.nan, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0]),
    ],
)
def test_elevation_rejects_malformed_base_quaternion(quaternion):
    position = np.array([1.0, 2.0, 0.45])

    with pytest.raises(RuntimeError, match="finite base pose quaternion"):
        _assert_planar_elevation_pose(position, quaternion)


def test_cli_rejects_unknown_option_typo():
    with pytest.raises(SystemExit) as error:
        _parse_args(["--episode_lne", "25"])

    assert error.value.code == 2


@pytest.mark.parametrize(
    "override",
    [
        ["--img_w", "320"],
        ["--img_h", "240"],
        ["--bev_extent_m", "4.0"],
        ["--bev_resolution_m", "0.2"],
    ],
)
def test_cli_rejects_noncontract_image_and_bev_values_before_isaac(override):
    """Accepting these values would fail later at the fixed 80x80 writer boundary."""
    with pytest.raises(SystemExit) as error:
        _parse_args(override)

    assert error.value.code == 2
