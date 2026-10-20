"""CPU-only tests for the thin tracking-acceptance diagnostic harness."""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from benchmark.manifest import BenchmarkEpisode, HumanReview
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig
from benchmark.tracking_acceptance import build_tracking_reference, load_tracking_cases
from scripts.bev_ground_truth import GlobalOccupancy
from scripts import run_tracking_acceptance as harness


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "tracking_acceptance.json"
)

V2_ACTION_TRACE_NAMES = (
    "projection_s_m",
    "delay_distance_m",
    "query_s_m",
    "pure_pursuit_curvature_1_m",
    "profile_curvature_1_m",
    "blend_weight",
    "budget_infeasible",
    "terminal_runway_s",
    "elapsed_s",
)
V2_TERMINAL_TRACE_NAMES = (
    "terminal_brake_start_s",
    "goal_entry_s",
    "stop_below_0p05_s",
    "hold_complete_s",
)


def _fake_episode(reference) -> BenchmarkEpisode:
    start = tuple(float(value) for value in reference.points_xy[0])
    goal = tuple(float(value) for value in reference.points_xy[-1])
    distance = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    return BenchmarkEpisode(
        episode_id=f"tracking_{reference.case.case_id}",
        split="dev",
        difficulty="E0",
        scene_uri="${SCENE_ROOT}/tracking_open_plane.usda",
        scene_sha256="11" * 32,
        occupancy_sha256="22" * 32,
        start_position_world_m=(start[0], start[1], 0.45),
        start_quaternion_world_wxyz=(1.0, 0.0, 0.0, 0.0),
        goal_position_world_m=(goal[0], goal[1], 0.45),
        euclidean_distance_m=max(distance, 1e-3),
        oracle_path_length_m=max(reference.path_length_m, distance, 1e-3),
        seed=3072,
        human_review=HumanReview("approved", "project_owner", "fixture"),
        classification_evidence={"diagnostic": True},
    )


def _fake_result(reference, *, action_count: int = 6):
    state_count = action_count + 1
    positions = np.linspace(reference.points_xy[0], reference.points_xy[-1], state_count)
    return SimpleNamespace(
        episode_id="tracking_fixture",
        scene_id="tracking_open_plane",
        difficulty="E0",
        policy_name="oracle",
        termination_reason="success",
        action_blocks=action_count,
        elapsed_s=action_count / 10.0,
        elapsed_s_action_aligned=np.arange(action_count, dtype=np.float64) / 10.0,
        final_distance_m=0.0,
        path_length_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        reference_length_m=reference.path_length_m,
        spl=1.0,
        goal_progress_m=reference.path_length_m,
        minimum_clearance_m=1.0,
        time_to_goal_s=0.2,
        runtime_error=None,
        positions_world_xy=positions,
        actions=np.zeros((action_count, 10), dtype=np.float32),
        clearances_m=np.ones(state_count, dtype=np.float64),
        physical_contacts=np.zeros(state_count, dtype=bool),
        collisions=np.zeros(state_count, dtype=bool),
        yaw_world_rad=np.zeros(state_count, dtype=np.float64),
        linear_velocity_world_m_s=np.zeros((state_count, 3), dtype=np.float64),
        angular_velocity_world_rad_s=np.zeros((state_count, 3), dtype=np.float64),
        joint_position_rad=np.zeros((state_count, 3), dtype=np.float64),
        joint_velocity_rad_s=np.zeros((state_count, 3), dtype=np.float64),
        applied_action=np.zeros((state_count, 2), dtype=np.float64),
        policy_reference_index=np.arange(action_count, dtype=np.int64),
        policy_cross_track_error_m=np.zeros(action_count, dtype=np.float64),
        policy_active_drive_direction=np.ones(action_count, dtype=np.int8),
        policy_desired_drive_direction=np.ones(action_count, dtype=np.int8),
        policy_pending_drive_direction=np.zeros(action_count, dtype=np.int8),
        policy_stopped_cycles=np.zeros(action_count, dtype=np.int64),
        policy_braking_state_code=np.ones(action_count, dtype=np.int8),
        simulation_time_s=np.arange(state_count, dtype=np.float64) / 10.0,
        physics_step_index=np.arange(state_count, dtype=np.int64) * 5,
        raw_simulation_time_s=12.0 + np.arange(state_count, dtype=np.float64) / 10.0,
        raw_physics_step_index=100 + np.arange(state_count, dtype=np.int64) * 5,
        roll_world_rad=np.zeros(state_count, dtype=np.float64),
        diagnostic_stop_hold_passed=True,
        diagnostic_goal_entry_action_block=1,
        diagnostic_settling_blocks=5,
        diagnostic_settling_runtime_error=None,
        diagnostic_settling_mask=np.asarray(
            [False] + [True] * (action_count - 1), dtype=bool
        ),
        diagnostic_terminal_brake_start_time_s=None,
        diagnostic_goal_entry_time_s=None,
        diagnostic_stop_below_speed_time_s=None,
        diagnostic_hold_complete_time_s=None,
    )


def _fake_physics_diagnostics(action_blocks: int = 6) -> dict[str, np.ndarray]:
    """Mirror every field exposed by IsaacBenchmarkRuntime.physics_diagnostics."""

    sample_count = 1 + 5 * action_blocks
    sample = np.arange(sample_count, dtype=np.float64)
    root_position = np.column_stack(
        (0.05 * sample, -0.02 * sample, np.full(sample.shape, 0.45))
    )
    root_position[0] = (0.0, 0.0, 0.45)
    root_quaternion = np.tile(
        np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64), (sample_count, 1)
    )
    linear_velocity = np.column_stack((0.01 * sample, -0.005 * sample, 0.0 * sample))
    angular_velocity = np.column_stack((0.0 * sample, 0.002 * sample, -0.001 * sample))
    joint_position = np.column_stack((0.01 * sample, -0.02 * sample, 0.03 * sample))
    joint_velocity = np.column_stack((0.1 * sample, -0.2 * sample, 0.3 * sample))
    raw_target = np.column_stack((0.4 + 0.01 * sample, -0.2 - 0.01 * sample))
    clipped_target = np.clip(raw_target, (-2.0, -0.5), (2.0, 0.5))
    applied_action = 0.5 * clipped_target
    raw_target[0] = (0.0, 0.0)
    clipped_target[0] = (0.0, 0.0)
    applied_action[0] = (0.0, 0.0)
    return {
        "raw_command_target": raw_target.astype(np.float32),
        "clipped_target": clipped_target.astype(np.float32),
        "applied_action": applied_action.astype(np.float32),
        "joint_position_rad": joint_position.astype(np.float32),
        "joint_velocity_rad_s": joint_velocity.astype(np.float32),
        "position_world_xy": root_position[:, :2].astype(np.float32),
        "root_position_world_m": root_position.astype(np.float32),
        "root_quaternion_world_wxyz": root_quaternion.astype(np.float32),
        "linear_velocity_world_m_s": linear_velocity.astype(np.float32),
        "angular_velocity_world_rad_s": angular_velocity.astype(np.float32),
        "yaw_world_rad": np.zeros(sample_count, dtype=np.float32),
        "roll_world_rad": np.zeros(sample_count, dtype=np.float32),
        "simulation_time_s": (0.02 * sample).astype(np.float64),
        "physics_step_index": np.arange(sample_count, dtype=np.int64),
        "raw_simulation_time_s": (12.0 + 0.02 * sample).astype(np.float64),
        "raw_physics_step_index": (100 + np.arange(sample_count)).astype(np.int64),
        "action_block_index": np.concatenate(
            (np.asarray((0,), dtype=np.int64), np.repeat(np.arange(action_blocks), 5))
        ),
        "physics_step_in_block": np.concatenate(
            (np.asarray((0,), dtype=np.int64), np.tile(np.arange(1, 6), action_blocks))
        ),
    }


def _fake_runtime(tmp_path: Path):
    occupancy = GlobalOccupancy(
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=0.1,
    )
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    return SimpleNamespace(
        occupancy=occupancy,
        captured_frames={"third_person": (frame, frame), "stereo_view": (frame, frame)},
        contact_measurement_available=True,
        config=SimpleNamespace(drive_profile_path=tmp_path / "drive_dynamics_v1.json"),
        physics_diagnostics=_fake_physics_diagnostics(),
    )


def _expected_producer_diagnostics() -> dict[str, np.ndarray]:
    """Literal independent oracle for one reset row and one five-step block."""

    return {
        "raw_command_target": np.asarray(
            [
                [0.0, 0.0],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
            ],
            dtype=np.float32,
        ),
        "clipped_target": np.asarray(
            [
                [0.0, 0.0],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
                [0.4, -0.2],
            ],
            dtype=np.float32,
        ),
        "applied_action": np.asarray(
            [
                [0.0, 0.0],
                [0.2, -0.1],
                [0.2, -0.1],
                [0.2, -0.1],
                [0.2, -0.1],
                [0.2, -0.1],
            ],
            dtype=np.float32,
        ),
        "joint_position_rad": np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.02, -0.01, 0.005],
                [0.04, -0.02, 0.01],
                [0.06, -0.03, 0.015],
                [0.08, -0.04, 0.02],
                [0.10, -0.05, 0.025],
            ],
            dtype=np.float32,
        ),
        "joint_velocity_rad_s": np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.2, -0.1, 0.05],
                [0.4, -0.2, 0.10],
                [0.6, -0.3, 0.15],
                [0.8, -0.4, 0.20],
                [1.0, -0.5, 0.25],
            ],
            dtype=np.float32,
        ),
        "position_world_xy": np.asarray(
            [
                [0.0, 0.0],
                [0.01, -0.02],
                [0.02, -0.04],
                [0.03, -0.06],
                [0.04, -0.08],
                [0.05, -0.10],
            ],
            dtype=np.float32,
        ),
        "root_position_world_m": np.asarray(
            [
                [0.0, 0.0, 0.45],
                [0.01, -0.02, 0.451],
                [0.02, -0.04, 0.452],
                [0.03, -0.06, 0.453],
                [0.04, -0.08, 0.454],
                [0.05, -0.10, 0.455],
            ],
            dtype=np.float32,
        ),
        "root_quaternion_world_wxyz": np.asarray(
            [
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        ),
        "linear_velocity_world_m_s": np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.1, -0.05, 0.01],
                [0.2, -0.10, 0.02],
                [0.3, -0.15, 0.03],
                [0.4, -0.20, 0.04],
                [0.5, -0.25, 0.05],
            ],
            dtype=np.float32,
        ),
        "angular_velocity_world_rad_s": np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.01, 0.02, -0.03],
                [0.02, 0.04, -0.06],
                [0.03, 0.06, -0.09],
                [0.04, 0.08, -0.12],
                [0.05, 0.10, -0.15],
            ],
            dtype=np.float32,
        ),
        "yaw_world_rad": np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64
        ),
        "roll_world_rad": np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64
        ),
        "simulation_time_s": np.asarray(
            [0.0, 0.02, 0.04, 0.06, 0.08, 0.10], dtype=np.float64
        ),
        "physics_step_index": np.asarray([0, 1, 2, 3, 4, 5], dtype=np.int64),
        "raw_simulation_time_s": np.asarray(
            [40.06, 40.08, 40.10, 40.12, 40.14, 40.16], dtype=np.float64
        ),
        "raw_physics_step_index": np.asarray(
            [203, 204, 205, 206, 207, 208], dtype=np.int64
        ),
        "action_block_index": np.asarray([0, 0, 0, 0, 0, 0], dtype=np.int64),
        "physics_step_in_block": np.asarray([0, 1, 2, 3, 4, 5], dtype=np.int64),
    }


def _producer_artifact_case(tmp_path: Path):
    """Exercise the real runtime recorder with a deterministic fake world."""

    class FakeRobot:
        def __init__(self) -> None:
            self.world: FakeWorld | None = None
            self._position = np.asarray((9.0, -4.0, 0.45), dtype=np.float64)
            self._quaternion = np.asarray((0.98, 0.0, 0.0, 0.2), dtype=np.float64)
            self._linear_velocity = np.asarray((0.7, -0.2, 0.1), dtype=np.float64)
            self._angular_velocity = np.asarray((0.1, 0.2, -0.3), dtype=np.float64)
            self._joint_positions = np.asarray((0.4, -0.2, 0.1), dtype=np.float64)
            self._joint_velocities = np.asarray((0.6, -0.5, 0.3), dtype=np.float64)
            self._pending_state: dict[str, np.ndarray] = {}
            self._sample = 0

        def _camera_prime_drift(self) -> None:
            self._position += np.asarray((0.003, -0.002, 0.0))
            self._linear_velocity += np.asarray((0.02, -0.01, 0.0))
            self._angular_velocity += np.asarray((0.01, -0.02, 0.03))
            self._joint_positions += np.asarray((0.004, -0.003, 0.002))
            self._joint_velocities += np.asarray((0.03, -0.02, 0.01))

        def _apply_pending_state(self) -> bool:
            had_pending = bool(self._pending_state)
            for name, value in self._pending_state.items():
                setattr(self, name, value.copy())
            if "_position" in self._pending_state:
                self._sample = 0
            self._pending_state.clear()
            return had_pending

        def _advance_physics(self) -> None:
            self._sample += 1
            sample = float(self._sample)
            self._position = np.asarray(
                (0.01 * sample, -0.02 * sample, 0.45 + 0.001 * sample),
                dtype=np.float64,
            )
            self._linear_velocity = np.asarray(
                (0.1 * sample, -0.05 * sample, 0.01 * sample), dtype=np.float64
            )
            self._angular_velocity = np.asarray(
                (0.01 * sample, 0.02 * sample, -0.03 * sample), dtype=np.float64
            )
            self._joint_positions = np.asarray(
                (0.02 * sample, -0.01 * sample, 0.005 * sample), dtype=np.float64
            )
            self._joint_velocities = np.asarray(
                (0.2 * sample, -0.1 * sample, 0.05 * sample), dtype=np.float64
            )

        def set_world_pose(self, *, position: object, orientation: object) -> None:
            self._pending_state["_position"] = np.asarray(position, dtype=np.float64)
            self._pending_state["_quaternion"] = np.asarray(
                orientation, dtype=np.float64
            )

        def set_linear_velocity(self, value: object) -> None:
            self._pending_state["_linear_velocity"] = np.asarray(
                value, dtype=np.float64
            )

        def set_angular_velocity(self, value: object) -> None:
            self._pending_state["_angular_velocity"] = np.asarray(
                value, dtype=np.float64
            )

        def set_joint_positions(self, value: object) -> None:
            self._pending_state["_joint_positions"] = np.asarray(
                value, dtype=np.float64
            )

        def set_joint_velocities(self, value: object) -> None:
            self._pending_state["_joint_velocities"] = np.asarray(
                value, dtype=np.float64
            )

        def get_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
            return self._position.copy(), self._quaternion.copy()

        def get_linear_velocity(self) -> np.ndarray:
            return self._linear_velocity.copy()

        def get_angular_velocity(self) -> np.ndarray:
            return self._angular_velocity.copy()

        def get_joint_positions(self) -> np.ndarray:
            return self._joint_positions.copy()

        def get_joint_velocities(self) -> np.ndarray:
            return self._joint_velocities.copy()

    class FakeWorld:
        def __init__(self) -> None:
            self.current_time = 40.0
            self.current_time_step_index = 200
            self.step_calls: list[bool] = []
            self.render_calls = 0
            self._prime_complete = False
            self.robot: FakeRobot | None = None

        def step(self, *, render: bool) -> None:
            self.step_calls.append(bool(render))
            self.current_time += 0.02
            self.current_time_step_index += 1
            if self.robot is None:
                return
            pending_applied = self.robot._apply_pending_state()
            if render and not self._prime_complete:
                self.robot._camera_prime_drift()
            elif not render and self._prime_complete and not pending_applied:
                self.robot._advance_physics()

        def render(self) -> None:
            self.render_calls += 1

        def mark_prime_complete(self) -> None:
            self._prime_complete = True

    class FakeSmoother:
        def __init__(self) -> None:
            self._current_action = np.asarray((0.75, -0.4), dtype=np.float32)
            self.reset_calls = 0

        @property
        def current_action(self) -> np.ndarray:
            return self._current_action.copy()

        def reset(self) -> None:
            self.reset_calls += 1
            self._current_action.fill(0.0)

        def step(
            self,
            target: object,
            _minimum: object,
            _maximum: object,
        ) -> np.ndarray:
            self._current_action = 0.5 * np.asarray(target, dtype=np.float32)
            return self._current_action.copy()

    class FakeController:
        def __init__(self) -> None:
            self.actions: list[object] = []

        def apply_action(self, action: object) -> None:
            self.actions.append(action)

    class FakeAction:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[0])
    fixture = tmp_path / "tracking_open_plane.usda"
    occupancy_path = tmp_path / "tracking_open_plane_free.npz"
    contract_path = tmp_path / "data_contract.yaml"
    drive_profile = tmp_path / "drive_dynamics_v1.json"
    fixture.write_text("#usda 1.0\n", encoding="utf-8")
    np.savez(
        occupancy_path,
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    contract_path.write_text("contract", encoding="utf-8")
    drive_profile.write_text("{}", encoding="utf-8")
    episode = replace(
        _fake_episode(reference),
        scene_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
        occupancy_sha256=hashlib.sha256(occupancy_path.read_bytes()).hexdigest(),
    )

    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            scene_root=tmp_path,
            contract_path=contract_path,
            gui=False,
            record_video=False,
            occupancy_path=occupancy_path,
            drive_profile_path=drive_profile,
            diagnostic_physics_trace=True,
        )
    )
    world = FakeWorld()
    robot = FakeRobot()
    robot.world = world
    world.robot = robot
    smoother = FakeSmoother()
    controller = FakeController()
    runtime._world = world
    runtime._robot = robot
    runtime._action_smoother = smoother
    runtime._controller = controller
    runtime._action_protocol = SimpleNamespace(
        controls_per_block=5,
        minimum=(-2.0, -0.5),
        maximum=(2.0, 0.5),
        clip_block=lambda action: np.asarray(action, dtype=np.float32).copy(),
    )
    runtime._ArticulationAction = FakeAction
    runtime._occupancy = GlobalOccupancy(
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=0.1,
    )
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    runtime._captured_frames = {
        "third_person": [frame.copy(), frame.copy()],
        "stereo_view": [frame.copy(), frame.copy()],
    }

    def initialize(_scene_path: Path, _occupancy_path: Path, _episode) -> None:
        for _ in range(2):
            world.step(render=True)
        world.mark_prime_complete()

    runtime._initialize = initialize
    runtime._capture = lambda: SimpleNamespace(observation=None)
    runtime.reset(episode)
    result = _fake_result(reference, action_count=1)
    result.positions_world_xy = np.asarray(
        (reference.points_xy[0], reference.points_xy[-1]), dtype=np.float64
    )
    runtime.execute(np.asarray([0.4, -0.2] * 5, dtype=np.float32))
    config = {
        "schema_version": "tracking_acceptance_v1",
        "case_id": reference.case.case_id,
        "lookahead_m": 1.0,
        "harness": {
            "scene_root": str(tmp_path),
            "fixture_path": str(fixture),
            "occupancy_path": str(occupancy_path),
            "contract_path": str(contract_path),
            "drive_profile_path": str(drive_profile),
        },
    }
    return reference, result, episode, runtime, config, world


def _valid_mixed_boundary_trace(reference):
    positions = reference.points_xy.copy()
    state_count = positions.shape[0]
    action_count = state_count - 1
    speeds = np.full(state_count, 0.01, dtype=np.float64)
    actions = np.zeros((action_count, 10), dtype=np.float64)
    first, second = (marker.point_index for marker in reference.stop_markers)
    actions[: first - 5, 0] = 0.5
    actions[first, 0] = -0.5
    actions[first + 1 : second - 5, 0] = -0.5
    actions[second, 0] = 0.5
    actions[second + 1 :, 0] = 0.5
    active = np.ones(action_count, dtype=np.int8)
    active[first:second] = -1
    active[second:] = 1
    desired = active.copy()
    return positions, speeds, actions, active, desired


def test_harness_timing_validation_is_fail_closed_and_allows_isaac_float32_tolerance():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[0])
    result = _fake_result(reference)

    harness.validate_diagnostic_timing(result)

    result.simulation_time_s = result.simulation_time_s.copy()
    result.simulation_time_s[2] += 5e-6
    harness.validate_diagnostic_timing(result)

    result.simulation_time_s[2] += 2e-4
    with pytest.raises(harness.DiagnosticValidationError, match="simulation_time_s"):
        harness.validate_diagnostic_timing(result)

    result.simulation_time_s[2] = np.nan
    with pytest.raises(harness.DiagnosticValidationError, match="finite"):
        harness.validate_diagnostic_timing(result)


def test_harness_mixed_boundary_validation_requires_real_stop_and_direction_change():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])
    positions = reference.points_xy.copy()
    speeds = np.full(positions.shape[0], 0.01, dtype=np.float64)
    # Deliberately claim progression by index only; no actual direction trace
    # must be accepted as a mixed-direction pass.
    with pytest.raises(harness.DiagnosticValidationError, match="direction"):
        harness.validate_direction_boundaries(
            reference,
            positions_world_xy=positions,
            speed_m_s=speeds,
            active_direction_codes=np.ones(positions.shape[0], dtype=np.int8),
        )


def test_mixed_boundaries_require_five_precommand_zero_blocks_near_cusp():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])
    positions, speeds, actions, active, desired = _valid_mixed_boundary_trace(reference)
    first = reference.stop_markers[0].point_index
    actions[first - 1, 0] = 0.5

    evidence = harness.validate_direction_boundaries(
        reference,
        positions_world_xy=positions,
        speed_m_s=speeds,
        active_direction_codes=active,
        desired_direction_codes=desired,
        actions=actions,
    )

    assert evidence["all_passed"] is False
    assert evidence["boundaries"][0]["stop_hold_passed"] is False


def test_mixed_boundaries_require_local_direction_transition_at_signed_command():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])
    positions, speeds, actions, active, desired = _valid_mixed_boundary_trace(reference)
    first = reference.stop_markers[0].point_index
    active[first : first + 3] = 1
    desired[first : first + 3] = 1

    evidence = harness.validate_direction_boundaries(
        reference,
        positions_world_xy=positions,
        speed_m_s=speeds,
        active_direction_codes=active,
        desired_direction_codes=desired,
        actions=actions,
    )

    assert evidence["all_passed"] is False
    assert evidence["boundaries"][0]["active_direction_changed"] is False
    assert evidence["boundaries"][0]["desired_direction_changed"] is False


def test_mixed_desired_transition_may_lead_active_switch_within_zero_braking_episode():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])
    positions, speeds, actions, active, desired = _valid_mixed_boundary_trace(reference)
    first = reference.stop_markers[0].point_index
    actions[first - 10 : first] = 0.0
    desired[first - 10 : first] = -1

    evidence = harness.validate_direction_boundaries(
        reference,
        positions_world_xy=positions,
        speed_m_s=speeds,
        active_direction_codes=active,
        desired_direction_codes=desired,
        actions=actions,
    )

    assert evidence["all_passed"] is True
    assert evidence["boundaries"][0]["active_direction_changed"] is True
    assert evidence["boundaries"][0]["desired_direction_changed"] is True


def test_mixed_boundaries_reject_skipped_cusps_even_with_final_reference_index():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])
    positions = np.repeat(reference.points_xy[-1][None, :], reference.points_xy.shape[0], axis=0)
    speeds = np.full(positions.shape[0], 0.01, dtype=np.float64)
    actions = np.zeros((positions.shape[0] - 1, 10), dtype=np.float64)
    active = np.ones(positions.shape[0] - 1, dtype=np.int8)
    desired = active.copy()

    evidence = harness.validate_direction_boundaries(
        reference,
        positions_world_xy=positions,
        speed_m_s=speeds,
        active_direction_codes=active,
        desired_direction_codes=desired,
        policy_reference_index=np.full(positions.shape[0], reference.points_xy.shape[0] - 1),
        actions=actions,
    )

    assert evidence["all_passed"] is False
    assert all(not item["boundary_reached"] for item in evidence["boundaries"])


def test_harness_acceptance_requires_nominal_success_termination():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[0])
    result = _fake_result(reference)
    start = reference.points_xy[0]
    goal = reference.points_xy[-1]
    result.positions_world_xy = np.vstack(
        (
            start,
            start + 0.5 * (goal - start),
            np.repeat(goal[None, :], 6, axis=0),
        )
    )
    state_count = result.positions_world_xy.shape[0]
    action_count = state_count - 1
    result.actions = np.zeros((action_count, 10), dtype=np.float32)
    result.clearances_m = np.ones(state_count, dtype=np.float64)
    result.physical_contacts = np.zeros(state_count, dtype=bool)
    result.collisions = np.zeros(state_count, dtype=bool)
    result.yaw_world_rad = np.zeros(state_count, dtype=np.float64)
    result.linear_velocity_world_m_s = np.zeros((state_count, 3), dtype=np.float64)
    result.joint_position_rad = np.zeros((state_count, 3), dtype=np.float64)
    result.joint_velocity_rad_s = np.zeros((state_count, 3), dtype=np.float64)
    result.roll_world_rad = np.zeros(state_count, dtype=np.float64)
    result.policy_reference_index = np.arange(action_count, dtype=np.int64)
    result.policy_cross_track_error_m = np.zeros(action_count, dtype=np.float64)
    result.policy_active_drive_direction = np.ones(action_count, dtype=np.int8)
    result.policy_desired_drive_direction = np.ones(action_count, dtype=np.int8)
    result.simulation_time_s = np.arange(state_count, dtype=np.float64) / 10.0
    result.physics_step_index = np.arange(state_count, dtype=np.int64) * 5
    result.raw_simulation_time_s = 12.0 + result.simulation_time_s
    result.raw_physics_step_index = 100 + result.physics_step_index
    result.termination_reason = "timeout"

    metrics = harness.compute_tracking_metrics(reference.case, reference, result)

    assert metrics["acceptance"]["nominal_success"] is False
    assert metrics["acceptance"]["complete"] is False
    assert metrics["failure_stage"] == "goal"


def _artifact_case(tmp_path: Path):
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[0])
    result = _fake_result(reference)
    episode = _fake_episode(reference)
    fixture = tmp_path / "tracking_open_plane.usda"
    occupancy = tmp_path / "tracking_open_plane_free.npz"
    contract = tmp_path / "data_contract.yaml"
    fixture.write_text("#usda 1.0\n", encoding="utf-8")
    np.savez(
        occupancy,
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    contract.write_text("contract", encoding="utf-8")
    drive_profile = tmp_path / "drive_dynamics_v1.json"
    drive_profile.write_text("{}", encoding="utf-8")
    runtime = _fake_runtime(tmp_path)
    config = {
        "schema_version": "tracking_acceptance_v1",
        "case_id": reference.case.case_id,
        "lookahead_m": 1.0,
        "harness": {
            "scene_root": str(tmp_path),
            "fixture_path": str(fixture),
            "occupancy_path": str(occupancy),
            "contract_path": str(contract),
            "drive_profile_path": str(drive_profile),
        },
    }
    return reference, result, episode, runtime, config


def test_harness_writes_required_artifacts_and_detects_config_tampering(tmp_path: Path):
    (
        reference,
        result,
        episode,
        runtime,
        config,
        world,
    ) = _producer_artifact_case(tmp_path)
    fixture = Path(config["harness"]["fixture_path"])
    contract = Path(config["harness"]["contract_path"])
    drive_profile = Path(config["harness"]["drive_profile_path"])

    assert world.step_calls == [True, True] + [False] * 6
    assert world.render_calls == 1

    output = harness.write_tracking_artifacts(
        tmp_path / "run",
        case=reference.case,
        reference=reference,
        episode=episode,
        result=result,
        runtime=runtime,
        harness_config=config,
    )

    assert {
        "states.npz",
        "commands.npz",
        "physics_diagnostics.npz",
        "trajectory.png",
        "third_person.mp4",
        "config.json",
        "config.sha256",
        "metrics.json",
    }.issubset(path.name for path in output.iterdir())
    assert harness.verify_config_hash(output)
    config_payload = json.loads((output / "config.json").read_text(encoding="utf-8"))
    assert config_payload["lookahead_m"] == pytest.approx(1.0)
    assert config_payload["fixture_sha256"] == harness._sha256(fixture)
    assert config_payload["contract_sha256"] == harness._sha256(contract)
    assert config_payload["drive_profile_sha256"] == harness._sha256(
        drive_profile
    )
    assert config_payload["harness_sha256"] == harness._sha256(harness._HERE)
    metrics_payload = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    assert metrics_payload["failure_stage"] == "stop_hold"
    with np.load(output / "states.npz", allow_pickle=False) as states:
        assert states["cross_track_error_m"].shape == result.positions_world_xy.shape[:1]
        assert states["heading_error_rad"].shape == result.positions_world_xy.shape[:1]
    expected_diagnostic = _expected_producer_diagnostics()
    with np.load(output / "physics_diagnostics.npz", allow_pickle=False) as diagnostics:
        assert set(diagnostics.files) == set(expected_diagnostic)
        for field, expected in expected_diagnostic.items():
            if field in {"simulation_time_s", "raw_simulation_time_s"}:
                np.testing.assert_allclose(
                    diagnostics[field], expected, rtol=0.0, atol=5e-5
                )
            else:
                np.testing.assert_array_equal(diagnostics[field], expected)

    config_path = output / "config.json"
    config_path.write_text(config_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(harness.DiagnosticValidationError, match="config.sha256"):
        harness.verify_config_hash(output)


@pytest.mark.parametrize(
    "failure",
    (
        "extra",
        "missing",
        "nonfinite",
        "inconsistent",
        "physics_clock",
        "relative_clock",
        "raw_clock",
        "action_block",
        "physics_step_in_block",
        "state_action_count",
        "complex",
    ),
)
def test_harness_rejects_unauditable_physics_diagnostics(tmp_path: Path, failure: str):
    reference, result, episode, runtime, config = _artifact_case(tmp_path)
    diagnostics = runtime.physics_diagnostics
    if failure == "extra":
        diagnostics["unexpected"] = np.zeros(31, dtype=np.float32)
        expected_message = "exact 18"
    elif failure == "missing":
        diagnostics.pop("root_position_world_m")
        expected_message = "missing="
    elif failure == "nonfinite":
        diagnostics["root_position_world_m"][0, 2] = np.nan
        expected_message = "must be finite"
    elif failure == "inconsistent":
        diagnostics["joint_position_rad"] = np.zeros((6, 3), dtype=np.float32)
        expected_message = "same sample count"
    elif failure == "physics_clock":
        diagnostics["physics_step_index"][10] = 8
        expected_message = "physics_step_index"
    elif failure == "relative_clock":
        diagnostics["simulation_time_s"][10] += 0.01
        expected_message = "simulation_time_s"
    elif failure == "raw_clock":
        diagnostics["raw_simulation_time_s"][10] += 0.01
        expected_message = "raw_simulation_time_s"
    elif failure == "action_block":
        diagnostics["action_block_index"][6] = 0
        expected_message = "action_block_index"
    elif failure == "physics_step_in_block":
        diagnostics["physics_step_in_block"][6] = 2
        expected_message = "physics_step_in_block"
    elif failure == "state_action_count":
        result.positions_world_xy = result.positions_world_xy[:-1]
        expected_message = "state/action traces"
    else:
        diagnostics["root_position_world_m"] = diagnostics[
            "root_position_world_m"
        ].astype(np.complex64)
        diagnostics["root_position_world_m"][1, 0] += 1j
        expected_message = "must be real"

    with pytest.raises(harness.DiagnosticValidationError, match=expected_message):
        harness.write_tracking_artifacts(
            tmp_path / "run",
            case=reference.case,
            reference=reference,
            episode=episode,
            result=result,
            runtime=runtime,
            harness_config=config,
        )


def _write_production_shaped_summary_case(
    root: Path,
    case,
    *,
    policy_variant: str = "oracle_trajectory_v1",
):
    root.mkdir(parents=True, exist_ok=True)
    fixture = root / "tracking_open_plane.usda"
    occupancy = root / "tracking_open_plane_free.npz"
    contract = root / "data_contract.yaml"
    profile = root / "drive_dynamics_v1.json"
    raw_config = root / "tracking_acceptance.json"
    fixture.write_text("#usda 1.0\n", encoding="utf-8")
    np.savez(
        occupancy,
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    contract.write_text("contract\n", encoding="utf-8")
    profile.write_text("{}\n", encoding="utf-8")
    raw_config.write_bytes(CONFIG_PATH.read_bytes())
    reference = build_tracking_reference(case)
    episode = harness.build_tracking_episode(
        case,
        reference,
        scene_root=root,
        fixture_path=fixture,
        occupancy_path=occupancy,
        seed=3072,
    )
    if policy_variant == "oracle_trajectory_v1":
        parameters = dict(harness.TRAJECTORY_POLICY_DEFAULTS)
    elif policy_variant == "oracle_trajectory_v2":
        parameters = dict(harness.TRAJECTORY_POLICY_V2_DEFAULTS)
    else:
        parameters = {
            "lookahead_m": 1.0,
            "yaw_rate_feedback_gain_s": 0.15,
        }
    config = harness._case_config_payload(
        case,
        reference,
        episode,
        {
            "policy_variant": policy_variant,
            **parameters,
            "harness": {
                "scene_root": str(root),
                "fixture_path": str(fixture),
                "occupancy_path": str(occupancy),
                "contract_path": str(contract),
                "drive_profile_path": str(profile),
                "config_path": str(raw_config),
            },
        },
        fixture_path=fixture,
        occupancy_path=occupancy,
        contract_path=contract,
        profile_path=profile,
    )
    metrics = {
        "case_id": case.case_id,
        "probe": bool(case.probe),
        "episode_id": episode.episode_id,
        "policy_variant": policy_variant,
        "policy": (
            "oracle"
            if policy_variant == harness.DEFAULT_POLICY_VARIANT
            else policy_variant
        ),
        "action_blocks": 1,
        "acceptance": {"complete": False},
        **parameters,
    }
    if policy_variant in {"oracle_trajectory_v1", "oracle_trajectory_v2"}:
        metrics["trajectory_policy"] = {
            "name": policy_variant,
            "parameters": dict(parameters),
        }
    if policy_variant == "oracle_trajectory_v2":
        v2_trace_names = (
            "projection_s_m",
            "delay_distance_m",
            "query_s_m",
            "pure_pursuit_curvature_1_m",
            "profile_curvature_1_m",
            "blend_weight",
            "terminal_runway_s",
            "elapsed_s",
        )
        v2_terminal_names = (
            "terminal_brake_start_s",
            "goal_entry_s",
            "stop_below_0p05_s",
            "hold_complete_s",
        )
        metrics["trajectory_diagnostics"] = {
            name: [0.0]
            for name in (
                *harness.TRAJECTORY_DIAGNOSTIC_NAMES,
                *v2_trace_names,
            )
        }
        metrics["trajectory_diagnostics"]["budget_infeasible"] = [False]
        for name in v2_terminal_names:
            metrics["trajectory_diagnostics"][name] = [-1.0]
        metrics.update(
            {
                "terminal_brake_start_time_s": None,
                "goal_entry_time_s": None,
                "stop_below_speed_time_s": None,
                "hold_complete_time_s": None,
            }
        )
    case_dir = root / case.case_id
    case_dir.mkdir()
    config_bytes = harness._canonical_json_bytes(config)
    (case_dir / "config.json").write_bytes(config_bytes)
    config_sha256 = hashlib.sha256(config_bytes).hexdigest()
    (case_dir / "config.sha256").write_text(config_sha256 + "\n", encoding="utf-8")
    metrics["config_sha256"] = config_sha256
    if policy_variant == "oracle_trajectory_v2":
        metrics["tracking_config_sha256"] = config["tracking_config_sha256"]
    (case_dir / "metrics.json").write_bytes(harness._canonical_json_bytes(metrics))
    return case_dir, config, metrics, episode


def _rewrite_summary_case_config(case_dir: Path, config: dict[str, object]) -> None:
    config_bytes = harness._canonical_json_bytes(config)
    (case_dir / "config.json").write_bytes(config_bytes)
    (case_dir / "config.sha256").write_text(
        hashlib.sha256(config_bytes).hexdigest() + "\n", encoding="utf-8"
    )


def _rewrite_summary_case_evidence(
    case_dir: Path,
    config: dict[str, object],
    metrics: dict[str, object],
) -> None:
    """Rewrite both mutable evidence files after an intentional tamper."""

    _rewrite_summary_case_config(case_dir, config)
    config_sha256 = hashlib.sha256((case_dir / "config.json").read_bytes()).hexdigest()
    metrics["config_sha256"] = config_sha256
    if config.get("policy_variant") == harness.TRAJECTORY_POLICY_V2_VARIANT:
        metrics["tracking_config_sha256"] = config["tracking_config_sha256"]
    (case_dir / "metrics.json").write_bytes(harness._canonical_json_bytes(metrics))


def test_summary_accepts_production_shaped_case_evidence(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    _write_production_shaped_summary_case(tmp_path, case)

    summary_path = harness._summary(
        tmp_path,
        (case,),
        policy_variant="oracle_trajectory_v1",
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["cases"][0]["case_id"] == case.case_id
    assert summary["cases"][0]["artifact_provenance"]["occupancy_sha256"]


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_metrics_policy",
        "wrong_metrics_policy",
        "missing_config_policy",
        "wrong_config_policy",
        "wrong_probe",
        "wrong_episode",
        "wrong_case_id",
        "wrong_occupancy_hash",
        "wrong_nested_occupancy_hash",
        "wrong_scene_hash",
        "wrong_fixture_hash",
        "wrong_contract_hash",
        "wrong_nested_contract_hash",
        "wrong_profile_hash",
        "wrong_nested_profile_hash",
        "wrong_source_hash",
        "wrong_source_policy_hash",
        "wrong_source_runner_hash",
        "wrong_source_profile_hash",
        "missing_occupancy_path",
        "missing_source_path",
        "mismatched_fixture_path",
        "mismatched_source_path",
        "tampered_config_digest",
        "missing_config_digest",
    ),
)
def test_summary_rejects_production_shaped_provenance_mutations(
    tmp_path: Path, mutation: str
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, episode = _write_production_shaped_summary_case(
        tmp_path, case
    )
    alternate = tmp_path / "alternate.bin"
    alternate.write_bytes(b"different frozen input\n")
    alternate_hash = hashlib.sha256(alternate.read_bytes()).hexdigest()

    if mutation == "missing_metrics_policy":
        metrics.pop("policy")
    elif mutation == "wrong_metrics_policy":
        metrics["policy"] = "oracle"
    elif mutation == "wrong_probe":
        metrics["probe"] = not case.probe
    elif mutation == "wrong_episode":
        metrics["episode_id"] = "tracking_acceptance_other_case"
    elif mutation == "missing_config_policy":
        config.pop("policy")
    elif mutation == "wrong_config_policy":
        config["policy"] = {
            "name": "oracle",
            "variant": "oracle_trajectory_v1",
        }
    elif mutation == "wrong_case_id":
        config["case"] = {**config["case"], "case_id": "other_case"}
    elif mutation == "wrong_occupancy_hash":
        config["occupancy_sha256"] = alternate_hash
    elif mutation == "wrong_nested_occupancy_hash":
        config["fixture"] = {
            **config["fixture"],
            "occupancy_sha256": alternate_hash,
        }
    elif mutation == "wrong_scene_hash":
        config["fixture"] = {
            **config["fixture"],
            "scene_sha256": alternate_hash,
        }
    elif mutation == "wrong_fixture_hash":
        config["fixture_sha256"] = alternate_hash
    elif mutation == "wrong_contract_hash":
        config["contract_sha256"] = alternate_hash
    elif mutation == "wrong_nested_contract_hash":
        config["contract"] = {
            **config["contract"],
            "sha256": alternate_hash,
        }
    elif mutation == "wrong_profile_hash":
        config["drive_profile_sha256"] = alternate_hash
    elif mutation == "wrong_nested_profile_hash":
        config["drive_profile"] = {
            **config["drive_profile"],
            "sha256": alternate_hash,
        }
    elif mutation == "wrong_source_hash":
        config["source"] = {
            **config["source"],
            "geometry_sha256": alternate_hash,
        }
    elif mutation in {
        "wrong_source_policy_hash",
        "wrong_source_runner_hash",
        "wrong_source_profile_hash",
    }:
        source_key = {
            "wrong_source_policy_hash": "policy_sha256",
            "wrong_source_runner_hash": "runner_sha256",
            "wrong_source_profile_hash": "trajectory_profile_sha256",
        }[mutation]
        config["source"] = {**config["source"], source_key: alternate_hash}
    elif mutation == "missing_occupancy_path":
        config["fixture"] = {
            key: value
            for key, value in config["fixture"].items()
            if key != "occupancy_path"
        }
    elif mutation == "missing_source_path":
        config["source"] = {
            key: value
            for key, value in config["source"].items()
            if key != "runner_path"
        }
    elif mutation == "mismatched_fixture_path":
        config["fixture"] = {
            **config["fixture"],
            "scene_path": str(alternate),
        }
    elif mutation == "mismatched_source_path":
        config["source"] = {
            **config["source"],
            "runner_path": str(alternate),
        }

    (case_dir / "metrics.json").write_bytes(harness._canonical_json_bytes(metrics))
    _rewrite_summary_case_config(case_dir, config)
    if mutation == "tampered_config_digest":
        (case_dir / "config.sha256").write_text(alternate_hash + "\n", encoding="utf-8")
    elif mutation == "missing_config_digest":
        (case_dir / "config.sha256").unlink()
    with pytest.raises(harness.DiagnosticValidationError):
        harness._summary(
            tmp_path,
            (case,),
            policy_variant="oracle_trajectory_v1",
        )


def test_summary_rejects_mixed_shared_inputs_but_allows_distinct_episode_ids(
    tmp_path: Path,
):
    cases = load_tracking_cases(CONFIG_PATH)[:2]
    first_dir, first_config, first_metrics, first_episode = (
        _write_production_shaped_summary_case(tmp_path, cases[0])
    )
    second_root = tmp_path / "second"
    second_dir, second_config, second_metrics, second_episode = (
        _write_production_shaped_summary_case(second_root, cases[1])
    )
    # Move the second case into the selected run root while retaining its
    # case-specific episode identity; only shared fixture evidence differs.
    destination = tmp_path / cases[1].case_id
    second_dir.rename(destination)
    alternate = tmp_path / "alternate_scene.usda"
    alternate.write_text("#usda alternate\n", encoding="utf-8")
    alternate_hash = hashlib.sha256(alternate.read_bytes()).hexdigest()
    second_config["fixture"] = {
        **second_config["fixture"],
        "scene_path": str(alternate),
        "scene_sha256": alternate_hash,
    }
    second_config["fixture_sha256"] = alternate_hash
    second_config["episode"] = {
        **second_config["episode"],
        "scene_sha256": alternate_hash,
    }
    _rewrite_summary_case_config(destination, second_config)

    with pytest.raises(harness.DiagnosticValidationError):
        harness._summary(
            tmp_path,
            cases,
            policy_variant="oracle_trajectory_v1",
        )


def test_summary_accepts_distinct_case_episode_ids_with_shared_inputs(tmp_path: Path):
    cases = load_tracking_cases(CONFIG_PATH)[:2]
    for index, case in enumerate(cases):
        case_dir, _, _, _ = _write_production_shaped_summary_case(
            tmp_path / f"case-inputs-{index}",
            case,
        )
        case_dir.rename(tmp_path / case.case_id)

    summary_path = harness._summary(
        tmp_path,
        cases,
        policy_variant="oracle_trajectory_v1",
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert [item["episode_id"] for item in summary["cases"]] == [
        f"tracking_acceptance_{case.case_id}" for case in cases
    ]


def test_summary_records_selected_lookahead(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, _, _, _ = _write_production_shaped_summary_case(
        tmp_path,
        case,
        policy_variant="baseline",
    )

    summary_path = harness._summary(
        tmp_path,
        (case,),
        policy_variant="baseline",
        lookahead_m=1.0,
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["lookahead_m"] == pytest.approx(1.0)
    assert summary["cases"][0]["lookahead_m"] == pytest.approx(1.0)


def test_harness_cli_defaults_to_all_and_supports_explicit_case():
    parser = harness._build_parser()
    args = parser.parse_args(
        ["--config", str(CONFIG_PATH), "--run-root", "/tmp/diag", "--headless"]
    )
    assert args.all_cases is True
    assert args.lookahead_m == pytest.approx(0.5)
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
        ]
    )
    assert selected.case_id == "line_forward_0p5"
    assert selected.all_cases is False


def test_default_baseline_coordinator_does_not_index_v1_only_parameters(
    monkeypatch, tmp_path: Path
):
    """The real coordinator path must resolve only baseline parameters."""

    launched: list[list[str]] = []

    def fake_worker(command, check=False):  # type: ignore[no-untyped-def]
        launched.append(command)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(harness.subprocess, "run", fake_worker)

    result = harness.main(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            str(tmp_path / "run"),
            "--headless",
        ]
    )

    assert result == 1
    assert len(launched) == 8
    assert all("--policy-variant" in command for command in launched)
    assert all(
        command[command.index("--policy-variant") + 1] == "baseline"
        for command in launched
    )


def test_harness_cli_propagates_selected_lookahead_to_child_command():
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--lookahead-m",
            "1.0",
        ]
    )

    command = harness._child_command(selected, "line_forward_0p5")

    assert command[command.index("--lookahead-m") + 1] == "1.0"


def test_harness_cli_propagates_selected_yaw_rate_feedback_gain_to_child_command():
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--yaw-rate-feedback-gain-s",
            "0",
        ]
    )

    command = harness._child_command(selected, "line_forward_0p5")

    assert selected.yaw_rate_feedback_gain_s == pytest.approx(0.0)
    assert command[command.index("--yaw-rate-feedback-gain-s") + 1] == "0.0"


def test_harness_cli_v1_uses_fixed_defaults_and_propagates_all_parameters():
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--policy-variant",
            "oracle_trajectory_v1",
        ]
    )

    expected = {
        "lookahead_m": "1.0",
        "smoothing_window_m": "0.4",
        "curvature_preview_m": "0.8",
        "lateral_curvature_gain_1_m2": "0.2",
        "heading_curvature_gain_1_m": "0.15",
        "max_feedback_curvature_1_m": "0.2",
        "yaw_rate_feedback_gain_s": "0.15",
        "latency_preview_s": "0.0",
    }
    assert selected.policy_variant == "oracle_trajectory_v1"
    assert selected.lookahead_m == pytest.approx(1.0)
    for name, value in expected.items():
        assert float(getattr(selected, name)) == pytest.approx(float(value))

    command = harness._child_command(selected, "line_forward_0p5")
    for name, value in expected.items():
        flag = "--" + name.replace("_", "-")
        assert command[command.index(flag) + 1] == value
    assert command[command.index("--policy-variant") + 1] == "oracle_trajectory_v1"


def test_harness_cli_trajectory_v2_propagates_exact_constructor_parameters_to_child_command():
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--policy-variant",
            "oracle_trajectory_v2",
            "--auxiliary-latency-s",
            "1.25",
            "--maximum-delay-distance-m",
            "0.28",
            "--profile-window-m",
            "0.24",
            "--profile-blend-start-1-m",
            "0.04",
            "--profile-blend-full-1-m",
            "0.18",
            "--feedback-lookahead-m",
            "1.10",
            "--maximum-feedback-curvature-1-m",
            "0.17",
            "--supported-main-command-rad-s",
            "0.50",
            "--episode-budget-s",
            "35.0",
        ]
    )

    expected = {
        "auxiliary_latency_s": "1.25",
        "maximum_delay_distance_m": "0.28",
        "profile_window_m": "0.24",
        "profile_blend_start_1_m": "0.04",
        "profile_blend_full_1_m": "0.18",
        "feedback_lookahead_m": "1.1",
        "maximum_feedback_curvature_1_m": "0.17",
        "supported_main_command_rad_s": "0.5",
        "episode_budget_s": "35.0",
    }
    assert selected.policy_variant == "oracle_trajectory_v2"
    command = harness._child_command(selected, "line_forward_0p5")
    for name, value in expected.items():
        flag = "--" + name.replace("_", "-")
        assert command.count(flag) == 1
        assert command[command.index(flag) + 1] == value


def test_v2_terminal_brake_buffer_uses_raw_default_and_explicit_candidate_transport(
    tmp_path: Path,
):
    raw_config = tmp_path / "tracking_acceptance.json"
    raw_config.write_bytes(CONFIG_PATH.read_bytes())
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(raw_config),
            "--run-root",
            str(tmp_path / "run"),
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--policy-variant",
            "oracle_trajectory_v2",
            "--terminal-brake-buffer-m",
            "0.30",
        ]
    )

    assert selected.terminal_brake_buffer_m == pytest.approx(0.30)
    command = harness._child_command(selected, "line_forward_0p5")
    flag = "--terminal-brake-buffer-m"
    assert command.count(flag) == 1
    assert command[command.index(flag) + 1] == "0.3"

    child_args = parser.parse_args(command[2:])
    explicit = harness._v2_explicit_parameters_from_namespace(child_args)
    assert explicit["terminal_brake_buffer_m"] == pytest.approx(0.30)
    loaded = harness.load_harness_config(
        child_args.config,
        policy_variant="oracle_trajectory_v2",
        **explicit,
    )
    assert loaded["terminal_brake_buffer_m"] == pytest.approx(0.30)
    provenance = loaded["trajectory_policy_v2_provenance"]
    assert provenance["raw_policy"]["parameters"]["terminal_brake_buffer_m"] == pytest.approx(
        0.10
    )
    assert provenance["override_markers"]["terminal_brake_buffer_m"] is True
    assert provenance["override_values"]["terminal_brake_buffer_m"] == pytest.approx(
        0.30
    )


@pytest.mark.parametrize(
    ("override_args", "expected_names"),
    (
        ([], set()),
        (["--auxiliary-latency-s", "1.25"], {"auxiliary_latency_s"}),
    ),
)
def test_v2_child_command_only_carries_original_cli_overrides_and_artifact_markers(
    tmp_path: Path,
    override_args: list[str],
    expected_names: set[str],
):
    """Omitted child values come from raw config and parser presence is authoritative."""

    case = load_tracking_cases(CONFIG_PATH)[0]
    _, _, _, episode = _write_production_shaped_summary_case(
        tmp_path,
        case,
        policy_variant="oracle_trajectory_v2",
    )
    raw_config = tmp_path / "tracking_acceptance.json"
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(raw_config),
            "--run-root",
            str(tmp_path / "run"),
            "--headless",
            "--case-id",
            case.case_id,
            "--policy-variant",
            "oracle_trajectory_v2",
            *override_args,
        ]
    )
    command = harness._child_command(selected, case.case_id)
    assert "--_v2-override-markers" not in command
    for name in harness.TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
        flag = "--" + name.replace("_", "-")
        assert command.count(flag) == int(name in expected_names)

    child_args = parser.parse_args(command[2:])
    explicit_values = {
        name: getattr(child_args, name)
        if bool(getattr(child_args, f"_{name}_provided", False))
        else None
        for name in harness.TRAJECTORY_POLICY_V2_PARAMETER_NAMES
    }
    loaded = harness.load_harness_config(
        child_args.config,
        scene_root=tmp_path,
        contract_path=tmp_path / "data_contract.yaml",
        occupancy_path=tmp_path / "tracking_open_plane_free.npz",
        fixture_path=tmp_path / "tracking_open_plane.usda",
        policy_variant="oracle_trajectory_v2",
        **explicit_values,
    )
    provenance = loaded["trajectory_policy_v2_provenance"]
    assert {
        name
        for name, marked in provenance["override_markers"].items()
        if marked
    } == expected_names
    artifact_config = harness._case_config_payload(
        case,
        build_tracking_reference(case),
        episode,
        loaded,
        fixture_path=tmp_path / "tracking_open_plane.usda",
        occupancy_path=tmp_path / "tracking_open_plane_free.npz",
        contract_path=tmp_path / "data_contract.yaml",
        profile_path=tmp_path / "drive_dynamics_v1.json",
    )
    assert {
        name
        for name, marked in artifact_config["tracking_config"]["override_markers"].items()
        if marked
    } == expected_names


def test_v2_parser_rejects_removed_hidden_override_marker_transport():
    parser = harness._build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--config",
                str(CONFIG_PATH),
                "--run-root",
                "/tmp/diag",
                "--headless",
                "--policy-variant",
                "oracle_trajectory_v2",
                "--_v2-override-markers",
                "auxiliary_latency_s",
            ]
        )


def test_v2_worker_omitted_values_reload_raw_config_without_parent_masking(
    tmp_path: Path,
):
    raw_config = tmp_path / "tracking_acceptance.json"
    raw_config.write_bytes(CONFIG_PATH.read_bytes())
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(raw_config),
            "--run-root",
            str(tmp_path / "run"),
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--policy-variant",
            "oracle_trajectory_v2",
        ]
    )
    command = harness._child_command(selected, "line_forward_0p5")
    assert all(
        "--" + name.replace("_", "-") not in command
        for name in harness.TRAJECTORY_POLICY_V2_PARAMETER_NAMES
    )

    payload = json.loads(raw_config.read_text(encoding="utf-8"))
    payload["trajectory_policy_v2"]["parameters"]["auxiliary_latency_s"] = 1.75
    raw_config.write_bytes(harness._canonical_json_bytes(payload))

    child_args = parser.parse_args(command[2:])
    child_parameters = harness._policy_parameters_from_namespace(child_args)
    assert child_parameters["auxiliary_latency_s"] == pytest.approx(1.75)


def test_harness_trajectory_v2_policy_builder_passes_exact_constructor_parameters(
    monkeypatch, tmp_path: Path
):
    captured = {}
    drive_model = object()

    class FakePolicy:
        def __init__(self, protocol, **kwargs):
            captured["protocol"] = protocol
            captured.update(kwargs)

    monkeypatch.setattr(
        harness, "DelayAwareTrajectoryOraclePolicy", FakePolicy, raising=False
    )
    monkeypatch.setattr(harness.DriveModel, "from_json", lambda path: drive_model)
    protocol = object()
    parameters = {
        "auxiliary_latency_s": 1.25,
        "maximum_delay_distance_m": 0.28,
        "profile_window_m": 0.24,
        "profile_blend_start_1_m": 0.04,
        "profile_blend_full_1_m": 0.18,
        "feedback_lookahead_m": 1.10,
        "maximum_feedback_curvature_1_m": 0.17,
        "supported_main_command_rad_s": 0.50,
        "episode_budget_s": 35.0,
        "terminal_brake_buffer_m": 0.30,
    }

    policy = harness._build_policy(
        protocol,
        tmp_path / "drive.json",
        policy_variant="oracle_trajectory_v2",
        **parameters,
    )

    assert isinstance(policy, FakePolicy)
    assert captured["protocol"] is protocol
    for name, value in parameters.items():
        assert captured[name] == pytest.approx(value)
    assert captured["drive_model"] is drive_model


def test_run_tracking_case_binds_custom_v2_budget_to_policy_and_runner(
    monkeypatch, tmp_path: Path
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    captured: dict[str, object] = {}

    monkeypatch.setattr(harness, "validate_diagnostic_fixture", lambda *args: {})
    monkeypatch.setattr(
        harness,
        "build_tracking_episode",
        lambda *args, **kwargs: object(),
    )
    monkeypatch.setattr(
        harness,
        "load_action_protocol",
        lambda path: SimpleNamespace(protocol_id="ball_action_v2_aux30"),
    )

    class FakeRuntime:
        def __init__(self, config):
            self.config = config
            self.occupancy = SimpleNamespace()

        def set_reference_visualization_path(self, path):
            captured["visualization_path"] = path

        def close(self, return_code=0):
            captured["return_code"] = return_code

    monkeypatch.setattr(harness, "IsaacBenchmarkRuntime", FakeRuntime)

    def fake_build_policy(protocol, profile_path, **kwargs):
        captured["policy_parameters"] = dict(kwargs)
        return object()

    monkeypatch.setattr(harness, "_build_policy", fake_build_policy)

    def fake_run_episode(runtime, policy, episode, *, action_protocol, config, **kwargs):
        captured["runner_config"] = config
        return SimpleNamespace(
            positions_world_xy=np.zeros((1, 2), dtype=np.float64),
            action_blocks=0,
        )

    monkeypatch.setattr(harness, "run_episode", fake_run_episode)

    def fake_write_tracking_artifacts(output_root, **kwargs):
        captured["artifact_harness_config"] = dict(kwargs["harness_config"])
        destination = Path(output_root) / case.case_id
        destination.mkdir(parents=True)
        (destination / "metrics.json").write_text(
            json.dumps({"episode_budget_s": 2.0}), encoding="utf-8"
        )
        return destination

    monkeypatch.setattr(harness, "write_tracking_artifacts", fake_write_tracking_artifacts)

    harness_config = {
        "policy_variant": "oracle_trajectory_v2",
        "harness": {
            "config_path": str(CONFIG_PATH),
            "scene_root": str(tmp_path),
            "fixture_path": str(tmp_path / "fixture.usda"),
            "occupancy_path": str(tmp_path / "occupancy.npz"),
            "contract_path": str(tmp_path / "contract.yaml"),
            "drive_profile_path": str(tmp_path / "profile.json"),
        },
    }
    harness.run_tracking_case(
        case,
        harness_config=harness_config,
        output_root=tmp_path / "run",
        gui=False,
        policy_variant="oracle_trajectory_v2",
        episode_budget_s=2.0,
    )

    assert captured["policy_parameters"]["episode_budget_s"] == pytest.approx(2.0)
    assert captured["runner_config"].episode_budget_s == pytest.approx(2.0)
    assert captured["runner_config"].diagnostic_settling_budget_s == pytest.approx(2.0)
    assert captured["artifact_harness_config"]["episode_budget_s"] == pytest.approx(2.0)
    assert captured["artifact_harness_config"]["settling_budget_s"] == pytest.approx(2.0)


def test_loader_exposes_v2_policy_parameters_as_nested_config():
    loaded = harness.load_harness_config(
        CONFIG_PATH,
        policy_variant="oracle_trajectory_v2",
    )

    assert loaded["trajectory_policy"] == {
        "name": "oracle_trajectory_v2",
        "parameters": dict(harness.TRAJECTORY_POLICY_V2_DEFAULTS),
    }


def test_v2_config_is_authority_unless_cli_parameter_is_explicit(tmp_path: Path):
    payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    payload["trajectory_policy_v2"]["parameters"]["auxiliary_latency_s"] = 1.25
    mutated_config = tmp_path / "tracking_acceptance.json"
    mutated_config.write_text(json.dumps(payload), encoding="utf-8")

    loaded = harness.load_harness_config(
        mutated_config,
        policy_variant="oracle_trajectory_v2",
    )
    assert loaded["auxiliary_latency_s"] == pytest.approx(1.25)
    assert loaded["terminal_brake_buffer_m"] == pytest.approx(0.10)

    overridden = harness.load_harness_config(
        mutated_config,
        policy_variant="oracle_trajectory_v2",
        auxiliary_latency_s=1.10,
    )
    assert overridden["auxiliary_latency_s"] == pytest.approx(1.10)

    candidate = harness.load_harness_config(
        mutated_config,
        policy_variant="oracle_trajectory_v2",
        terminal_brake_buffer_m=0.30,
    )
    assert candidate["terminal_brake_buffer_m"] == pytest.approx(0.30)
    provenance = candidate["trajectory_policy_v2_provenance"]
    assert provenance["raw_policy"]["parameters"]["terminal_brake_buffer_m"] == pytest.approx(
        0.10
    )
    assert provenance["override_markers"]["terminal_brake_buffer_m"] is True
    assert provenance["override_values"]["terminal_brake_buffer_m"] == pytest.approx(
        0.30
    )


def test_baseline_and_v1_do_not_read_v2_terminal_brake_buffer(monkeypatch):
    def fail_if_v2_authority_is_read(payload):
        raise AssertionError("baseline/V1 must not read V2 policy parameters")

    monkeypatch.setattr(harness, "_v2_parameters_from_payload", fail_if_v2_authority_is_read)
    for variant in (harness.DEFAULT_POLICY_VARIANT, harness.TRAJECTORY_POLICY_VARIANT):
        loaded = harness.load_harness_config(CONFIG_PATH, policy_variant=variant)
        assert "terminal_brake_buffer_m" not in loaded


def test_v2_case_records_and_replays_explicit_override_provenance(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    override_value = 1.10
    config["auxiliary_latency_s"] = override_value
    config["trajectory_policy"]["parameters"]["auxiliary_latency_s"] = override_value
    config["tracking_config"]["override_markers"]["auxiliary_latency_s"] = True
    config["tracking_config"]["override_values"]["auxiliary_latency_s"] = override_value
    metrics["auxiliary_latency_s"] = override_value
    metrics["trajectory_policy"]["parameters"]["auxiliary_latency_s"] = override_value
    _rewrite_summary_case_evidence(case_dir, config, metrics)

    summary_path = harness._summary(
        tmp_path,
        (case,),
        policy_variant="oracle_trajectory_v2",
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["tracking_config"]["override_markers"]["auxiliary_latency_s"] is True
    assert summary["tracking_config"]["override_values"]["auxiliary_latency_s"] == pytest.approx(
        override_value
    )


def test_trajectory_v2_artifact_writer_rejects_missing_policy_diagnostics(tmp_path: Path):
    reference, result, episode, runtime, config = _artifact_case(tmp_path)
    result.policy_name = "oracle_trajectory_v2"
    config.update(
        {
            "policy_variant": "oracle_trajectory_v2",
            **{
                name: value
                for name, value in {
                    "auxiliary_latency_s": 1.0,
                    "maximum_delay_distance_m": 0.30,
                    "profile_window_m": 0.30,
                    "profile_blend_start_1_m": 0.05,
                    "profile_blend_full_1_m": 0.20,
                    "feedback_lookahead_m": 1.0,
                    "maximum_feedback_curvature_1_m": 0.20,
                    "supported_main_command_rad_s": 0.50,
                    "episode_budget_s": 35.0,
                }.items()
            },
        }
    )

    with pytest.raises(harness.DiagnosticValidationError, match="diagnostic"):
        harness.write_tracking_artifacts(
            tmp_path / "run",
            case=reference.case,
            reference=reference,
            episode=episode,
            result=result,
            runtime=runtime,
            harness_config=config,
        )


def test_trajectory_v2_case_artifacts_record_policy_parameters_and_provenance(tmp_path: Path):
    reference, result, episode, runtime, config = _artifact_case(tmp_path)
    result.policy_name = "oracle_trajectory_v2"
    parameters = {
        "auxiliary_latency_s": 1.0,
        "maximum_delay_distance_m": 0.30,
        "profile_window_m": 0.30,
        "profile_blend_start_1_m": 0.05,
        "profile_blend_full_1_m": 0.20,
        "feedback_lookahead_m": 1.0,
        "maximum_feedback_curvature_1_m": 0.20,
        "supported_main_command_rad_s": 0.50,
        "episode_budget_s": 35.0,
    }
    for name in harness.TRAJECTORY_DIAGNOSTIC_NAMES:
        setattr(result, name, np.linspace(0.01, 0.06, 6, dtype=np.float64))
    for name in V2_ACTION_TRACE_NAMES:
        if name == "budget_infeasible":
            setattr(result, name, np.zeros(6, dtype=bool))
        else:
            setattr(result, name, np.linspace(0.01, 0.06, 6, dtype=np.float64))
    for name in V2_TERMINAL_TRACE_NAMES:
        setattr(result, name, np.full(6, -1.0, dtype=np.float64))
    result.elapsed_s = 0.5
    result.diagnostic_terminal_brake_start_time_s = None
    result.diagnostic_goal_entry_time_s = None
    result.diagnostic_stop_below_speed_time_s = None
    result.diagnostic_hold_complete_time_s = None
    config["harness"]["config_path"] = str(CONFIG_PATH)
    config.update({"policy_variant": "oracle_trajectory_v2", **parameters})

    output = harness.write_tracking_artifacts(
        tmp_path / "run",
        case=reference.case,
        reference=reference,
        episode=episode,
        result=result,
        runtime=runtime,
        harness_config=config,
    )

    config_payload = json.loads((output / "config.json").read_text(encoding="utf-8"))
    metrics_payload = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    assert config_payload["policy_variant"] == "oracle_trajectory_v2"
    assert metrics_payload["policy_variant"] == "oracle_trajectory_v2"
    for name, value in parameters.items():
        assert config_payload[name] == pytest.approx(value)
        assert metrics_payload[name] == pytest.approx(value)
    for name in (*harness.TRAJECTORY_DIAGNOSTIC_NAMES, *V2_ACTION_TRACE_NAMES, *V2_TERMINAL_TRACE_NAMES):
        assert name in metrics_payload["trajectory_diagnostics"]
        assert len(metrics_payload["trajectory_diagnostics"][name]) == result.action_blocks
    assert metrics_payload["trajectory_diagnostics"]["budget_infeasible"] == [False] * 6
    assert metrics_payload["tracking_config_sha256"] == hashlib.sha256(
        CONFIG_PATH.read_bytes()
    ).hexdigest()
    assert config_payload["source"]["policy_sha256"] == harness._sha256(
        Path(__file__).resolve().parents[1] / "benchmark" / "policies.py"
    )
    assert metrics_payload["config_sha256"] == hashlib.sha256(
        (output / "config.json").read_bytes()
    ).hexdigest()


def test_trajectory_v2_summary_rejects_missing_diagnostic_trace(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    metrics.pop("trajectory_diagnostics", None)
    (case_dir / "metrics.json").write_bytes(harness._canonical_json_bytes(metrics))
    with pytest.raises(harness.DiagnosticValidationError, match="diagnostic"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_artifact_writer_rejects_mismatched_action_trace_length(
    tmp_path: Path,
):
    reference, result, episode, runtime, config = _artifact_case(tmp_path)
    result.policy_name = "oracle_trajectory_v2"
    config["harness"]["config_path"] = str(CONFIG_PATH)
    config.update(
        {
            "policy_variant": "oracle_trajectory_v2",
            **dict(harness.TRAJECTORY_POLICY_V2_DEFAULTS),
        }
    )
    for name in harness.TRAJECTORY_DIAGNOSTIC_NAMES:
        setattr(result, name, np.zeros(result.action_blocks, dtype=np.float64))
    for name in V2_ACTION_TRACE_NAMES:
        if name == "budget_infeasible":
            setattr(result, name, np.zeros(result.action_blocks, dtype=bool))
        else:
            setattr(result, name, np.zeros(result.action_blocks, dtype=np.float64))
    for name in V2_TERMINAL_TRACE_NAMES:
        setattr(result, name, np.full(result.action_blocks, -1.0, dtype=np.float64))
    result.projection_s_m = np.zeros(result.action_blocks - 1, dtype=np.float64)

    with pytest.raises(harness.DiagnosticValidationError, match="projection_s_m"):
        harness.write_tracking_artifacts(
            tmp_path / "run",
            case=reference.case,
            reference=reference,
            episode=episode,
            result=result,
            runtime=runtime,
            harness_config=config,
        )


def test_trajectory_v2_summary_rejects_mismatched_diagnostic_trace_length(
    tmp_path: Path,
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, _, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    metrics["action_blocks"] = 2
    (case_dir / "metrics.json").write_bytes(harness._canonical_json_bytes(metrics))

    with pytest.raises(harness.DiagnosticValidationError, match="action_blocks"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_summary_records_parameters_and_bound_provenance(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, _, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )

    summary_path = harness._summary(
        tmp_path,
        (case,),
        policy_variant="oracle_trajectory_v2",
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    expected = harness.TRAJECTORY_POLICY_V2_DEFAULTS
    assert summary["policy_variant"] == "oracle_trajectory_v2"
    for name, value in expected.items():
        assert summary[name] == pytest.approx(value)
    assert summary["trajectory_policy"]["parameters"] == pytest.approx(expected)
    provenance = summary["cases"][0]["artifact_provenance"]
    assert provenance["config_sha256"] == hashlib.sha256(
        (case_dir / "config.json").read_bytes()
    ).hexdigest()
    assert provenance["metrics_sha256"] == hashlib.sha256(
        (case_dir / "metrics.json").read_bytes()
    ).hexdigest()
    for name in ("fixture_sha256", "occupancy_sha256", "contract_sha256", "drive_profile_sha256"):
        assert provenance[name] == config[name]


def test_trajectory_v2_summary_rejects_mutated_raw_tracking_config(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, _, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    raw_config_path = Path(config["tracking_config"]["path"])
    raw_config_path.write_bytes(raw_config_path.read_bytes() + b"\n")

    with pytest.raises(harness.DiagnosticValidationError, match="tracking_config"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_summary_rejects_changed_config_path_with_self_consistent_hash(
    tmp_path: Path,
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    original_path = Path(config["tracking_config"]["path"])
    alternate_path = tmp_path / "alternate-tracking-acceptance.json"
    alternate_path.write_bytes(original_path.read_bytes())
    alternate_hash = hashlib.sha256(alternate_path.read_bytes()).hexdigest()
    config["tracking_config"]["path"] = str(alternate_path)
    config["tracking_config"]["sha256"] = alternate_hash
    config["tracking_config_sha256"] = alternate_hash
    _rewrite_summary_case_evidence(case_dir, config, metrics)

    with pytest.raises(harness.DiagnosticValidationError, match="config_path"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_summary_rejects_tampered_preserved_raw_policy_block(
    tmp_path: Path,
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    tracking_config = config["tracking_config"]
    raw_policy = tracking_config.setdefault(
        "raw_policy",
        {
            "name": harness.TRAJECTORY_POLICY_V2_VARIANT,
            "parameters": dict(harness.TRAJECTORY_POLICY_V2_DEFAULTS),
        },
    )
    raw_policy["parameters"]["auxiliary_latency_s"] = 1.25
    _rewrite_summary_case_evidence(case_dir, config, metrics)

    with pytest.raises(harness.DiagnosticValidationError, match="raw"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_summary_rejects_self_consistent_effective_parameter_tamper(
    tmp_path: Path,
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    tampered = 1.25
    config["auxiliary_latency_s"] = tampered
    config["trajectory_policy"]["parameters"]["auxiliary_latency_s"] = tampered
    metrics["auxiliary_latency_s"] = tampered
    metrics["trajectory_policy"]["parameters"]["auxiliary_latency_s"] = tampered
    _rewrite_summary_case_evidence(case_dir, config, metrics)

    with pytest.raises(harness.DiagnosticValidationError, match="raw|authority|parameter"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_trajectory_v2_summary_rejects_tampered_override_marker(
    tmp_path: Path,
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config, metrics, _ = _write_production_shaped_summary_case(
        tmp_path, case, policy_variant="oracle_trajectory_v2"
    )
    tracking_config = config["tracking_config"]
    markers = tracking_config.setdefault(
        "override_markers",
        {name: False for name in harness.TRAJECTORY_POLICY_V2_PARAMETER_NAMES},
    )
    values = tracking_config.setdefault("override_values", {})
    markers["auxiliary_latency_s"] = True
    values["auxiliary_latency_s"] = 1.10
    _rewrite_summary_case_evidence(case_dir, config, metrics)

    with pytest.raises(harness.DiagnosticValidationError, match="override|authority|parameter"):
        harness._summary(tmp_path, (case,), policy_variant="oracle_trajectory_v2")


def test_harness_child_command_propagates_custom_v1_values_once():
    parser = harness._build_parser()
    selected = parser.parse_args(
        [
            "--config",
            str(CONFIG_PATH),
            "--run-root",
            "/tmp/diag",
            "--headless",
            "--case-id",
            "line_forward_0p5",
            "--policy-variant",
            "oracle_trajectory_v1",
            "--lookahead-m",
            "1.25",
            "--smoothing-window-m",
            "0.35",
            "--curvature-preview-m",
            "0.70",
            "--lateral-curvature-gain-1-m2",
            "0.30",
            "--heading-curvature-gain-1-m",
            "0.12",
            "--max-feedback-curvature-1-m",
            "0.18",
            "--yaw-rate-feedback-gain-s",
            "0.05",
            "--latency-preview-s",
            "0.0",
        ]
    )

    command = harness._child_command(selected, "line_forward_0p5")
    expected = {
        "lookahead_m": "1.25",
        "smoothing_window_m": "0.35",
        "curvature_preview_m": "0.7",
        "lateral_curvature_gain_1_m2": "0.3",
        "heading_curvature_gain_1_m": "0.12",
        "max_feedback_curvature_1_m": "0.18",
        "yaw_rate_feedback_gain_s": "0.05",
        "latency_preview_s": "0.0",
    }
    for name, value in expected.items():
        flag = "--" + name.replace("_", "-")
        assert command.count(flag) == 1
        assert command[command.index(flag) + 1] == value


@pytest.mark.parametrize(
    "flag,value",
    (
        ("--lookahead-m", "0"),
        ("--smoothing-window-m", "-0.1"),
        ("--curvature-preview-m", "nan"),
        ("--lateral-curvature-gain-1-m2", "inf"),
        ("--heading-curvature-gain-1-m", "-1"),
        ("--max-feedback-curvature-1-m", "nan"),
        ("--yaw-rate-feedback-gain-s", "-1"),
        ("--latency-preview-s", "0.001"),
    ),
)
def test_harness_cli_v1_rejects_invalid_controller_parameters(flag: str, value: str):
    parser = harness._build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--config",
                str(CONFIG_PATH),
                "--run-root",
                "/tmp/diag",
                "--headless",
                "--policy-variant",
                "oracle_trajectory_v1",
                flag,
                value,
            ]
        )


def test_selected_lookahead_reaches_oracle_policy_constructor(monkeypatch, tmp_path: Path):
    captured = {}
    drive_model = object()

    class FakePolicy:
        def __init__(
            self, protocol, *, lookahead_m, yaw_rate_feedback_gain_s, drive_model
        ):
            captured.update(
                protocol=protocol,
                lookahead_m=lookahead_m,
                yaw_rate_feedback_gain_s=yaw_rate_feedback_gain_s,
                drive_model=drive_model,
            )

    monkeypatch.setattr(harness, "OraclePathPolicy", FakePolicy)
    monkeypatch.setattr(harness.DriveModel, "from_json", lambda path: drive_model)
    protocol = object()

    harness._build_oracle_policy(
        protocol,
        tmp_path / "drive.json",
        lookahead_m=1.0,
        yaw_rate_feedback_gain_s=0.0,
    )

    assert captured["protocol"] is protocol
    assert captured["lookahead_m"] == pytest.approx(1.0)
    assert captured["yaw_rate_feedback_gain_s"] == pytest.approx(0.0)
    assert captured["drive_model"] is drive_model


def test_v1_policy_builder_propagates_each_constructor_parameter(monkeypatch, tmp_path: Path):
    captured = {}
    drive_model = object()

    class FakePolicy:
        def __init__(self, protocol, **kwargs):
            captured["protocol"] = protocol
            captured.update(kwargs)

    monkeypatch.setattr(harness, "TrajectoryOraclePolicy", FakePolicy, raising=False)
    monkeypatch.setattr(harness.DriveModel, "from_json", lambda path: drive_model)
    protocol = object()
    parameters = {
        "lookahead_m": 1.0,
        "smoothing_window_m": 0.40,
        "curvature_preview_m": 0.80,
        "lateral_curvature_gain_1_m2": 0.20,
        "heading_curvature_gain_1_m": 0.15,
        "max_feedback_curvature_1_m": 0.20,
        "yaw_rate_feedback_gain_s": 0.15,
        "latency_preview_s": 0.0,
    }

    policy = harness._build_policy(
        protocol,
        tmp_path / "drive.json",
        policy_variant="oracle_trajectory_v1",
        **parameters,
    )

    assert isinstance(policy, FakePolicy)
    assert captured["protocol"] is protocol
    for name, value in parameters.items():
        assert captured[name] == pytest.approx(value)
    assert policy.drive_model is drive_model


def test_harness_config_records_yaw_rate_feedback_gain_and_hashes_it(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    reference = build_tracking_reference(case)
    result = _fake_result(reference)
    episode = _fake_episode(reference)
    fixture = tmp_path / "tracking_open_plane.usda"
    occupancy = tmp_path / "tracking_open_plane_free.npz"
    contract = tmp_path / "data_contract.yaml"
    fixture.write_text("#usda 1.0\n", encoding="utf-8")
    np.savez(
        occupancy,
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    contract.write_text("contract", encoding="utf-8")
    (tmp_path / "drive_dynamics_v1.json").write_text("{}", encoding="utf-8")
    runtime = _fake_runtime(tmp_path)
    config = {
        "schema_version": "tracking_acceptance_v1",
        "case_id": case.case_id,
        "lookahead_m": 1.0,
        "yaw_rate_feedback_gain_s": 0.0,
        "harness": {
            "scene_root": str(tmp_path),
            "fixture_path": str(fixture),
            "occupancy_path": str(occupancy),
            "contract_path": str(contract),
            "drive_profile_path": str(tmp_path / "drive_dynamics_v1.json"),
        },
    }

    output = harness.write_tracking_artifacts(
        tmp_path / "run",
        case=case,
        reference=reference,
        episode=episode,
        result=result,
        runtime=runtime,
        harness_config=config,
    )

    assert harness.verify_config_hash(output)
    config_payload = json.loads((output / "config.json").read_text(encoding="utf-8"))
    assert config_payload["yaw_rate_feedback_gain_s"] == pytest.approx(0.0)


def test_v1_case_config_and_metrics_record_all_controller_parameters(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    reference = build_tracking_reference(case)
    result = _fake_result(reference)
    episode = _fake_episode(reference)
    fixture = tmp_path / "tracking_open_plane.usda"
    occupancy = tmp_path / "tracking_open_plane_free.npz"
    contract = tmp_path / "data_contract.yaml"
    profile = tmp_path / "drive_dynamics_v1.json"
    fixture.write_text("#usda 1.0\n", encoding="utf-8")
    contract.write_text("contract", encoding="utf-8")
    profile.write_text("{}", encoding="utf-8")
    np.savez(
        occupancy,
        grid=np.zeros((80, 80), dtype=np.uint8),
        global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    runtime = _fake_runtime(tmp_path)
    runtime.config.drive_profile_path = profile
    trajectory_diagnostics = {
        name: np.linspace(0.01, 0.01 * 6, 6, dtype=np.float64)
        for name in harness.TRAJECTORY_DIAGNOSTIC_NAMES
    }
    for name, values in trajectory_diagnostics.items():
        setattr(result, name, values)
    result.policy_name = "oracle_trajectory_v1"
    config = {
        "schema_version": "tracking_acceptance_v1",
        "policy_variant": "oracle_trajectory_v1",
        "lookahead_m": 1.0,
        "smoothing_window_m": 0.40,
        "curvature_preview_m": 0.80,
        "lateral_curvature_gain_1_m2": 0.20,
        "heading_curvature_gain_1_m": 0.15,
        "max_feedback_curvature_1_m": 0.20,
        "yaw_rate_feedback_gain_s": 0.15,
        "latency_preview_s": 0.0,
        "harness": {
            "scene_root": str(tmp_path),
            "fixture_path": str(fixture),
            "occupancy_path": str(occupancy),
            "contract_path": str(contract),
            "drive_profile_path": str(profile),
        },
    }

    output = harness.write_tracking_artifacts(
        tmp_path / "run",
        case=case,
        reference=reference,
        episode=episode,
        result=result,
        runtime=runtime,
        harness_config=config,
    )

    expected = {
        "lookahead_m": 1.0,
        "smoothing_window_m": 0.40,
        "curvature_preview_m": 0.80,
        "lateral_curvature_gain_1_m2": 0.20,
        "heading_curvature_gain_1_m": 0.15,
        "max_feedback_curvature_1_m": 0.20,
        "yaw_rate_feedback_gain_s": 0.15,
        "latency_preview_s": 0.0,
    }
    config_payload = json.loads((output / "config.json").read_text(encoding="utf-8"))
    assert config_payload["policy_variant"] == "oracle_trajectory_v1"
    for name, value in expected.items():
        assert config_payload[name] == pytest.approx(value)
    metrics_payload = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
    for name, values in trajectory_diagnostics.items():
        assert name in metrics_payload
        np.testing.assert_allclose(
            metrics_payload["trajectory_diagnostics"][name], values
        )
    with np.load(output / "states.npz", allow_pickle=False) as states:
        for name, values in trajectory_diagnostics.items():
            artifact_name = (
                "policy_heading_error_rad"
                if name == "heading_error_rad"
                else name
            )
            assert states[artifact_name].shape == (result.actions.shape[0],)
            assert np.all(np.isfinite(states[artifact_name]))
            np.testing.assert_allclose(states[artifact_name], values)
    config_sha256 = hashlib.sha256((output / "config.json").read_bytes()).hexdigest()
    assert harness.verify_config_hash(output)
    assert metrics_payload["config_sha256"] == config_sha256

    result.policy_name = "oracle"
    with pytest.raises(harness.DiagnosticValidationError, match="policy identity"):
        harness.write_tracking_artifacts(
            tmp_path / "mismatched-policy-run",
            case=case,
            reference=reference,
            episode=episode,
            result=result,
            runtime=runtime,
            harness_config=config,
        )


@pytest.mark.parametrize(
    "invalid_value",
    (None, np.asarray([1.0 + 2.0j] * 6, dtype=np.complex128)),
    ids=("none", "complex_array"),
)
def test_artifact_conversion_rejects_present_invalid_v1_diagnostics(invalid_value):
    result = SimpleNamespace(reference_curvature_1_m=invalid_value)

    with pytest.raises(
        harness.DiagnosticValidationError,
        match="reference_curvature_1_m",
    ):
        harness._action_aligned_policy_diagnostic(
            result,
            "reference_curvature_1_m",
            6,
        )


def test_v1_summary_defaults_record_all_controller_parameters(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir, config_payload, _, _ = _write_production_shaped_summary_case(
        tmp_path,
        case,
    )
    config_bytes = (case_dir / "config.json").read_bytes()
    config_digest = hashlib.sha256(config_bytes).hexdigest()

    summary_path = harness._summary(
        tmp_path,
        (case,),
        policy_variant="oracle_trajectory_v1",
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["policy_variant"] == "oracle_trajectory_v1"
    for name, value in harness.TRAJECTORY_POLICY_DEFAULTS.items():
        assert summary[name] == pytest.approx(value)
    assert summary["trajectory_policy"]["parameters"] == pytest.approx(
        harness.TRAJECTORY_POLICY_DEFAULTS
    )
    provenance = summary["cases"][0]["artifact_provenance"]
    assert provenance["metrics_sha256"] == hashlib.sha256(
        (case_dir / "metrics.json").read_bytes()
    ).hexdigest()
    assert provenance["config_sha256"] == hashlib.sha256(config_bytes).hexdigest()
    assert provenance["config_digest_sha256"] == config_digest
    assert provenance["fixture_sha256"] == config_payload["fixture_sha256"]
    assert provenance["occupancy_sha256"] == config_payload["occupancy_sha256"]
    assert provenance["contract_sha256"] == config_payload["contract_sha256"]
    assert provenance["drive_profile_sha256"] == config_payload["drive_profile_sha256"]
    assert summary["artifact_hashes"]["fixture_sha256"][case.case_id] == config_payload["fixture_sha256"]
    assert summary["artifact_hashes"]["occupancy_sha256"][case.case_id] == config_payload["occupancy_sha256"]
    assert summary["artifact_hashes"]["contract_sha256"][case.case_id] == config_payload["contract_sha256"]
    assert summary["artifact_hashes"]["drive_profile_sha256"][case.case_id] == config_payload["drive_profile_sha256"]
    source_hashes = summary["source_hashes"]
    assert source_hashes["policy_sha256"] == harness._sha256(
        Path(__file__).resolve().parents[1] / "benchmark" / "policies.py"
    )
    assert source_hashes["runner_sha256"] == harness._sha256(
        Path(__file__).resolve().parents[1] / "benchmark" / "runner.py"
    )
    assert source_hashes["trajectory_profile_sha256"] == harness._sha256(
        Path(__file__).resolve().parents[1]
        / "benchmark"
        / "trajectory_tracking.py"
    )
    assert source_hashes["harness_sha256"] == harness._sha256(harness._HERE)
    assert source_hashes["geometry_sha256"] == harness._sha256(
        Path(__file__).resolve().parents[1]
        / "benchmark"
        / "tracking_acceptance.py"
    )


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_config",
        "missing_digest",
        "tampered_digest",
        "case_id",
        "policy_variant",
        "lookahead_m",
        "fixture_hash",
        "contract_hash",
        "drive_profile_hash",
        "harness_hash",
        "geometry_hash",
    ),
)
def test_summary_rejects_incomplete_or_mismatched_case_evidence(
    tmp_path: Path, mutation: str
):
    case = load_tracking_cases(CONFIG_PATH)[0]
    case_dir = tmp_path / case.case_id
    case_dir.mkdir()
    parameters = dict(harness.TRAJECTORY_POLICY_DEFAULTS)
    (case_dir / "metrics.json").write_text(
        json.dumps(
            {
                "case_id": case.case_id,
                "probe": False,
                "acceptance": {},
                "policy_variant": "oracle_trajectory_v1",
                **parameters,
            }
        ),
        encoding="utf-8",
    )
    config_payload = {
        "case": {"case_id": case.case_id},
        "policy_variant": "oracle_trajectory_v1",
        **parameters,
        "fixture_sha256": "11" * 32,
        "contract_sha256": "22" * 32,
        "drive_profile_sha256": "33" * 32,
        "harness_sha256": "44" * 32,
        "source": {"geometry_sha256": "55" * 32},
    }
    if mutation == "case_id":
        config_payload["case"] = {"case_id": "wrong_case"}
    elif mutation == "policy_variant":
        config_payload["policy_variant"] = "baseline"
    elif mutation == "lookahead_m":
        config_payload["lookahead_m"] = 0.5
    elif mutation == "fixture_hash":
        config_payload["fixture_sha256"] = "bad"
    elif mutation == "contract_hash":
        config_payload["contract_sha256"] = "bad"
    elif mutation == "drive_profile_hash":
        config_payload["drive_profile_sha256"] = "bad"
    elif mutation == "harness_hash":
        config_payload["harness_sha256"] = "bad"
    elif mutation == "geometry_hash":
        config_payload["source"] = {"geometry_sha256": "bad"}
    config_bytes = json.dumps(config_payload, sort_keys=True).encode("utf-8")
    if mutation != "missing_config":
        (case_dir / "config.json").write_bytes(config_bytes)
    if mutation not in {"missing_config", "missing_digest"}:
        digest = hashlib.sha256(config_bytes).hexdigest()
        if mutation == "tampered_digest":
            digest = "deadbeef"
        (case_dir / "config.sha256").write_text(digest + "\n", encoding="utf-8")

    with pytest.raises(harness.DiagnosticValidationError):
        harness._summary(
            tmp_path,
            (case,),
            policy_variant="oracle_trajectory_v1",
        )


def test_baseline_config_does_not_include_v1_controller_parameters(tmp_path: Path):
    case = load_tracking_cases(CONFIG_PATH)[0]
    reference = build_tracking_reference(case)
    episode = _fake_episode(reference)
    config = {
        "schema_version": "tracking_acceptance_v1",
        "policy_variant": "baseline",
        "lookahead_m": 0.5,
        "yaw_rate_feedback_gain_s": 0.15,
        "harness": {
            "scene_root": str(tmp_path),
            "fixture_path": str(tmp_path / "scene.usda"),
            "occupancy_path": str(tmp_path / "occupancy.npz"),
            "contract_path": str(tmp_path / "contract.yaml"),
            "drive_profile_path": str(tmp_path / "profile.json"),
        },
    }
    (tmp_path / "scene.usda").write_text("#usda 1.0\n", encoding="utf-8")
    (tmp_path / "contract.yaml").write_text("contract", encoding="utf-8")
    (tmp_path / "profile.json").write_text("{}", encoding="utf-8")
    np.savez(
        tmp_path / "occupancy.npz",
        grid=np.zeros((8, 8), dtype=np.uint8),
        global_origin_xy=np.asarray((0.0, 0.0)),
        resolution_m=np.asarray(0.1),
    )
    payload = harness._case_config_payload(
        case,
        reference,
        episode,
        config,
    )
    assert payload["policy_variant"] == "baseline"
    assert "smoothing_window_m" not in payload
    assert "curvature_preview_m" not in payload
    assert "lateral_curvature_gain_1_m2" not in payload
    assert "heading_curvature_gain_1_m" not in payload
    assert "max_feedback_curvature_1_m" not in payload
    assert "latency_preview_s" not in payload


@pytest.mark.parametrize("value", ["0", "nan"])
def test_harness_cli_rejects_invalid_lookahead(value: str):
    parser = harness._build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--config",
                str(CONFIG_PATH),
                "--run-root",
                "/tmp/diag",
                "--headless",
                "--lookahead-m",
                value,
            ]
        )


def test_diagnostic_fixture_is_ground_free_and_occupancy_is_all_free():
    fixture = CONFIG_PATH.parents[2] / "data" / "benchmark_v1" / "scenes" / "tracking_open_plane.usda"
    occupancy = CONFIG_PATH.parents[2] / "data" / "benchmark_v1" / "occupancy" / "tracking_open_plane_free.npz"
    source = fixture.read_text(encoding="utf-8")
    assert "defaultGroundPlane" not in source
    assert "CollisionAPI" not in source
    with np.load(occupancy, allow_pickle=False) as archive:
        assert set(("grid", "global_origin_xy", "resolution_m")) <= set(archive.files)
        assert archive["grid"].dtype == np.uint8
        assert np.all(archive["grid"] == 0)


def test_fixture_validation_rejects_external_collision_or_occupied_cells(tmp_path: Path):
    scene = tmp_path / "scene.usda"
    occupancy = tmp_path / "occupancy.npz"
    scene.write_text("CollisionAPI", encoding="utf-8")
    np.savez(
        occupancy,
        grid=np.asarray([[0, 1]], dtype=np.uint8),
        global_origin_xy=np.asarray((0.0, 0.0), dtype=np.float64),
        resolution_m=np.asarray(0.1, dtype=np.float64),
    )
    with pytest.raises(harness.DiagnosticValidationError, match="collision"):
        harness.validate_diagnostic_fixture(scene, occupancy)
