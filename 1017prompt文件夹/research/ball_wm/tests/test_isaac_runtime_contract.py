"""Isaac-free contract tests for kinodynamic runtime integration."""

from __future__ import annotations

import hashlib
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import benchmark.isaac_runtime as isaac_runtime
from benchmark.isaac_runtime import (
    IsaacBenchmarkRuntime,
    IsaacRuntimeConfig,
    _plan_kinodynamic_reference,
    _scene_obstacle_collision_paths,
)
from benchmark.dynamic_lattice_planning import DynamicPlanResult
from benchmark.kinodynamic_planning import KinodynamicPlanResult
from benchmark.manifest import BenchmarkEpisode, HumanReview
from benchmark.policies import PolicyObservation
from benchmark.runner import RuntimeStep


def _episode(episode_id: str = "runtime-contract") -> BenchmarkEpisode:
    return BenchmarkEpisode(
        episode_id=episode_id,
        split="dev",
        difficulty="E0",
        scene_uri="${SCENE_ROOT}/scene.usd",
        scene_sha256="11" * 32,
        occupancy_sha256="22" * 32,
        start_position_world_m=(-3.0, 0.0, 0.45),
        start_quaternion_world_wxyz=(1.0, 0.0, 0.0, 0.0),
        goal_position_world_m=(-2.1, 0.0, 0.45),
        euclidean_distance_m=0.9,
        oracle_path_length_m=0.9,
        seed=1,
        human_review=HumanReview("approved", "project_owner", "fixture"),
        classification_evidence={"line_is_free": True},
    )


def test_runtime_config_defaults_to_tracked_drive_profile(tmp_path: Path):
    config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
    )

    assert config.drive_profile_path.name == "drive_dynamics_v1.json"
    assert config.drive_profile_path.is_file()


def test_open_plane_mode_is_explicitly_opt_in(tmp_path: Path):
    formal = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
    )
    diagnostic = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
        diagnostic_open_plane=True,
    )

    assert formal.diagnostic_open_plane is False
    assert diagnostic.diagnostic_open_plane is True


def test_diagnostic_reference_visualization_path_is_finite_and_copied():
    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._reference_visualization_path = None
    points = np.asarray([[0.0, 0.0], [1.0, 0.5]], dtype=np.float64)

    runtime.set_reference_visualization_path(points)
    points[0, 0] = 9.0

    np.testing.assert_array_equal(runtime._reference_visualization_path, [[0.0, 0.0], [1.0, 0.5]])
    with pytest.raises(ValueError, match="shape"):
        runtime.set_reference_visualization_path([[0.0, 0.0]])
    with pytest.raises(ValueError, match="finite"):
        runtime.set_reference_visualization_path([[0.0, 0.0], [np.nan, 1.0]])


def test_diagnostic_reference_bypasses_internal_planners(monkeypatch):
    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime.config = IsaacRuntimeConfig(
        scene_root=Path("."),
        contract_path=Path("contract.yaml"),
        gui=False,
        record_video=False,
        diagnostic_open_plane=True,
    )
    injected = np.asarray(((0.0, 0.0), (0.5, 0.25), (1.0, 0.0)), dtype=np.float64)
    runtime._reference_visualization_path = injected
    runtime._oracle_path = np.empty((0, 2), dtype=np.float64)
    runtime._kinodynamic_plan = object()

    def planner_must_not_run(*args, **kwargs):
        raise AssertionError("diagnostic reference must bypass internal planner")

    monkeypatch.setattr(isaac_runtime, "astar_path", planner_must_not_run)
    monkeypatch.setattr(isaac_runtime, "_plan_kinodynamic_reference", planner_must_not_run)

    runtime._prepare_reference_planning(_episode())

    np.testing.assert_array_equal(runtime._oracle_path, injected)
    assert runtime._kinodynamic_plan is None


def test_default_ground_is_never_in_obstacle_contact_filter():
    assert _scene_obstacle_collision_paths(
        "/World/Scene",
        [
            "/World/defaultGroundPlane",
            "/World/defaultGroundPlane/Collision",
            "/World/Scene/defaultGroundPlane",
            "/World/Scene/defaultGroundPlane/Collision",
            "/World/Scene/Obstacle/Collision",
        ],
    ) == ["/World/Scene/Obstacle/Collision"]


def test_reset_state_zero_is_identical_after_variable_camera_priming(
    tmp_path: Path,
):
    """Camera warmup must not leak variable physics into the captured reset state."""

    class FakeWorld:
        def __init__(self) -> None:
            self.current_time = 40.0
            self.current_time_step_index = 200
            self.events: list[str] = []
            self.step_calls: list[bool] = []
            self.post_prime_step_calls: list[bool] = []
            self._prime_complete = False
            self.robot: FakeRobot | None = None

        def step(self, *, render: bool) -> None:
            self.events.append(f"world.step(render={render})")
            self.step_calls.append(render)
            if self._prime_complete:
                self.post_prime_step_calls.append(render)
            self.current_time += 0.02
            self.current_time_step_index += 1
            if self.robot is not None:
                self.robot._apply_pending_state()
                if render:
                    self.robot._camera_warmup_drift()

        def mark_prime_complete(self) -> None:
            self._prime_complete = True
            self.events.append("camera_prime.complete")

    class FakeRobot:
        def __init__(self, world: FakeWorld) -> None:
            self._world = world
            world.robot = self
            self._position = np.asarray((9.0, -4.0, 0.45), dtype=np.float64)
            self._quaternion = np.asarray((0.98, 0.0, 0.0, 0.2), dtype=np.float64)
            self._linear_velocity = np.asarray((0.7, -0.2, 0.1), dtype=np.float64)
            self._angular_velocity = np.asarray((0.1, 0.2, -0.3), dtype=np.float64)
            self._joint_positions = np.asarray((0.4, -0.2, 0.1), dtype=np.float64)
            self._joint_velocities = np.asarray((0.6, -0.5, 0.3), dtype=np.float64)
            self._pending_state: dict[str, np.ndarray] = {}

        def _camera_warmup_drift(self) -> None:
            self._position += np.asarray((0.003, -0.002, 0.0))
            self._linear_velocity += np.asarray((0.02, -0.01, 0.0))
            self._angular_velocity += np.asarray((0.01, -0.02, 0.03))
            self._joint_positions += np.asarray((0.004, -0.003, 0.002))
            self._joint_velocities += np.asarray((0.03, -0.02, 0.01))

        def _apply_pending_state(self) -> None:
            for name, value in self._pending_state.items():
                setattr(self, name, value.copy())
            self._pending_state.clear()

        def set_world_pose(self, *, position: object, orientation: object) -> None:
            self._world.events.append("robot.set_world_pose")
            self._pending_state["_position"] = np.asarray(position, dtype=np.float64)
            self._pending_state["_quaternion"] = np.asarray(
                orientation, dtype=np.float64
            )

        def set_linear_velocity(self, value: object) -> None:
            self._world.events.append("robot.set_linear_velocity")
            self._pending_state["_linear_velocity"] = np.asarray(
                value, dtype=np.float64
            )

        def set_angular_velocity(self, value: object) -> None:
            self._world.events.append("robot.set_angular_velocity")
            self._pending_state["_angular_velocity"] = np.asarray(
                value, dtype=np.float64
            )

        def set_joint_positions(self, value: object) -> None:
            self._world.events.append("robot.set_joint_positions")
            self._pending_state["_joint_positions"] = np.asarray(
                value, dtype=np.float64
            )

        def set_joint_velocities(self, value: object) -> None:
            self._world.events.append("robot.set_joint_velocities")
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

    class FakeSmoother:
        def __init__(self, world: FakeWorld) -> None:
            self.events = world.events
            self._current_action = np.asarray((0.75, -0.4), dtype=np.float64)
            self.reset_calls = 0

        @property
        def current_action(self) -> np.ndarray:
            return self._current_action.copy()

        def reset(self) -> None:
            self.reset_calls += 1
            self.events.append("smoother.reset")
            self._current_action.fill(0.0)

    class TrackingAction(np.ndarray):
        events: list[str] = []

        def fill(self, value: object, *args: object, **kwargs: object) -> None:
            TrackingAction.events.append("previous_action.fill")
            super().fill(value, *args, **kwargs)

    scene_path = tmp_path / "scene.usd"
    occupancy_path = tmp_path / "occupancy.npz"
    contract_path = tmp_path / "contract.yaml"
    scene_path.write_bytes(b"scene")
    occupancy_path.write_bytes(b"occupancy")
    contract_path.write_text("contract", encoding="utf-8")
    episode = replace(
        _episode(),
        scene_sha256=hashlib.sha256(b"scene").hexdigest(),
        occupancy_sha256=hashlib.sha256(b"occupancy").hexdigest(),
    )

    def run_with_prime_steps(prime_steps: int, *, trace_enabled: bool = True):
        runtime = IsaacBenchmarkRuntime(
            IsaacRuntimeConfig(
                scene_root=tmp_path,
                contract_path=contract_path,
                gui=False,
                record_video=False,
                occupancy_path=occupancy_path,
                diagnostic_physics_trace=trace_enabled,
            )
        )
        world = FakeWorld()
        robot = FakeRobot(world)
        smoother = FakeSmoother(world)
        runtime._world = world
        runtime._robot = robot
        runtime._action_smoother = smoother
        TrackingAction.events = world.events
        runtime._previous_action = np.asarray(
            (0.6, -0.25), dtype=np.float32
        ).view(TrackingAction)

        def initialize(_scene_path, _occupancy_path, _episode):
            # This mirrors the existing runtime's variable camera priming.
            for _ in range(prime_steps):
                world.step(render=True)
            world.mark_prime_complete()

        original_establish_epoch = runtime._establish_clock_epoch

        def establish_epoch():
            world.events.append("clock_epoch.establish")
            original_establish_epoch()

        original_record_diagnostic = runtime._record_physics_diagnostic

        def record_diagnostic(*args, **kwargs):
            world.events.append("diagnostic.record")
            return original_record_diagnostic(*args, **kwargs)

        runtime._record_physics_diagnostic = record_diagnostic

        def capture():
            world.events.append("capture")
            position, quaternion = robot.get_world_pose()
            relative_time, relative_step, raw_time, raw_step = runtime._clock_metadata()
            return SimpleNamespace(
                observation=_reset_observation(),
                root_position_world_m=position,
                root_quaternion_world_wxyz=quaternion,
                linear_velocity_world_m_s=robot.get_linear_velocity(),
                angular_velocity_world_rad_s=robot.get_angular_velocity(),
                joint_position_rad=robot.get_joint_positions(),
                joint_velocity_rad_s=robot.get_joint_velocities(),
                smoother_state=smoother.current_action,
                previous_action=runtime._previous_action.copy(),
                simulation_time_s=relative_time,
                physics_step_index=relative_step,
                raw_simulation_time_s=raw_time,
                raw_physics_step_index=raw_step,
            )

        runtime._initialize = initialize
        runtime._establish_clock_epoch = establish_epoch
        runtime._capture = capture
        return runtime.reset(episode), world, smoother, runtime

    one_step, one_world, one_smoother, one_runtime = run_with_prime_steps(1)
    six_steps, six_world, six_smoother, six_runtime = run_with_prime_steps(6)

    for field in (
        "root_position_world_m",
        "root_quaternion_world_wxyz",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "smoother_state",
        "previous_action",
    ):
        np.testing.assert_array_equal(
            getattr(one_step, field), getattr(six_steps, field)
        )
    assert one_step.simulation_time_s == pytest.approx(0.0)
    assert six_steps.simulation_time_s == pytest.approx(0.0)
    assert one_step.physics_step_index == 0
    assert six_steps.physics_step_index == 0
    assert one_step.smoother_state.tolist() == [0.0, 0.0]
    assert six_steps.smoother_state.tolist() == [0.0, 0.0]
    assert one_step.previous_action.tolist() == [0.0, 0.0]
    assert six_steps.previous_action.tolist() == [0.0, 0.0]
    np.testing.assert_array_equal(
        one_step.root_position_world_m,
        np.asarray(episode.start_position_world_m, dtype=np.float32),
    )
    np.testing.assert_array_equal(
        six_steps.root_position_world_m,
        np.asarray(episode.start_position_world_m, dtype=np.float32),
    )
    np.testing.assert_array_equal(one_step.linear_velocity_world_m_s, np.zeros(3))
    np.testing.assert_array_equal(six_steps.linear_velocity_world_m_s, np.zeros(3))
    np.testing.assert_array_equal(one_step.angular_velocity_world_rad_s, np.zeros(3))
    np.testing.assert_array_equal(six_steps.angular_velocity_world_rad_s, np.zeros(3))
    np.testing.assert_array_equal(one_step.joint_position_rad, np.zeros(3))
    np.testing.assert_array_equal(six_steps.joint_position_rad, np.zeros(3))
    np.testing.assert_array_equal(one_step.joint_velocity_rad_s, np.zeros(3))
    np.testing.assert_array_equal(six_steps.joint_velocity_rad_s, np.zeros(3))
    assert one_world.post_prime_step_calls == [False]
    assert six_world.post_prime_step_calls == [False]
    expected_post_prime_events = [
        "robot.set_world_pose",
        "robot.set_linear_velocity",
        "robot.set_angular_velocity",
        "robot.set_joint_positions",
        "robot.set_joint_velocities",
        "smoother.reset",
        "previous_action.fill",
        "world.step(render=False)",
        "clock_epoch.establish",
        "diagnostic.record",
        "capture",
    ]
    for world in (one_world, six_world):
        prime_complete_index = world.events.index("camera_prime.complete")
        assert world.events[prime_complete_index + 1 :] == expected_post_prime_events
    assert one_smoother.reset_calls == 1
    assert six_smoother.reset_calls == 1

    expected_diagnostic_fields = {
        "raw_command_target",
        "clipped_target",
        "smoothed_target",
        "applied_action",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "position_world_xy",
        "root_position_world_m",
        "root_quaternion_world_wxyz",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "yaw_world_rad",
        "roll_world_rad",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
        "action_block_index",
        "physics_step_in_block",
    }
    for runtime in (one_runtime, six_runtime):
        trace = runtime.physics_diagnostics
        assert set(trace) == expected_diagnostic_fields
        assert all(values.shape[0] == 1 for values in trace.values())
        np.testing.assert_array_equal(
            trace["root_position_world_m"][0],
            np.asarray(episode.start_position_world_m, dtype=np.float32),
        )
        np.testing.assert_array_equal(
            trace["root_quaternion_world_wxyz"][0],
            np.asarray(episode.start_quaternion_world_wxyz, dtype=np.float32),
        )
        np.testing.assert_array_equal(trace["linear_velocity_world_m_s"][0], np.zeros(3))
        np.testing.assert_array_equal(trace["angular_velocity_world_rad_s"][0], np.zeros(3))
        np.testing.assert_array_equal(trace["joint_position_rad"][0], np.zeros(3))
        np.testing.assert_array_equal(trace["joint_velocity_rad_s"][0], np.zeros(3))
        np.testing.assert_array_equal(trace["raw_command_target"][0], np.zeros(2))
        np.testing.assert_array_equal(trace["clipped_target"][0], np.zeros(2))
        np.testing.assert_array_equal(trace["smoothed_target"][0], np.zeros(2))
        np.testing.assert_array_equal(trace["applied_action"][0], np.zeros(2))
        np.testing.assert_array_equal(trace["simulation_time_s"], [0.0])
        np.testing.assert_array_equal(trace["physics_step_index"], [0])
        np.testing.assert_array_equal(trace["action_block_index"], [0])
        np.testing.assert_array_equal(trace["physics_step_in_block"], [0])

    _, disabled_world, _, disabled_runtime = run_with_prime_steps(
        3, trace_enabled=False
    )
    assert all(values.size == 0 for values in disabled_runtime.physics_diagnostics.values())
    assert disabled_world.post_prime_step_calls == [False]
    assert "diagnostic.record" not in disabled_world.events


def test_pure_runtime_planning_adapter_uses_episode_pose_and_inflated_grid():
    config = IsaacRuntimeConfig(
        scene_root=Path("."),
        contract_path=Path("contract.yaml"),
        gui=False,
        record_video=False,
    )

    result = _plan_kinodynamic_reference(
        occupancy_grid=np.zeros((81, 81), dtype=np.uint8),
        resolution_m=0.1,
        origin_xy=(-4.0, -4.0),
        episode=_episode(),
        drive_profile_path=config.drive_profile_path,
    )

    assert result.status == "success"
    assert result.waypoints[0].x_m == pytest.approx(-3.0)
    assert result.waypoints[0].drive_direction == "forward"


def test_runtime_exposes_plan_only_for_active_episode():
    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._episode = _episode()
    runtime._kinodynamic_plan = KinodynamicPlanResult(
        status="unreachable",
        waypoints=(),
        expanded_nodes=12,
        path_length_m=0.0,
        direction_changes=0,
    )

    assert runtime.kinodynamic_plan(_episode()) == runtime._kinodynamic_plan
    with pytest.raises(ValueError, match="active episode"):
        runtime.kinodynamic_plan(_episode("other"))


def test_legacy_runtime_step_keeps_measured_telemetry_unavailable():
    step = _reset_step()

    assert step.simulation_time_s is None
    assert step.physics_step_index is None
    assert step.raw_simulation_time_s is None
    assert step.raw_physics_step_index is None
    assert step.roll_world_rad is None


def test_nan_runtime_telemetry_is_explicitly_unavailable():
    step = RuntimeStep(
        observation=_reset_observation(),
        truth_clearance_m=1.0,
        physical_contact=False,
        collision=False,
        frames={},
        simulation_time_s=np.nan,
        physics_step_index=np.nan,
        raw_simulation_time_s=np.nan,
        raw_physics_step_index=np.nan,
        roll_world_rad=np.nan,
    )

    assert step.simulation_time_s is None
    assert step.physics_step_index is None
    assert step.raw_simulation_time_s is None
    assert step.raw_physics_step_index is None
    assert step.roll_world_rad is None


def test_runtime_step_requires_both_independent_clock_quantities():
    observation = _reset_observation()
    with pytest.raises(ValueError, match="simulation_time_s and physics_step_index"):
        RuntimeStep(
            observation=observation,
            truth_clearance_m=1.0,
            physical_contact=False,
            collision=False,
            frames={},
            simulation_time_s=0.0,
        )
    with pytest.raises(ValueError, match="raw_simulation_time_s and raw_physics_step_index"):
        RuntimeStep(
            observation=observation,
            truth_clearance_m=1.0,
            physical_contact=False,
            collision=False,
            frames={},
            raw_physics_step_index=100,
        )


def test_roll_is_extracted_from_the_same_world_quaternion_as_yaw():
    roll = 0.31
    yaw = 0.0
    quaternion = np.asarray(
        (
            np.cos(roll / 2.0) * np.cos(yaw / 2.0),
            np.sin(roll / 2.0) * np.cos(yaw / 2.0),
            0.0,
            np.cos(roll / 2.0) * np.sin(yaw / 2.0),
        ),
        dtype=np.float64,
    )

    assert isaac_runtime._yaw_from_wxyz(quaternion) == pytest.approx(yaw)
    roll_extractor = getattr(isaac_runtime, "_roll_from_wxyz", lambda _value: np.nan)
    assert roll_extractor(quaternion) == pytest.approx(roll)


def test_execute_records_independent_world_clock_after_five_steps_and_one_render():
    class FakeWorld:
        def __init__(self) -> None:
            self.step_calls: list[bool] = []
            self.render_calls = 0
            self.current_time = 12.345
            self.current_time_step_index = 91

        def step(self, *, render: bool) -> None:
            self.step_calls.append(render)
            if not render:
                self.current_time += 0.02
                self.current_time_step_index += 1

        def render(self) -> None:
            self.render_calls += 1

    class FakeSmoother:
        def step(
            self,
            target: np.ndarray,
            _minimum: tuple[float, float],
            _maximum: tuple[float, float],
        ) -> np.ndarray:
            return np.asarray(target, dtype=np.float32)

    class FakeController:
        def __init__(self) -> None:
            self.actions: list[object] = []

        def apply_action(self, action: object) -> None:
            self.actions.append(action)

    class FakeAction:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._episode = object()
    runtime.config = SimpleNamespace(gui=False)
    runtime._world = FakeWorld()
    runtime._action_protocol = SimpleNamespace(
        controls_per_block=5,
        minimum=(-2.0, -0.5),
        maximum=(2.0, 0.5),
        clip_block=lambda action: np.asarray(action, dtype=np.float32),
    )
    runtime._action_smoother = FakeSmoother()
    runtime._controller = FakeController()
    runtime._ArticulationAction = FakeAction
    runtime._previous_action = np.zeros(2, dtype=np.float32)
    runtime._history_actions = []
    runtime._clock_epoch_current_time = None
    runtime._clock_epoch_current_time_step_index = None
    runtime._establish_clock_epoch()

    def capture() -> RuntimeStep:
        relative_time, relative_step, raw_time, raw_step = runtime._clock_metadata()
        return RuntimeStep(
            observation=_reset_observation(),
            truth_clearance_m=1.0,
            physical_contact=False,
            collision=False,
            frames={},
            simulation_time_s=relative_time,
            physics_step_index=relative_step,
            raw_simulation_time_s=raw_time,
            raw_physics_step_index=raw_step,
            roll_world_rad=0.0,
        )

    runtime._capture = capture

    initial = runtime._capture()

    result = runtime.execute(np.zeros(10, dtype=np.float32))

    assert initial.simulation_time_s == pytest.approx(0.0)
    assert initial.physics_step_index == 0
    assert initial.raw_simulation_time_s == pytest.approx(12.345)
    assert initial.raw_physics_step_index == 91
    assert result.simulation_time_s == pytest.approx(0.1)
    assert result.physics_step_index == 5
    assert result.raw_simulation_time_s == pytest.approx(12.445)
    assert result.raw_physics_step_index == 96
    assert runtime._world.step_calls == [False] * 5
    assert runtime._world.render_calls == 1
    assert runtime._world.current_time == pytest.approx(12.445)
    assert runtime._world.current_time_step_index == 96
    assert len(runtime._controller.actions) == 5


def test_runtime_config_rejects_nonpositive_nonfinite_or_formal_gain_overrides(
    tmp_path: Path,
):
    base = dict(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
    )
    defaults = IsaacRuntimeConfig(**base)
    assert defaults.diagnostic_physics_trace is False
    assert defaults.diagnostic_auxiliary_kp is None
    assert defaults.diagnostic_auxiliary_kd is None

    for field in ("diagnostic_auxiliary_kp", "diagnostic_auxiliary_kd"):
        for invalid in (0.0, -1.0, np.nan, np.inf, True):
            with pytest.raises(ValueError, match="finite and positive"):
                IsaacRuntimeConfig(**base, **{field: invalid})
    with pytest.raises(ValueError, match="diagnostic_open_plane"):
        IsaacRuntimeConfig(**base, diagnostic_auxiliary_kp=123.0)


def test_runtime_config_defaults_to_no_auxiliary_velocity_override(tmp_path: Path):
    config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
    )

    assert config.diagnostic_auxiliary_target_rate_rad_s is None
    assert config.diagnostic_auxiliary_velocity_limit_rad_s is None


def test_runtime_config_accepts_auxiliary_velocity_override_only_in_diagnostics(
    tmp_path: Path,
):
    config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
        diagnostic_open_plane=True,
        diagnostic_auxiliary_target_rate_rad_s=math.radians(20.0),
        diagnostic_auxiliary_velocity_limit_rad_s=0.25,
    )

    assert config.diagnostic_auxiliary_target_rate_rad_s == pytest.approx(
        math.radians(20.0)
    )
    assert config.diagnostic_auxiliary_velocity_limit_rad_s == pytest.approx(0.25)

    with pytest.raises(ValueError, match="diagnostic_open_plane"):
        IsaacRuntimeConfig(
            scene_root=tmp_path,
            contract_path=tmp_path / "contract.yaml",
            gui=False,
            record_video=False,
            diagnostic_auxiliary_velocity_limit_rad_s=0.25,
        )


def test_runtime_config_accepts_formal_auxiliary_velocity_override(tmp_path: Path):
    config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
        auxiliary_velocity_limit_rad_s=0.30,
    )

    assert config.diagnostic_open_plane is False
    assert config.auxiliary_velocity_limit_rad_s == pytest.approx(0.30)


@pytest.mark.parametrize(
    "field,value",
    [
        ("diagnostic_auxiliary_target_rate_rad_s", 0.0),
        ("diagnostic_auxiliary_velocity_limit_rad_s", -1.0),
        ("diagnostic_auxiliary_velocity_limit_rad_s", np.nan),
    ],
)
def test_runtime_config_rejects_invalid_auxiliary_velocity_overrides(
    tmp_path: Path, field: str, value: object
):
    with pytest.raises(ValueError, match="finite and positive"):
        IsaacRuntimeConfig(
            scene_root=tmp_path,
            contract_path=tmp_path / "contract.yaml",
            gui=False,
            record_video=False,
            diagnostic_open_plane=True,
            diagnostic_auxiliary_target_rate_rad_s=0.25
            if field != "diagnostic_auxiliary_target_rate_rad_s"
            else value,
            diagnostic_auxiliary_velocity_limit_rad_s=0.25
            if field != "diagnostic_auxiliary_velocity_limit_rad_s"
            else value,
        )


def test_runtime_config_accepts_diagnostic_trace_and_gain_overrides(tmp_path: Path):
    config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=tmp_path / "contract.yaml",
        gui=False,
        record_video=False,
        diagnostic_open_plane=True,
        diagnostic_physics_trace=True,
        diagnostic_auxiliary_kp=123,
        diagnostic_auxiliary_kd=45.5,
    )

    assert config.diagnostic_physics_trace is True
    assert config.diagnostic_auxiliary_kp == pytest.approx(123.0)
    assert config.diagnostic_auxiliary_kd == pytest.approx(45.5)


def test_runtime_gain_arrays_use_baseline_or_explicit_diagnostic_overrides(
    tmp_path: Path,
):
    baseline = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(tmp_path, tmp_path / "contract.yaml", False, False)
    )
    baseline_kps, baseline_kds = baseline._configured_articulation_gains()
    np.testing.assert_array_equal(baseline_kps, [0.0, 300.0, 0.0])
    np.testing.assert_array_equal(baseline_kds, [80.0, 60.0, 0.0])

    diagnostic = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            tmp_path,
            tmp_path / "contract.yaml",
            False,
            False,
            diagnostic_open_plane=True,
            diagnostic_auxiliary_kp=123.0,
            diagnostic_auxiliary_kd=45.0,
        )
    )
    kps, kds = diagnostic._configured_articulation_gains()
    np.testing.assert_array_equal(kps, [0.0, 123.0, 0.0])
    np.testing.assert_array_equal(kds, [80.0, 45.0, 0.0])


def test_diagnostic_execute_records_post_step_50hz_truth_and_copies_arrays():
    class FakeWorld:
        def __init__(self) -> None:
            self.step_calls: list[bool] = []
            self.render_calls = 0
            self.current_time = 8.0
            self.current_time_step_index = 400

        def step(self, *, render: bool) -> None:
            self.step_calls.append(render)
            if not render:
                self.current_time += 0.02
                self.current_time_step_index += 1

        def render(self) -> None:
            self.render_calls += 1

    class FakeSmoother:
        def __init__(self) -> None:
            self.targets: list[np.ndarray] = []

        def step(
            self,
            target: np.ndarray,
            _minimum: tuple[float, float],
            _maximum: tuple[float, float],
        ) -> np.ndarray:
            target = np.asarray(target, dtype=np.float32).copy()
            self.targets.append(target)
            return np.asarray((target[0] * 0.5, target[1] * 0.5), dtype=np.float32)

    class FakeController:
        def __init__(self) -> None:
            self.actions: list[object] = []

        def apply_action(self, action: object) -> None:
            self.actions.append(action)

    class FakeAction:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class FakeRobot:
        dof_names = ("main", "aux", "unused")

        def __init__(self) -> None:
            self.sample = 0

        def _next(self) -> int:
            self.sample += 1
            return self.sample

        def get_world_pose(self):
            index = self._next()
            return np.asarray((float(index), 0.25, 0.45)), np.asarray((1.0, 0.0, 0.0, 0.0))

        def get_joint_positions(self):
            return np.asarray((101.0 + self.sample, 201.0 + self.sample, 301.0))

        def get_joint_velocities(self):
            return np.asarray((401.0 + self.sample, 501.0 + self.sample, 601.0))

        def get_linear_velocity(self):
            return np.asarray((1.0, 2.0, 3.0))

        def get_angular_velocity(self):
            return np.asarray((4.0, 5.0, 6.0))

    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._episode = object()
    runtime.config = SimpleNamespace(
        gui=False,
        diagnostic_physics_trace=True,
    )
    runtime._world = FakeWorld()
    runtime._robot = FakeRobot()
    runtime._action_protocol = SimpleNamespace(
        controls_per_block=5,
        minimum=(-2.0, -0.5),
        maximum=(2.0, 0.5),
        clip_block=lambda action: np.clip(
            np.asarray(action, dtype=np.float32),
            np.tile(np.asarray((-2.0, -0.5), dtype=np.float32), 5),
            np.tile(np.asarray((2.0, 0.5), dtype=np.float32), 5),
        ),
    )
    runtime._action_smoother = FakeSmoother()
    runtime._controller = FakeController()
    runtime._ArticulationAction = FakeAction
    runtime._previous_action = np.zeros(2, dtype=np.float32)
    runtime._history_actions = []
    runtime._clock_epoch_current_time = None
    runtime._clock_epoch_current_time_step_index = None
    runtime._physics_diagnostics = runtime._new_physics_diagnostics()
    runtime._diagnostic_block_index = 0
    runtime._establish_clock_epoch()

    def capture() -> RuntimeStep:
        return RuntimeStep(
            observation=_reset_observation(),
            truth_clearance_m=1.0,
            physical_contact=False,
            collision=False,
            frames={},
        )

    runtime._capture = capture

    runtime._record_physics_diagnostic(
        np.zeros(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
        action_block_index=0,
        physics_step_in_block=0,
    )
    runtime.execute(np.asarray([3.0, 0.8] * 5, dtype=np.float32))
    trace = runtime.physics_diagnostics
    assert runtime._world.step_calls == [False] * 5
    assert runtime._world.render_calls == 1
    assert set(trace) >= {
        "raw_command_target",
        "clipped_target",
        "smoothed_target",
        "applied_action",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "position_world_xy",
        "root_position_world_m",
        "root_quaternion_world_wxyz",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "yaw_world_rad",
        "roll_world_rad",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
    }
    assert trace["raw_command_target"].shape == (6, 2)
    np.testing.assert_array_equal(trace["raw_command_target"][0], [0.0, 0.0])
    np.testing.assert_array_equal(trace["raw_command_target"][1:, 0], 3.0)
    np.testing.assert_array_equal(trace["clipped_target"][0], [0.0, 0.0])
    np.testing.assert_array_equal(trace["clipped_target"][1:, 0], 2.0)
    np.testing.assert_array_equal(trace["clipped_target"][1:, 1], 0.5)
    np.testing.assert_allclose(
        trace["applied_action"], [[0.0, 0.0]] + [[1.0, 0.25]] * 5
    )
    np.testing.assert_allclose(
        trace["raw_simulation_time_s"], [8.0, 8.02, 8.04, 8.06, 8.08, 8.10]
    )
    np.testing.assert_allclose(
        trace["simulation_time_s"], [0.0, 0.02, 0.04, 0.06, 0.08, 0.10]
    )
    np.testing.assert_array_equal(trace["physics_step_index"], [0, 1, 2, 3, 4, 5])
    np.testing.assert_array_equal(
        trace["raw_physics_step_index"], [400, 401, 402, 403, 404, 405]
    )
    np.testing.assert_array_equal(trace["action_block_index"], [0, 0, 0, 0, 0, 0])
    np.testing.assert_array_equal(trace["physics_step_in_block"], [0, 1, 2, 3, 4, 5])
    np.testing.assert_array_equal(
        trace["joint_position_rad"][:, 0], [102, 103, 104, 105, 106, 107]
    )
    np.testing.assert_array_equal(
        trace["joint_velocity_rad_s"][:, 0], [402, 403, 404, 405, 406, 407]
    )
    assert len(runtime._controller.actions) == 5
    np.testing.assert_array_equal(
        runtime._controller.actions[0].kwargs["joint_velocities"],
        [1.0, np.nan, np.nan],
    )

    trace["raw_command_target"][1, 0] = -99.0
    assert runtime.physics_diagnostics["raw_command_target"][1, 0] == pytest.approx(3.0)


def test_diagnostic_reset_clears_physics_trace():
    runtime = object.__new__(IsaacBenchmarkRuntime)
    runtime._closed = False
    runtime._simulation_app = None
    runtime._episode = None
    runtime._physics_diagnostics = {
        "raw_command_target": [np.asarray((1.0, 2.0), dtype=np.float32)]
    }
    runtime.config = SimpleNamespace(
        scene_root=Path("."),
        contract_path=Path("missing-contract"),
        occupancy_path=Path("missing-occupancy"),
    )
    with pytest.raises(FileNotFoundError):
        runtime.reset(_episode())
    assert all(len(values) == 0 for values in runtime._physics_diagnostics.values())


def _reset_observation() -> PolicyObservation:
    proprio = np.zeros((5, 17), dtype=np.float32)
    proprio[-1, :3] = (0.3, 0.4, 0.2)
    proprio[-1, 10] = np.deg2rad(20.0)
    proprio[-1, 12] = 0.75
    proprio[-1, 15:] = (1.25, np.deg2rad(-10.0))
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray((-2.8, 0.25), dtype=np.float64),
        yaw_world_rad=0.35,
    )


def _reset_step() -> RuntimeStep:
    return RuntimeStep(
        observation=_reset_observation(),
        truth_clearance_m=1.0,
        physical_contact=False,
        collision=False,
        frames={},
    )


def _reset_without_isaac(
    runtime: IsaacBenchmarkRuntime,
    episode: BenchmarkEpisode,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> BenchmarkEpisode:
    scene = tmp_path / "scene.usd"
    occupancy = tmp_path / "occupancy.npz"
    contract = tmp_path / "contract.yaml"
    scene.write_bytes(b"scene")
    occupancy.write_bytes(b"occupancy")
    contract.write_text("contract", encoding="utf-8")
    episode = replace(
        episode,
        scene_sha256=hashlib.sha256(b"scene").hexdigest(),
        occupancy_sha256=hashlib.sha256(b"occupancy").hexdigest(),
    )
    runtime.config = IsaacRuntimeConfig(
        scene_root=tmp_path,
        contract_path=contract,
        gui=False,
        record_video=False,
        occupancy_path=occupancy,
    )

    def initialize(_scene_path, _occupancy_path, _episode):
        runtime._inflated_grid = np.asarray(((0, 1), (0, 0)), dtype=np.uint8)
        runtime._occupancy = SimpleNamespace(
            resolution_m=0.1,
            global_origin_xy=np.asarray((-4.0, -4.0), dtype=np.float64),
        )

    monkeypatch.setattr(runtime, "_initialize", initialize)
    monkeypatch.setattr(runtime, "_capture", _reset_step)
    assert runtime.reset(episode).observation is not None
    return episode


def test_dynamic_plan_uses_nonzero_reset_observation_and_frozen_runtime_map(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(tmp_path, tmp_path / "unused", False, False)
    )
    episode = _reset_without_isaac(runtime, _episode(), tmp_path, monkeypatch)
    planned = DynamicPlanResult(status="unreachable", expanded_nodes=17)
    captured = {}

    def plan(request, drive_model):
        captured["request"] = request
        captured["drive_model"] = drive_model
        return planned

    monkeypatch.setattr(isaac_runtime, "plan_dynamic_lattice", plan, raising=False)

    assert runtime.dynamic_plan(episode) is planned
    request = captured["request"]
    state = request.initial_state
    assert (state.x_m, state.y_m, state.yaw_rad) == pytest.approx(
        (-2.8, 0.25, 0.35)
    )
    assert state.main_command_rad_s == pytest.approx(0.75)
    assert state.auxiliary_angle_rad == pytest.approx(np.deg2rad(20.0))
    assert state.speed_m_s == pytest.approx(0.5)
    assert state.drive_direction == "forward"
    np.testing.assert_array_equal(request.occupancy_grid, ((0, 1), (0, 0)))
    assert request.resolution_m == pytest.approx(0.1)
    assert request.origin_xy == pytest.approx((-4.0, -4.0))
    assert request.goal_xy == pytest.approx(episode.goal_position_world_m[:2])


def test_dynamic_plan_is_cached_per_active_episode_and_rejects_other_episode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(tmp_path, tmp_path / "unused", False, False)
    )
    episode = _reset_without_isaac(runtime, _episode(), tmp_path, monkeypatch)
    planned = DynamicPlanResult(status="node_budget_exhausted", expanded_nodes=500_000)
    calls = 0

    def plan(_request, _drive_model):
        nonlocal calls
        calls += 1
        return planned

    monkeypatch.setattr(isaac_runtime, "plan_dynamic_lattice", plan, raising=False)

    assert runtime.dynamic_plan(episode) is planned
    assert runtime.dynamic_plan(episode) is planned
    assert calls == 1
    with pytest.raises(ValueError, match="active episode"):
        runtime.dynamic_plan(_episode("other"))
    assert calls == 1
