"""Pure closed-loop tests for the strict Benchmark V1 episode runner."""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from benchmark import policies as benchmark_policies
from benchmark.manifest import BenchmarkEpisode, HumanReview
from benchmark.kinodynamic_planning import (
    KinodynamicPlanResult,
    KinodynamicWaypoint,
)
from benchmark.dynamic_lattice_planning import DynamicPlanResult, DynamicWaypoint
from benchmark.policies import PolicyObservation
from benchmark.runner import (
    RunnerConfig,
    RuntimeStep,
    run_episode,
    world_goal_to_base,
)
from scripts import run_tracking_acceptance as tracking_acceptance_harness
from training.action_protocol import ActionProtocol


def protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def episode() -> BenchmarkEpisode:
    return BenchmarkEpisode(
        episode_id="runner_e0_0001",
        split="golden",
        difficulty="E0",
        scene_uri="${SCENE_ROOT}/scene.usd",
        scene_sha256="01" * 32,
        occupancy_sha256="02" * 32,
        start_position_world_m=(0.0, 0.0, 0.45),
        start_quaternion_world_wxyz=(1.0, 0.0, 0.0, 0.0),
        goal_position_world_m=(1.0, 0.0, 0.45),
        euclidean_distance_m=1.0,
        oracle_path_length_m=1.0,
        seed=3072,
        human_review=HumanReview("approved", "project_owner", "runner fixture"),
        classification_evidence={"line_is_free": True},
    )


def observation(
    position: tuple[float, float],
    yaw: float = 0.0,
    proprio_last: tuple[float, ...] | None = None,
) -> PolicyObservation:
    proprio = np.zeros((5, 17), dtype=np.float32)
    if proprio_last is not None:
        proprio[-1] = np.asarray(proprio_last, dtype=np.float32)
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray(position, dtype=np.float64),
        yaw_world_rad=yaw,
    )


def runtime_step(
    position: tuple[float, float],
    *,
    yaw: float = 0.0,
    proprio_last: tuple[float, ...] | None = None,
    collision: bool = False,
    clearance_m: float = 1.0,
    simulation_time_s: float | None = None,
    physics_step_index: int | None = None,
    raw_simulation_time_s: float | None = None,
    raw_physics_step_index: int | None = None,
    roll_world_rad: float | None = None,
) -> RuntimeStep:
    return RuntimeStep(
        observation=observation(position, yaw=yaw, proprio_last=proprio_last),
        truth_clearance_m=clearance_m,
        physical_contact=collision,
        collision=collision,
        frames={},
        simulation_time_s=simulation_time_s,
        physics_step_index=physics_step_index,
        raw_simulation_time_s=raw_simulation_time_s,
        raw_physics_step_index=raw_physics_step_index,
        roll_world_rad=roll_world_rad,
    )


class SequenceRuntime:
    def __init__(self, steps: list[RuntimeStep], events: list[str]):
        self.steps = list(steps)
        self.events = events
        self.executed: list[np.ndarray] = []

    def reset(self, task: BenchmarkEpisode) -> RuntimeStep:
        self.events.append("runtime_reset")
        return self.steps[0]

    def execute(self, action_block: np.ndarray) -> RuntimeStep:
        self.events.append("execute")
        self.executed.append(np.asarray(action_block).copy())
        index = min(len(self.executed), len(self.steps) - 1)
        return self.steps[index]

    def close(self) -> None:
        self.events.append("close")


class FailingRuntime(SequenceRuntime):
    def execute(self, action_block: np.ndarray) -> RuntimeStep:
        self.events.append("execute")
        raise RuntimeError("stereo sensor unavailable")


class OracleProvidingRuntime(SequenceRuntime):
    def oracle_path_xy(self, task: BenchmarkEpisode) -> np.ndarray:
        assert task.episode_id == episode().episode_id
        return np.asarray(((0.0, 0.0), (0.5, 0.0), (1.0, 0.0)))

    def kinodynamic_plan(self, task: BenchmarkEpisode) -> KinodynamicPlanResult:
        assert task.episode_id == episode().episode_id
        return KinodynamicPlanResult(
            status="success",
            waypoints=(
                KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 1.0, 0.0),
                KinodynamicWaypoint(1.0, 0.0, 0.0, "forward", 0.0, 0.0),
            ),
            expanded_nodes=2,
            path_length_m=1.0,
            direction_changes=0,
        )


class SpyPolicy:
    policy_name = "spy"
    privileged = False

    def __init__(self, events: list[str], action: np.ndarray | None = None):
        self.events = events
        self.action = np.zeros(10, dtype=np.float32) if action is None else action
        self.goals: list[np.ndarray] = []
        self.context = None

    def reset(self, context: object) -> None:
        self.events.append("policy_reset")
        self.context = context

    def act(self, current: PolicyObservation, pointgoal_base_xy: np.ndarray) -> np.ndarray:
        self.events.append("act")
        self.goals.append(np.asarray(pointgoal_base_xy).copy())
        return self.action.copy()


class PrivilegedSpyPolicy(SpyPolicy):
    policy_name = "privileged_spy"
    privileged = True


class DiagnosedPolicy(SpyPolicy):
    def __init__(self, events: list[str]):
        super().__init__(events)
        self.act_count = 0

    @property
    def diagnostics(self) -> dict[str, object]:
        index = self.act_count
        return {
            "reference_index": 10 + index,
            "cross_track_error_m": 0.1 * index,
            "active_drive_direction": "forward" if index == 1 else "reverse",
            "desired_drive_direction": "reverse",
            "pending_drive_direction": None if index == 1 else "forward",
            "stopped_cycles": index,
            "braking_state": "tracking" if index == 1 else "direction_change",
        }

    def act(self, current: PolicyObservation, pointgoal_base_xy: np.ndarray) -> np.ndarray:
        self.act_count += 1
        return super().act(current, pointgoal_base_xy)


def _v2_diagnostic_payload(index: int = 1) -> dict[str, object]:
    value = float(index)
    return {
        "reference_index": index,
        "cross_track_error_m": 0.01 * value,
        "active_drive_direction": "forward",
        "desired_drive_direction": "forward",
        "pending_drive_direction": None,
        "stopped_cycles": 0,
        "braking_state": "tracking",
        "reference_curvature_1_m": 0.10 * value,
        "feedback_curvature_1_m": -0.01 * value,
        "command_curvature_1_m": 0.09 * value,
        "lateral_error_m": 0.02 * value,
        "heading_error_rad": -0.03 * value,
        "main_cap_rad_s": 0.50,
        "auxiliary_feedforward_rad": 0.04 * value,
        "auxiliary_feedback_rad": -0.005 * value,
        "projection_s_m": 0.20 * value,
        "delay_distance_m": 0.10,
        "query_s_m": 0.20 * value + 0.10,
        "pure_pursuit_curvature_1_m": 0.03 * value,
        "profile_curvature_1_m": 0.05 * value,
        "blend_weight": 0.5,
        "budget_infeasible": False,
        "terminal_runway_s": 20.0 - value,
        "terminal_brake_start_s": -1.0,
        "goal_entry_s": -1.0,
        "stop_below_0p05_s": -1.0,
        "hold_complete_s": -1.0,
        "elapsed_s": 0.1 * value,
    }


class V2DiagnosedPolicy(SpyPolicy):
    policy_name = "oracle_trajectory_v2"

    @property
    def diagnostics(self) -> dict[str, object]:
        return _v2_diagnostic_payload(len(self.goals))


def test_twenty_seconds_is_two_hundred_action_blocks():
    config = RunnerConfig(
        episode_budget_s=20.0,
        replan_hz=10,
        goal_tolerance_m=0.2,
    )

    assert config.max_action_blocks == 200


def test_default_budget_is_35_seconds_and_350_action_blocks():
    config = RunnerConfig()
    assert config.episode_budget_s == 35.0
    assert config.max_action_blocks == 350


def test_default_runner_executes_350_blocks_before_timeout():
    events = []
    runtime = SequenceRuntime([runtime_step((0.0, 0.0))], events)
    result = run_episode(
        runtime, SpyPolicy(events), episode(),
        action_protocol=protocol(), config=RunnerConfig(),
    )
    assert result.termination_reason == "timeout"
    assert result.action_blocks == len(runtime.executed) == 350
    assert result.elapsed_s == 35.0


def test_diagnostic_settling_preserves_nominal_success_and_waits_for_hold_window():
    """A moving first goal state must not be treated as a completed stop hold."""

    events: list[str] = []
    # The post-action state is in tolerance but still moving.  Five later
    # 10 Hz endpoints are slow and in tolerance, so the diagnostic settling
    # phase should stop exactly after those five zero-command blocks.
    steps = [
        runtime_step((0.0, 0.0), proprio_last=(0.0,) * 17),
        runtime_step((0.9, 0.0), proprio_last=(0.2, 0.0) + (0.0,) * 15),
    ]
    steps.extend(
        runtime_step((1.0, 0.0), proprio_last=(0.04, 0.0) + (0.0,) * 15)
        for _ in range(5)
    )
    runtime = SequenceRuntime(steps, events)
    policy = SpyPolicy(events)

    result = run_episode(
        runtime,
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=2.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert result.action_blocks == len(runtime.executed) == 6
    assert len(policy.goals) == 1
    np.testing.assert_allclose(result.actions[1:], 0.0)
    assert result.diagnostic_stop_hold_passed is True
    assert result.diagnostic_settling_blocks == 5
    assert result.diagnostic_goal_entry_action_block == 1


def test_diagnostic_settling_budget_is_default_off():
    config = RunnerConfig()
    assert config.diagnostic_settling_budget_s == 0.0
    assert config.diagnostic_stop_hold_s == 0.5
    assert config.diagnostic_stop_speed_m_s == 0.05


def test_diagnostic_settling_budget_cannot_exceed_fixed_three_second_window():
    with pytest.raises(ValueError, match="three seconds"):
        RunnerConfig(
            episode_budget_s=5.0,
            diagnostic_settling_budget_s=3.1,
        )


def test_diagnostic_settling_requires_five_post_entry_states_when_already_at_goal():
    events: list[str] = []
    at_goal = dataclasses.replace(
        episode(),
        start_position_world_m=(1.0, 0.0, 0.45),
        goal_position_world_m=(1.0, 0.0, 0.45),
    )
    runtime = SequenceRuntime(
        [runtime_step((1.0, 0.0), proprio_last=(0.0,) * 17)]
        + [runtime_step((1.0, 0.0), proprio_last=(0.0,) * 17) for _ in range(5)],
        events,
    )

    result = run_episode(
        runtime,
        SpyPolicy(events),
        at_goal,
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=1.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.diagnostic_stop_hold_passed is True
    assert result.diagnostic_settling_blocks == 5
    assert result.action_blocks == 5
    np.testing.assert_allclose(result.actions, 0.0)


def test_v2_initial_goal_entry_records_reset_low_speed_but_hold_starts_after_entry():
    """An already-settled reset still needs five post-entry endpoints."""

    events: list[str] = []
    at_goal = dataclasses.replace(
        episode(),
        start_position_world_m=(1.0, 0.0, 0.45),
        goal_position_world_m=(1.0, 0.0, 0.45),
    )
    runtime = SequenceRuntime(
        [
            runtime_step(
                (1.0, 0.0),
                proprio_last=(0.0,) * 17,
                simulation_time_s=0.0,
                physics_step_index=0,
            )
        ]
        + [
            runtime_step(
                (1.0, 0.0),
                proprio_last=(0.0,) * 17,
                simulation_time_s=0.1 * index,
                physics_step_index=10 * index,
            )
            for index in range(1, 6)
        ],
        events,
    )

    result = run_episode(
        runtime,
        V2DiagnosedPolicy(events),
        at_goal,
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=1.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    independent = tracking_acceptance_harness._independent_stop_hold(
        result,
        SimpleNamespace(goal_xy=np.asarray((1.0, 0.0), dtype=np.float64)),
        goal_tolerance_m=0.2,
        stop_speed_m_s=0.05,
        stop_hold_s=0.5,
        settling_budget_s=0.5,
        replan_hz=10,
    )
    assert result.diagnostic_goal_entry_time_s == pytest.approx(0.0)
    assert result.diagnostic_stop_below_speed_time_s == pytest.approx(0.0)
    assert result.diagnostic_terminal_brake_start_time_s is None
    assert result.diagnostic_hold_complete_time_s == pytest.approx(0.5)
    assert result.diagnostic_stop_hold_passed is True
    assert independent["goal_entry_state_index"] == 0
    assert independent["hold_state_indices"] == [1, 2, 3, 4, 5]
    assert independent["stop_hold_passed"] is True
    assert result.diagnostic_hold_complete_time_s == pytest.approx(
        result.simulation_time_s[5] - result.simulation_time_s[0]
    )


def test_v2_hold_starts_after_low_speed_goal_entry_endpoint():
    """The nominal goal endpoint is not the first independent hold sample."""

    events: list[str] = []
    # The first post-action endpoint is already in tolerance and below the
    # speed threshold.  The independent harness hold starts at the next
    # endpoint, so five more zero-action endpoints are required.
    steps = [
        runtime_step(
            (0.0, 0.0),
            proprio_last=(0.0,) * 17,
            simulation_time_s=0.0,
            physics_step_index=0,
        ),
        runtime_step(
            (1.0, 0.0),
            proprio_last=(0.04, 0.0) + (0.0,) * 15,
            simulation_time_s=0.1,
            physics_step_index=10,
        ),
    ]
    steps.extend(
        runtime_step(
            (1.0, 0.0),
            proprio_last=(0.04, 0.0) + (0.0,) * 15,
            simulation_time_s=0.1 * (index + 2),
            physics_step_index=10 * (index + 2),
        )
        for index in range(5)
    )
    runtime = SequenceRuntime(steps, events)

    result = run_episode(
        runtime,
        V2DiagnosedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=2.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.6,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert result.diagnostic_goal_entry_time_s == pytest.approx(0.1)
    assert result.diagnostic_stop_below_speed_time_s == pytest.approx(0.1)
    assert result.diagnostic_hold_complete_time_s == pytest.approx(0.6)
    assert result.action_blocks == len(runtime.executed) == 6
    independent = tracking_acceptance_harness._independent_stop_hold(
        result,
        SimpleNamespace(goal_xy=np.asarray((1.0, 0.0), dtype=np.float64)),
        goal_tolerance_m=0.2,
        stop_speed_m_s=0.05,
        stop_hold_s=0.5,
        settling_budget_s=0.6,
        replan_hz=10,
    )
    assert independent["stop_hold_passed"] is True
    assert independent["goal_entry_state_index"] == 1
    assert independent["hold_state_indices"] == [2, 3, 4, 5, 6]
    assert result.diagnostic_hold_complete_time_s == pytest.approx(
        result.simulation_time_s[6] - result.simulation_time_s[0]
    )


def test_world_goal_is_transformed_into_robot_base_frame():
    result = world_goal_to_base(
        goal_world_xy=np.asarray((1.0, 0.0)),
        position_world_xy=np.asarray((0.0, 0.0)),
        yaw_world_rad=math.pi / 2.0,
    )

    np.testing.assert_allclose(result, (0.0, -1.0), atol=1e-12)


def test_runner_uses_reset_act_execute_order_and_records_success_metrics():
    events: list[str] = []
    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((0.5, 0.0)), runtime_step((1.0, 0.0))],
        events,
    )
    policy = SpyPolicy(events)

    result = run_episode(
        runtime,
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert events == [
        "runtime_reset",
        "policy_reset",
        "act",
        "execute",
        "act",
        "execute",
    ]
    np.testing.assert_allclose(policy.goals, ((1.0, 0.0), (0.5, 0.0)))
    assert result.termination_reason == "success"
    assert result.action_blocks == 2
    assert result.path_length_m == pytest.approx(1.0)
    assert result.final_distance_m == pytest.approx(0.0)
    assert result.spl == pytest.approx(1.0)
    assert result.time_to_goal_s == pytest.approx(0.2)


def test_runner_records_state_and_policy_diagnostics_with_correct_time_alignment():
    events: list[str] = []
    runtime = SequenceRuntime(
        [
            runtime_step((0.0, 0.0), yaw=0.1, proprio_last=tuple(range(17))),
            runtime_step((0.5, 0.0), yaw=0.2, proprio_last=tuple(range(17, 34))),
            runtime_step((1.0, 0.0), yaw=0.3, proprio_last=tuple(range(34, 51))),
        ],
        events,
    )

    result = run_episode(
        runtime,
        DiagnosedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    # State arrays include reset state plus one state after each of two actions.
    np.testing.assert_allclose(result.yaw_world_rad, (0.1, 0.2, 0.3))
    np.testing.assert_allclose(
        result.linear_velocity_world_m_s,
        ((0, 1, 2), (17, 18, 19), (34, 35, 36)),
    )
    np.testing.assert_allclose(
        result.angular_velocity_world_rad_s,
        ((3, 4, 5), (20, 21, 22), (37, 38, 39)),
    )
    np.testing.assert_allclose(
        result.joint_position_rad,
        ((9, 10, 11), (26, 27, 28), (43, 44, 45)),
    )
    np.testing.assert_allclose(
        result.joint_velocity_rad_s,
        ((12, 13, 14), (29, 30, 31), (46, 47, 48)),
    )
    np.testing.assert_allclose(
        result.applied_action,
        ((15, 16), (32, 33), (49, 50)),
    )

    # Policy diagnostics are sampled after act(), so they align one-to-one with actions.
    np.testing.assert_array_equal(result.policy_reference_index, (11, 12))
    np.testing.assert_allclose(result.policy_cross_track_error_m, (0.1, 0.2))
    np.testing.assert_array_equal(result.policy_active_drive_direction, (1, -1))
    np.testing.assert_array_equal(result.policy_desired_drive_direction, (-1, -1))
    np.testing.assert_array_equal(result.policy_pending_drive_direction, (0, 1))
    np.testing.assert_array_equal(result.policy_stopped_cycles, (1, 2))
    np.testing.assert_array_equal(result.policy_braking_state_code, (1, 2))


def test_runner_records_round3_intervention_telemetry_and_zero_fills_legacy_policies():
    events: list[str] = []

    class Round3DiagnosedPolicy(SpyPolicy):
        @property
        def diagnostics(self) -> dict[str, object]:
            index = len(self.goals)
            return {
                "reference_progress_s_m": 0.1 * index,
                "reference_heading_rad": -0.2 * index,
                "reference_curvature_1_m": 0.3 * index,
                "desired_curvature_1_m": 0.4 * index,
                "baseline_main_command_rad_s": 1.0,
                "scheduled_main_command_rad_s": 0.5,
                "auxiliary_target_angle_rad": 0.25,
                "auxiliary_velocity_rad_s": 0.12,
                "scheduler_active": index == 1,
                "scheduler_trigger_code": 11 if index == 1 else 0,
            }

    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((0.5, 0.0)), runtime_step((1.0, 0.0))],
        events,
    )
    result = run_episode(
        runtime,
        Round3DiagnosedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )
    np.testing.assert_allclose(result.policy_reference_progress_s_m, (0.1, 0.2))
    np.testing.assert_allclose(result.policy_reference_heading_rad, (-0.2, -0.4))
    np.testing.assert_allclose(result.policy_scheduled_main_command_rad_s, (0.5, 0.5))
    np.testing.assert_array_equal(result.policy_scheduler_active, (True, False))
    np.testing.assert_array_equal(result.policy_scheduler_trigger_code, (11, 0))

    legacy = run_episode(
        SequenceRuntime(
            [runtime_step((0.0, 0.0)), runtime_step((0.5, 0.0)), runtime_step((1.0, 0.0))],
            [],
        ),
        SpyPolicy([]),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )
    np.testing.assert_allclose(legacy.policy_reference_progress_s_m, (0.0, 0.0))
    np.testing.assert_array_equal(legacy.policy_scheduler_active, (False, False))


def test_runner_transports_v1_policy_diagnostics_and_defaults_missing_fields():
    events: list[str] = []

    class V1DiagnosedPolicy(SpyPolicy):
        @property
        def diagnostics(self) -> dict[str, object]:
            index = len(self.goals)
            return {
                "reference_index": index,
                "cross_track_error_m": 0.01 * index,
                "active_drive_direction": "forward",
                "desired_drive_direction": "forward",
                "pending_drive_direction": None,
                "stopped_cycles": 0,
                "braking_state": "tracking",
                "reference_curvature_1_m": 0.10 * index,
                "feedback_curvature_1_m": -0.01 * index,
                "command_curvature_1_m": 0.09 * index,
                "lateral_error_m": 0.02 * index,
                "heading_error_rad": -0.03 * index,
                "main_cap_rad_s": 0.50,
                "auxiliary_feedforward_rad": 0.04 * index,
                "auxiliary_feedback_rad": -0.005 * index,
            }

    runtime = SequenceRuntime(
        [
            runtime_step((0.0, 0.0)),
            runtime_step((0.5, 0.0)),
            runtime_step((1.0, 0.0)),
        ],
        events,
    )
    result = run_episode(
        runtime,
        V1DiagnosedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    names = (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    )
    for name in names:
        values = getattr(result, name)
        assert values.shape == (result.action_blocks,)
        assert np.all(np.isfinite(values))
    np.testing.assert_allclose(result.reference_curvature_1_m, (0.1, 0.2))
    np.testing.assert_allclose(result.feedback_curvature_1_m, (-0.01, -0.02))
    np.testing.assert_allclose(result.command_curvature_1_m, (0.09, 0.18))
    np.testing.assert_allclose(result.main_cap_rad_s, (0.5, 0.5))

    missing = run_episode(
        runtime=SequenceRuntime(
            [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0))], []
        ),
        policy=SpyPolicy([]),
        episode=episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )
    for name in names:
        assert getattr(missing, name).shape == (missing.action_blocks,)
        assert np.all(getattr(missing, name) == 0.0)


def test_runner_transports_all_v2_action_aligned_diagnostics_without_zero_fill():
    events: list[str] = []
    runtime = SequenceRuntime(
        [
            runtime_step((0.0, 0.0)),
            runtime_step((0.5, 0.0)),
            runtime_step((1.0, 0.0)),
        ],
        events,
    )

    result = run_episode(
        runtime,
        V2DiagnosedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "success"
    assert result.action_blocks == 2
    expected_numeric = {
        "projection_s_m": (0.2, 0.4),
        "delay_distance_m": (0.1, 0.1),
        "query_s_m": (0.3, 0.5),
        "pure_pursuit_curvature_1_m": (0.03, 0.06),
        "profile_curvature_1_m": (0.05, 0.10),
        "blend_weight": (0.5, 0.5),
        "terminal_runway_s": (19.0, 18.0),
    }
    for name, expected in expected_numeric.items():
        values = getattr(result, name)
        assert values.shape == (result.action_blocks,)
        np.testing.assert_allclose(values, expected)
    assert result.budget_infeasible.dtype == np.dtype(bool)
    np.testing.assert_array_equal(result.budget_infeasible, (False, False))
    np.testing.assert_allclose(result.elapsed_s_action_aligned, (0.1, 0.2))
    for name in (
        "terminal_brake_start_s",
        "goal_entry_s",
        "stop_below_0p05_s",
        "hold_complete_s",
    ):
        assert getattr(result, name).shape == (result.action_blocks,)
        np.testing.assert_allclose(getattr(result, name), (-1.0, -1.0))


def test_runner_rejects_missing_v2_diagnostic_before_executing_action():
    events: list[str] = []

    class MissingV2DiagnosticPolicy(V2DiagnosedPolicy):
        @property
        def diagnostics(self) -> dict[str, object]:
            payload = _v2_diagnostic_payload(len(self.goals))
            payload.pop("projection_s_m")
            return payload

    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0))],
        events,
    )
    result = run_episode(
        runtime,
        MissingV2DiagnosticPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "runtime_error"
    assert result.runtime_error is not None
    assert "projection_s_m" in result.runtime_error
    assert result.action_blocks == 0
    assert runtime.executed == []


@pytest.mark.parametrize(
    "invalid_value",
    (None, "bad", np.nan, np.inf, 1.0 + 2.0j),
    ids=("none", "text", "nan", "inf", "complex"),
)
def test_runner_rejects_present_invalid_v1_diagnostics(invalid_value):
    events: list[str] = []

    class MalformedPolicy(SpyPolicy):
        @property
        def diagnostics(self) -> dict[str, object]:
            return {"reference_curvature_1_m": invalid_value}

    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0))], events
    )
    result = run_episode(
        runtime,
        MalformedPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "runtime_error"
    assert result.runtime_error is not None
    assert "reference_curvature_1_m" in result.runtime_error
    assert result.action_blocks == 0
    assert result.actions.shape == (0, 10)
    assert result.reference_curvature_1_m.size == 0
    assert result.to_metrics_dict()["reference_curvature_1_m"] == []
    assert runtime.executed == []


def test_settling_repeats_latest_v1_diagnostics_only_on_settling_actions():
    events: list[str] = []
    names = (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    )

    class SettlingDiagnosedPolicy(SpyPolicy):
        def __init__(self, events: list[str]):
            super().__init__(events)
            self.act_count = 0

        @property
        def diagnostics(self) -> dict[str, object]:
            return {
                name: 100.0 * index + float(self.act_count)
                for index, name in enumerate(names, start=1)
            }

        def act(
            self, current: PolicyObservation, pointgoal_base_xy: np.ndarray
        ) -> np.ndarray:
            self.act_count += 1
            return super().act(current, pointgoal_base_xy)

    steps = [
        runtime_step((0.0, 0.0), proprio_last=(0.0,) * 17),
        runtime_step((0.7, 0.0), proprio_last=(0.2, 0.0) + (0.0,) * 15),
        runtime_step((0.9, 0.0), proprio_last=(0.2, 0.0) + (0.0,) * 15),
    ]
    steps.extend(
        runtime_step((1.0, 0.0), proprio_last=(0.04, 0.0) + (0.0,) * 15)
        for _ in range(5)
    )
    policy = SettlingDiagnosedPolicy(events)
    result = run_episode(
        SequenceRuntime(steps, events),
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=2.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert policy.act_count == 2
    np.testing.assert_array_equal(
        result.diagnostic_settling_mask,
        (False, False, True, True, True, True, True),
    )
    np.testing.assert_allclose(result.actions[result.diagnostic_settling_mask], 0.0)
    for index, name in enumerate(names, start=1):
        np.testing.assert_allclose(
            getattr(result, name),
            (100.0 * index + 1.0, 100.0 * index + 2.0, *([100.0 * index + 2.0] * 5)),
        )


def test_settling_reuses_cached_diagnostics_when_property_changes_without_act():
    events: list[str] = []
    names = (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    )

    class ReadChangingPolicy(SpyPolicy):
        def __init__(self, events: list[str]):
            super().__init__(events)
            self.diagnostic_reads = 0

        @property
        def diagnostics(self) -> dict[str, object]:
            self.diagnostic_reads += 1
            return {
                name: float(self.diagnostic_reads)
                for name in names
            }

    steps = [
        runtime_step((0.0, 0.0), proprio_last=(0.0,) * 17),
        runtime_step((0.9, 0.0), proprio_last=(0.2, 0.0) + (0.0,) * 15),
    ]
    steps.extend(
        runtime_step((1.0, 0.0), proprio_last=(0.04, 0.0) + (0.0,) * 15)
        for _ in range(5)
    )
    policy = ReadChangingPolicy(events)
    result = run_episode(
        SequenceRuntime(steps, events),
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=2.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert policy.diagnostic_reads == 1
    np.testing.assert_array_equal(
        result.diagnostic_settling_mask,
        (False, True, True, True, True, True),
    )
    for name in names:
        np.testing.assert_allclose(getattr(result, name), np.ones(6))


def test_already_at_goal_caches_one_post_reset_diagnostic_for_settling():
    events: list[str] = []
    names = (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    )
    at_goal = dataclasses.replace(
        episode(),
        start_position_world_m=(1.0, 0.0, 0.45),
        goal_position_world_m=(1.0, 0.0, 0.45),
    )

    class PostResetChangingPolicy(SpyPolicy):
        def __init__(self, events: list[str]):
            super().__init__(events)
            self.diagnostic_reads = 0

        @property
        def diagnostics(self) -> dict[str, object]:
            self.diagnostic_reads += 1
            return {name: float(self.diagnostic_reads) for name in names}

    runtime = SequenceRuntime(
        [runtime_step((1.0, 0.0), proprio_last=(0.0,) * 17)]
        + [runtime_step((1.0, 0.0), proprio_last=(0.0,) * 17) for _ in range(5)],
        events,
    )
    policy = PostResetChangingPolicy(events)
    result = run_episode(
        runtime,
        policy,
        at_goal,
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=1.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.5,
            diagnostic_stop_hold_s=0.5,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert policy.diagnostic_reads == 1
    np.testing.assert_array_equal(
        result.diagnostic_settling_mask,
        (True, True, True, True, True),
    )
    for name in names:
        np.testing.assert_allclose(getattr(result, name), np.ones(5))


def test_runner_owns_relative_terminal_event_times_through_settling():
    events: list[str] = []
    runtime = SequenceRuntime(
        [
            runtime_step(
                (0.0, 0.0),
                proprio_last=(0.0,) * 17,
                simulation_time_s=10.0,
                physics_step_index=100,
            ),
            runtime_step(
                (0.6, 0.0),
                proprio_last=(0.2, 0.0) + (0.0,) * 15,
                simulation_time_s=10.2,
                physics_step_index=110,
            ),
            runtime_step(
                (0.9, 0.0),
                proprio_last=(0.10, 0.0) + (0.0,) * 15,
                simulation_time_s=10.45,
                physics_step_index=122,
            ),
            runtime_step(
                (1.0, 0.0),
                proprio_last=(0.04, 0.0) + (0.0,) * 15,
                simulation_time_s=10.70,
                physics_step_index=135,
            ),
            runtime_step(
                (1.0, 0.0),
                proprio_last=(0.03, 0.0) + (0.0,) * 15,
                simulation_time_s=10.90,
                physics_step_index=145,
            ),
        ],
        events,
    )

    class BrakingV2Policy(V2DiagnosedPolicy):
        @property
        def diagnostics(self) -> dict[str, object]:
            payload = _v2_diagnostic_payload(len(self.goals))
            if len(self.goals) == 1:
                payload.update(
                    braking_state="goal_braking",
                    terminal_brake_start_s=0.0,
                )
            return payload

    result = run_episode(
        runtime,
        BrakingV2Policy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(
            episode_budget_s=2.0,
            replan_hz=10,
            goal_tolerance_m=0.2,
            diagnostic_settling_budget_s=0.3,
            diagnostic_stop_hold_s=0.2,
            diagnostic_stop_speed_m_s=0.05,
        ),
    )

    assert result.termination_reason == "success"
    assert result.action_blocks == 4
    assert result.diagnostic_terminal_brake_start_time_s == pytest.approx(0.0)
    assert result.diagnostic_goal_entry_time_s == pytest.approx(0.45)
    assert result.diagnostic_stop_below_speed_time_s == pytest.approx(0.70)
    assert result.diagnostic_hold_complete_time_s == pytest.approx(0.90)
    assert result.diagnostic_terminal_brake_start_time_s <= result.diagnostic_goal_entry_time_s
    assert result.diagnostic_goal_entry_time_s < result.diagnostic_stop_below_speed_time_s
    assert result.diagnostic_stop_below_speed_time_s < result.diagnostic_hold_complete_time_s


def test_runner_collects_state_aligned_clock_and_roll_from_every_runtime_step():
    events: list[str] = []
    runtime = SequenceRuntime(
        [
            runtime_step(
                (0.0, 0.0),
                simulation_time_s=0.0,
                physics_step_index=0,
                raw_simulation_time_s=8.0,
                raw_physics_step_index=200,
                roll_world_rad=0.01,
            ),
            runtime_step(
                (0.5, 0.0),
                simulation_time_s=0.1,
                physics_step_index=5,
                raw_simulation_time_s=8.1,
                raw_physics_step_index=205,
                roll_world_rad=0.02,
            ),
            runtime_step(
                (1.0, 0.0),
                simulation_time_s=0.2,
                physics_step_index=10,
                raw_simulation_time_s=8.2,
                raw_physics_step_index=210,
                roll_world_rad=0.03,
            ),
        ],
        events,
    )

    result = run_episode(
        runtime,
        SpyPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.positions_world_xy.shape[0] == result.actions.shape[0] + 1
    np.testing.assert_allclose(result.simulation_time_s, (0.0, 0.1, 0.2))
    np.testing.assert_array_equal(result.physics_step_index, (0, 5, 10))
    np.testing.assert_allclose(result.raw_simulation_time_s, (8.0, 8.1, 8.2))
    np.testing.assert_array_equal(result.raw_physics_step_index, (200, 205, 210))
    np.testing.assert_allclose(result.roll_world_rad, (0.01, 0.02, 0.03))


def test_runner_marks_legacy_missing_clock_and_roll_as_unavailable():
    events: list[str] = []
    runtime = SequenceRuntime([runtime_step((0.0, 0.0))], events)

    result = run_episode(
        runtime,
        SpyPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=0.1, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.simulation_time_s.shape == (2,)
    assert result.physics_step_index.shape == (2,)
    assert result.raw_simulation_time_s.shape == (2,)
    assert result.raw_physics_step_index.shape == (2,)
    assert result.roll_world_rad.shape == (2,)
    assert np.isnan(result.simulation_time_s).all()
    assert np.isnan(result.physics_step_index).all()
    assert np.isnan(result.raw_simulation_time_s).all()
    assert np.isnan(result.raw_physics_step_index).all()
    assert np.isnan(result.roll_world_rad).all()


def test_collision_wins_when_goal_is_reached_on_same_step():
    events: list[str] = []
    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0), collision=True, clearance_m=-0.01)],
        events,
    )

    result = run_episode(
        runtime,
        SpyPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=2.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "collision"
    assert result.spl == 0.0
    assert result.minimum_clearance_m == pytest.approx(-0.01)


def test_runner_times_out_after_exact_configured_budget():
    events: list[str] = []
    runtime = SequenceRuntime([runtime_step((0.0, 0.0))], events)

    result = run_episode(
        runtime,
        SpyPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=0.2, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "timeout"
    assert result.action_blocks == 2
    assert len(runtime.executed) == 2


def test_invalid_policy_action_is_preserved_as_runtime_error_without_execution():
    events: list[str] = []
    runtime = SequenceRuntime([runtime_step((0.0, 0.0))], events)
    policy = SpyPolicy(events, action=np.full(10, 99.0, dtype=np.float32))

    result = run_episode(
        runtime,
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "runtime_error"
    assert "outside" in result.runtime_error
    assert runtime.executed == []


def test_runtime_failure_remains_an_explicit_episode_result():
    events: list[str] = []
    runtime = FailingRuntime([runtime_step((0.0, 0.0))], events)

    result = run_episode(
        runtime,
        SpyPolicy(events),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "runtime_error"
    assert "stereo sensor unavailable" in result.runtime_error
    assert result.to_metrics_dict()["runtime_error"] == 1.0


def test_privileged_policy_gets_runtime_oracle_after_reset():
    events: list[str] = []
    runtime = OracleProvidingRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0))],
        events,
    )
    policy = PrivilegedSpyPolicy(events)

    result = run_episode(
        runtime,
        policy,
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
    )

    assert result.termination_reason == "success"
    np.testing.assert_allclose(
        policy.context.oracle_path_xy,
        ((0.0, 0.0), (0.5, 0.0), (1.0, 0.0)),
    )
    assert policy.context.kinodynamic_reference == runtime.kinodynamic_plan(
        episode()
    ).waypoints


def test_runner_injects_dynamic_plan_without_v1_oracle_providers_and_aligns_diagnostics():
    events: list[str] = []
    runtime = SequenceRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((0.4, 0.0)), runtime_step((1.0, 0.0))],
        events,
    )
    plan = DynamicPlanResult(
        status="success",
        waypoints=(
            DynamicWaypoint(0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward"),
            DynamicWaypoint(0.4, 0.0, 0.0, 0.5, 0.0, 0.2, 0.1, "forward"),
            DynamicWaypoint(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, "stopped"),
        ),
        duration_s=0.2,
    )

    result = run_episode(
        runtime,
        benchmark_policies.DynamicOraclePolicy(protocol()),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
        dynamic_plan=plan,
    )

    assert result.termination_reason == "success"
    assert result.positions_world_xy.shape[0] == result.actions.shape[0] + 1
    assert result.dynamic_reference_index.shape == (result.action_blocks,)
    assert result.dynamic_position_error_m.shape == (result.action_blocks,)
    assert result.dynamic_yaw_error_rad.shape == (result.action_blocks,)
    assert result.dynamic_auxiliary_error_rad.shape == (result.action_blocks,)


def test_runner_rejects_dynamic_plan_injection_for_v1_oracle():
    events: list[str] = []
    runtime = OracleProvidingRuntime(
        [runtime_step((0.0, 0.0)), runtime_step((1.0, 0.0))], events
    )
    plan = DynamicPlanResult(
        status="success",
        waypoints=(
            DynamicWaypoint(0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward"),
            DynamicWaypoint(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, "stopped"),
        ),
        duration_s=0.2,
    )

    result = run_episode(
        runtime,
        benchmark_policies.OraclePathPolicy(protocol()),
        episode(),
        action_protocol=protocol(),
        config=RunnerConfig(episode_budget_s=1.0, replan_hz=10, goal_tolerance_m=0.2),
        dynamic_plan=plan,
    )

    assert result.termination_reason == "runtime_error"
    assert "dynamic_plan" in result.runtime_error
    assert "DynamicOraclePolicy" in result.runtime_error
    assert runtime.executed == []


def test_cli_policy_names_keep_v1_oracle_and_add_distinct_dynamic_oracle():
    from scripts.run_pointgoal_benchmark import _build_parser, _make_policy

    assert isinstance(
        _make_policy("oracle", protocol(), seed=3072),
        benchmark_policies.OraclePathPolicy,
    )
    assert isinstance(
        _make_policy("oracle_v2", protocol(), seed=3072),
        benchmark_policies.DynamicOraclePolicy,
    )
    with pytest.raises(ValueError, match="unsupported policy"):
        _make_policy("oracle_v3", protocol(), seed=3072)
    policy_action = next(
        action for action in _build_parser()._actions if action.dest == "policy"
    )
    assert "oracle_v2" in policy_action.choices


def test_formal_cli_oracle_lookahead_defaults_and_rejects_nonfinite_values():
    from scripts.run_pointgoal_benchmark import _build_parser

    parser = _build_parser()
    required = [
        "--manifest", "manifest.json", "--split", "dev", "--policy", "oracle",
        "--scene-root", "scenes", "--run-root", "runs", "--headless",
        "--record-video",
    ]
    default = parser.parse_args(required)
    assert default.oracle_lookahead_m == pytest.approx(0.5)

    selected = parser.parse_args(required + ["--oracle-lookahead-m", "1.0"])
    assert selected.oracle_lookahead_m == pytest.approx(1.0)

    for value in ("0", "nan", "inf", "-1"):
        with pytest.raises(SystemExit):
            parser.parse_args(required + ["--oracle-lookahead-m", value])


def test_formal_cli_auxiliary_velocity_limit_is_opt_in():
    from scripts.run_pointgoal_benchmark import _build_parser

    parser = _build_parser()
    required = [
        "--manifest", "manifest.json", "--split", "golden", "--policy", "oracle",
        "--scene-root", "scenes", "--run-root", "runs", "--headless",
        "--record-video",
    ]
    default = parser.parse_args(required)
    assert default.auxiliary_velocity_limit_rad_s is None

    selected = parser.parse_args(
        required + ["--auxiliary-velocity-limit-rad-s", "0.30"]
    )
    assert selected.auxiliary_velocity_limit_rad_s == pytest.approx(0.30)


def test_formal_cli_accepts_explicit_drive_profile():
    from scripts.run_pointgoal_benchmark import _build_parser

    parser = _build_parser()
    required = [
        "--manifest", "manifest.json", "--split", "golden", "--policy", "oracle",
        "--scene-root", "scenes", "--run-root", "runs", "--headless",
        "--record-video",
    ]

    selected = parser.parse_args(required + ["--drive-profile", "drive_dynamics_v2.json"])

    assert selected.drive_profile == Path("drive_dynamics_v2.json")


def test_formal_oracle_lookahead_reaches_only_v1_oracle_and_rejects_other_policies():
    from scripts.run_pointgoal_benchmark import _make_policy

    policy = _make_policy(
        "oracle", protocol(), seed=3072, oracle_lookahead_m=1.0
    )
    assert isinstance(policy, benchmark_policies.OraclePathPolicy)
    assert policy.lookahead_m == pytest.approx(1.0)

    with pytest.raises(ValueError, match="only supported with --policy oracle"):
        _make_policy(
            "goal_greedy", protocol(), seed=3072, oracle_lookahead_m=1.0
        )


def test_formal_child_command_propagates_oracle_lookahead_to_worker():
    from scripts.run_pointgoal_benchmark import _child_command

    args = SimpleNamespace(
        manifest=Path("manifest.json"),
        split="dev",
        policy="oracle",
        scene_root=Path("scenes"),
        run_root=Path("runs"),
        gui=False,
        seed=3072,
        contract=Path("contract.yaml"),
        scene_index=Path("scene_index.json"),
        data_root=None,
        oracle_lookahead_m=1.0,
    )

    command = _child_command(args, "runner_e0_0001")

    assert command[command.index("--oracle-lookahead-m") + 1] == "1.0"


def test_formal_child_command_propagates_auxiliary_velocity_limit_to_worker():
    from scripts.run_pointgoal_benchmark import _child_command

    args = SimpleNamespace(
        manifest=Path("manifest.json"),
        split="golden",
        policy="oracle",
        scene_root=Path("scenes"),
        run_root=Path("runs"),
        gui=False,
        seed=3072,
        contract=Path("contract.yaml"),
        scene_index=Path("scene_index.json"),
        data_root=None,
        oracle_lookahead_m=0.5,
        auxiliary_velocity_limit_rad_s=0.30,
    )

    command = _child_command(args, "golden_e0_0001")

    assert command[command.index("--auxiliary-velocity-limit-rad-s") + 1] == "0.3"


def test_formal_child_command_propagates_drive_profile_to_worker():
    from scripts.run_pointgoal_benchmark import _child_command

    args = SimpleNamespace(
        manifest=Path("manifest.json"),
        split="golden",
        policy="oracle",
        scene_root=Path("scenes"),
        run_root=Path("runs"),
        gui=False,
        seed=3072,
        contract=Path("contract.yaml"),
        scene_index=Path("scene_index.json"),
        data_root=None,
        oracle_lookahead_m=0.5,
        auxiliary_velocity_limit_rad_s=0.30,
        drive_profile=Path("drive_dynamics_v2.json"),
    )

    command = _child_command(args, "golden_e0_0001")

    assert command[command.index("--drive-profile") + 1] == "drive_dynamics_v2.json"


def test_formal_environment_and_summary_record_oracle_lookahead(tmp_path: Path, monkeypatch):
    from scripts import run_pointgoal_benchmark as entrypoint

    args = SimpleNamespace(
        manifest=tmp_path / "manifest.json",
        policy="oracle",
        seed=3072,
        gui=False,
        oracle_lookahead_m=1.0,
    )
    runtime = SimpleNamespace(contact_measurement_available=True)
    policy = benchmark_policies.OraclePathPolicy(protocol())
    monkeypatch.setattr(
        entrypoint,
        "load_manifest",
        lambda _path: SimpleNamespace(
            benchmark_version="v1", manifest_sha256="ab" * 32
        ),
    )
    monkeypatch.setattr(entrypoint, "_command_text", lambda _command: None)

    environment = entrypoint._environment_payload(
        args, episode(), protocol(), policy, runtime
    )

    assert environment["oracle_lookahead_m"] == pytest.approx(1.0)

    episode_dir = tmp_path / "episode_runner_e0_0001"
    episode_dir.mkdir()
    (episode_dir / "metrics.json").write_text(
        json.dumps(
            {
                "episode_budget_s": 35.0,
                "evaluation_protocol_id": "ball_pointgoal_v1_1_35s",
                "success": 0.0,
                "collision": 0.0,
                "timeout": 1.0,
                "runtime_error": 0.0,
                "spl": 0.0,
                "oracle_lookahead_m": 1.0,
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        entrypoint,
        "summarize_navigation",
        lambda metrics, *, goal_tolerance_m, max_mpc_steps: {
            "num_eval": len(metrics)
        },
    )

    summary_path = entrypoint._write_summary(
        tmp_path, (episode(),), oracle_lookahead_m=1.0
    )

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["oracle_lookahead_m"] == pytest.approx(1.0)


def test_dynamic_environment_records_policy_and_frozen_model_parameters(
    monkeypatch: pytest.MonkeyPatch, tmp_path,
):
    from scripts import run_pointgoal_benchmark as entrypoint

    drive_profile = tmp_path / "drive.json"
    drive_profile.write_text("{}", encoding="utf-8")
    args = type(
        "Args",
        (),
        {
            "manifest": tmp_path / "manifest.json",
            "policy": "oracle_v2",
            "seed": 3072,
            "gui": False,
        },
    )()
    runtime = type(
        "Runtime",
        (),
        {
            "contact_measurement_available": True,
            "config": type("Config", (), {"drive_profile_path": drive_profile})(),
        },
    )()
    monkeypatch.setattr(
        entrypoint,
        "load_manifest",
        lambda _path: type(
            "Manifest",
            (),
            {"benchmark_version": "v1", "manifest_sha256": "ab" * 32},
        )(),
    )
    monkeypatch.setattr(entrypoint, "_command_text", lambda _command: None)

    payload = entrypoint._environment_payload(
        args,
        episode(),
        protocol(),
        benchmark_policies.DynamicOraclePolicy(protocol()),
        runtime,
    )

    assert payload["policy"] == "oracle_v2"
    assert payload["dynamic_planner"] == {
        "planner_version": "dynamic_lattice_v2",
        "drive_profile_path": str(drive_profile),
        "primitive_duration_s": 1.0,
        "integration_dt_s": 0.02,
        "main_acceleration_limit_rad_s2": 0.5,
        "auxiliary_rate_limit_rad_s": pytest.approx(math.radians(30.0)),
        "maximum_predicted_duration_s": 32.0,
        "node_budget": 500_000,
        "planning_timeout_s": 60.0,
    }
