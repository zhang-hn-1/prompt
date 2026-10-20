"""Contract and behavior tests for Benchmark V1 policy adapters."""

from __future__ import annotations

import math
from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pytest

from benchmark import policies as benchmark_policies
from benchmark.drive_model import DriveModel
from benchmark.policies import (
    EpisodeContext,
    GoalGreedyPolicy,
    OraclePathPolicy,
    PolicyObservation,
    RandomPolicy,
    StopPolicy,
    DelayAwareTrajectoryOraclePolicy,
    TrajectoryOraclePolicy,
    adaptive_lookahead_distance,
    calibrated_auxiliary_angle,
    calibrated_stopping_distance,
    curvature_tracker_action,
)
from benchmark.dynamic_lattice_planning import DynamicWaypoint
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.trajectory_tracking import SpatialReferenceProfile
from training.action_protocol import ActionProtocol


def aux30_protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def valid_observation(
    *,
    position_world_xy: tuple[float, float] = (0.0, 0.0),
    yaw_world_rad: float = 0.0,
    forward_speed_m_s: float = 0.0,
    yaw_rate_rad_s: float = 0.0,
    auxiliary_angle_rad: float = 0.0,
) -> PolicyObservation:
    proprio = np.zeros((5, 17), dtype=np.float32)
    proprio[-1, 0] = forward_speed_m_s
    proprio[-1, 5] = yaw_rate_rad_s
    proprio[-1, 10] = auxiliary_angle_rad
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray(position_world_xy, dtype=np.float64),
        yaw_world_rad=yaw_world_rad,
    )


def non_privileged_context() -> EpisodeContext:
    return EpisodeContext(
        episode_id="golden_e0_0001",
        privileged=False,
        oracle_path_xy=None,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
        dynamic_reference=None,
    )


def dynamic_reference() -> tuple[DynamicWaypoint, ...]:
    return (
        DynamicWaypoint(0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward"),
        DynamicWaypoint(
            0.05, 0.0, 0.0, 1.0, math.radians(10.0), 0.4, 0.1, "forward"
        ),
        DynamicWaypoint(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, "stopped"),
    )


def dynamic_context(
    reference: tuple[DynamicWaypoint, ...] | None = None,
) -> EpisodeContext:
    return EpisodeContext(
        episode_id="dynamic-e1",
        privileged=True,
        oracle_path_xy=None,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
        dynamic_reference=dynamic_reference() if reference is None else reference,
    )


def kinodynamic_reference() -> tuple[KinodynamicWaypoint, ...]:
    return (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 1.0, 0.0),
        KinodynamicWaypoint(0.5, 0.0, 0.0, "forward", 1.0, 0.0),
        KinodynamicWaypoint(1.0, 0.3, 0.3, "forward", 0.0, 0.0),
    )


def bidirectional_reference() -> tuple[KinodynamicWaypoint, ...]:
    return (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.3, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.6, 0.0, 0.0, "reverse", -0.5, 0.0),
        KinodynamicWaypoint(0.3, -0.1, 0.2, "reverse", 0.0, 0.0),
    )


def straight_oracle_context(direction: str) -> EpisodeContext:
    main_command = 1.5 if direction == "forward" else -1.5
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, direction, main_command, 0.0),
        KinodynamicWaypoint(2.0, 0.0, 0.0, direction, main_command, 0.0),
    )
    return EpisodeContext(
        episode_id=f"straight-{direction}",
        privileged=True,
        oracle_path_xy=np.asarray(((0.0, 0.0), (2.0, 0.0)), dtype=np.float64),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference,
    )


def trajectory_line_context(
    *,
    direction: str = "forward",
    length_m: float = 4.0,
    command: float | None = None,
    yaw_rad: float = 0.0,
    include_oracle_path: bool = True,
) -> EpisodeContext:
    sign = 1.0 if direction == "forward" else -1.0
    main = sign * 1.5 if command is None else float(command)
    points = tuple(
        KinodynamicWaypoint(
            sign * float(index),
            0.0,
            yaw_rad,
            direction,
            main if index < int(length_m) else 0.0,
            0.0,
        )
        for index in range(int(length_m) + 1)
    )
    path = (
        np.asarray([(point.x_m, point.y_m) for point in points], dtype=np.float64)
        if include_oracle_path
        else None
    )
    return EpisodeContext(
        episode_id=f"trajectory-line-{direction}",
        privileged=True,
        oracle_path_xy=path,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=points,
    )


def _constant_curvature_reference(
    curvature_1_m: float,
    *,
    direction: str = "forward",
    point_count: int = 5,
    spacing_m: float = 1.0,
) -> tuple[KinodynamicWaypoint, ...]:
    """Create a literal constant-curvature spatial reference."""

    sign = 1.0 if direction == "forward" else -1.0
    yaw = 0.0
    x = 0.0
    y = 0.0
    result: list[KinodynamicWaypoint] = []
    for index in range(point_count):
        result.append(
            KinodynamicWaypoint(
                x,
                y,
                yaw,
                direction,
                sign * 1.5 if index < point_count - 1 else 0.0,
                0.0,
            )
        )
        if index + 1 == point_count:
            break
        step = float(spacing_m)
        if abs(curvature_1_m) <= 1e-12:
            x += step * math.cos(yaw)
            y += step * math.sin(yaw)
        else:
            next_yaw = yaw + curvature_1_m * step
            x += (math.sin(next_yaw) - math.sin(yaw)) / curvature_1_m
            y += (-math.cos(next_yaw) + math.cos(yaw)) / curvature_1_m
            yaw = next_yaw
    return tuple(result)


def trajectory_context_from_reference(
    reference: tuple[KinodynamicWaypoint, ...],
    *,
    episode_id: str = "trajectory-reference",
    include_oracle_path: bool = True,
) -> EpisodeContext:
    path = (
        np.asarray([(point.x_m, point.y_m) for point in reference], dtype=np.float64)
        if include_oracle_path
        else None
    )
    return EpisodeContext(
        episode_id=episode_id,
        privileged=True,
        oracle_path_xy=path,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference,
    )


def _reverse_quarter_circle_reference() -> tuple[KinodynamicWaypoint, ...]:
    # Temporal reverse motion: position and body yaw both evolve opposite to
    # the forward left-turn sample, so profile curvature is -1/3 m^-1.
    samples = (
        (0.000000, 0.000000, 0.000000),
        (-0.776457, 0.102809, -0.261799),
        (-1.500000, 0.401924, -0.523599),
        (-2.121320, 0.878680, -0.785398),
        (-2.598076, 1.500000, -1.047198),
        (-2.897777, 2.223543, -1.308997),
        (-3.000000, 3.000000, -1.570796),
    )
    return tuple(
        KinodynamicWaypoint(
            x,
            y,
            yaw,
            "reverse",
            -1.5 if index < len(samples) - 1 else 0.0,
            0.0,
        )
        for index, (x, y, yaw) in enumerate(samples)
    )


def reference_from_xy(path: np.ndarray) -> tuple[KinodynamicWaypoint, ...]:
    return tuple(
        KinodynamicWaypoint(
            float(point[0]),
            float(point[1]),
            0.0,
            "forward",
            0.0 if index == len(path) - 1 else 1.5,
            0.0,
        )
        for index, point in enumerate(path)
    )


def privileged_context() -> EpisodeContext:
    return EpisodeContext(
        episode_id="golden_e1_0001",
        privileged=True,
        oracle_path_xy=np.asarray(
            ((0.0, 0.0), (0.5, 0.0), (1.0, 0.3), (1.5, 0.8), (2.0, 1.0)),
            dtype=np.float64,
        ),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=kinodynamic_reference(),
    )


def test_formal_policy_observation_does_not_expose_privileged_reference():
    assert {field.name for field in fields(PolicyObservation)} == {
        "stereo_left_rgb",
        "stereo_right_rgb",
        "proprio_history",
        "action_history",
        "position_world_xy",
        "yaw_world_rad",
    }


def test_non_privileged_context_rejects_kinodynamic_reference():
    with pytest.raises(PermissionError, match="non-privileged"):
        EpisodeContext(
            episode_id="leak",
            privileged=False,
            oracle_path_xy=None,
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=kinodynamic_reference(),
        )


def test_non_privileged_context_rejects_dynamic_reference():
    with pytest.raises(PermissionError, match="non-privileged"):
        EpisodeContext(
            episode_id="dynamic-leak",
            privileged=False,
            oracle_path_xy=None,
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=None,
            dynamic_reference=dynamic_reference(),
        )


def test_dynamic_oracle_permissions_expose_only_timed_reference_privilege():
    permissions = benchmark_policies.DynamicOraclePolicy(
        aux30_protocol()
    ).online_input_permissions

    assert "dynamic_reference" in permissions
    assert "kinodynamic_reference" not in permissions
    assert "oracle_path_xy" not in permissions
    assert "occupancy" not in permissions
    assert "truth_clearance" not in permissions
    assert "collision" not in permissions


def test_v1_and_v2_oracles_require_their_own_reference_types():
    dynamic = benchmark_policies.DynamicOraclePolicy(aux30_protocol())
    with pytest.raises(ValueError, match="dynamic_reference"):
        dynamic.reset(privileged_context())

    v1 = OraclePathPolicy(aux30_protocol())
    missing_v1_reference = EpisodeContext(
        episode_id="missing-v1-reference",
        privileged=True,
        oracle_path_xy=np.asarray(((0.0, 0.0), (1.0, 0.0))),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
        dynamic_reference=None,
    )
    with pytest.raises(ValueError, match="kinodynamic_reference"):
        v1.reset(missing_v1_reference)


def test_v1_oracle_rejects_context_containing_both_reference_types():
    mixed = EpisodeContext(
        episode_id="mixed-references",
        privileged=True,
        oracle_path_xy=np.asarray(((0.0, 0.0), (1.0, 0.0))),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=kinodynamic_reference(),
        dynamic_reference=dynamic_reference(),
    )

    with pytest.raises(ValueError, match="dynamic_reference"):
        OraclePathPolicy(aux30_protocol()).reset(mixed)


def test_oracle_yaw_rate_feedback_gain_adds_signed_correction_to_real_feedforward():
    drive_model = DriveModel.from_json(
        Path(__file__).resolve().parents[1]
        / "configs"
        / "benchmark_v1"
        / "drive_dynamics_v1.json"
    )

    for direction, expected_correction in (("forward", 0.03), ("reverse", -0.03)):
        context = straight_oracle_context(direction)
        feedforward_only = OraclePathPolicy(
            aux30_protocol(), yaw_rate_feedback_gain_s=0.0, drive_model=drive_model
        )
        default_gain = OraclePathPolicy(
            aux30_protocol(), drive_model=drive_model
        )
        feedforward_only.reset(context)
        default_gain.reset(context)

        feedforward_action = feedforward_only.act(
            valid_observation(yaw_rate_rad_s=0.2), pointgoal_base_xy=(2.0, 0.0)
        )
        default_action = default_gain.act(
            valid_observation(yaw_rate_rad_s=0.2), pointgoal_base_xy=(2.0, 0.0)
        )

        assert default_gain.yaw_rate_feedback_gain_s == pytest.approx(0.15)
        assert float(default_action[1] - feedforward_action[1]) == pytest.approx(
            expected_correction, abs=1e-6
        )


def test_oracle_zero_yaw_rate_feedback_gain_removes_yaw_rate_dependence():
    policy = OraclePathPolicy(
        aux30_protocol(),
        yaw_rate_feedback_gain_s=0.0,
        drive_model=DriveModel.from_json(
            Path(__file__).resolve().parents[1]
            / "configs"
            / "benchmark_v1"
            / "drive_dynamics_v1.json"
        ),
    )
    policy.reset(straight_oracle_context("forward"))

    no_yaw_rate = policy.act(
        valid_observation(yaw_rate_rad_s=0.0), pointgoal_base_xy=(2.0, 0.0)
    )
    measured_yaw_rate = policy.act(
        valid_observation(yaw_rate_rad_s=0.2), pointgoal_base_xy=(2.0, 0.0)
    )

    np.testing.assert_array_equal(no_yaw_rate, measured_yaw_rate)


def test_oracle_yaw_rate_feedback_gain_preserves_auxiliary_clamp():
    drive_model = DriveModel.from_json(
        Path(__file__).resolve().parents[1]
        / "configs"
        / "benchmark_v1"
        / "drive_dynamics_v1.json"
    )
    policy = OraclePathPolicy(
        aux30_protocol(), yaw_rate_feedback_gain_s=1.0, drive_model=drive_model
    )
    policy.reset(straight_oracle_context("forward"))

    action = policy.act(
        valid_observation(yaw_rate_rad_s=1.0), pointgoal_base_xy=(2.0, 0.0)
    )
    feedforward = OraclePathPolicy(
        aux30_protocol(), yaw_rate_feedback_gain_s=0.0, drive_model=drive_model
    )
    feedforward.reset(straight_oracle_context("forward"))
    feedforward_action = feedforward.act(
        valid_observation(yaw_rate_rad_s=1.0), pointgoal_base_xy=(2.0, 0.0)
    )

    assert float(action[1] - feedforward_action[1]) == pytest.approx(
        math.radians(5.0), abs=1e-6
    )


@pytest.mark.parametrize("value", (-1.0, True, float("nan"), float("inf"), float("-inf")))
def test_oracle_rejects_invalid_yaw_rate_feedback_gain(value: object):
    with pytest.raises(ValueError, match="yaw_rate_feedback_gain_s"):
        OraclePathPolicy(aux30_protocol(), yaw_rate_feedback_gain_s=value)


def test_dynamic_oracle_uses_action_clock_feedforward_and_terminal_stop():
    policy = benchmark_policies.DynamicOraclePolicy(aux30_protocol())
    policy.reset(dynamic_context())

    first = policy.act(
        valid_observation(auxiliary_angle_rad=0.0), pointgoal_base_xy=(1.0, 0.0)
    )
    second = policy.act(
        valid_observation(auxiliary_angle_rad=math.radians(10.0)),
        pointgoal_base_xy=(0.95, 0.0),
    )
    terminal = policy.act(
        valid_observation(position_world_xy=(0.1, 0.0)),
        pointgoal_base_xy=(0.9, 0.0),
    )

    np.testing.assert_allclose(first.reshape(5, 2), ((0.5, 0.0),) * 5)
    np.testing.assert_allclose(
        second.reshape(5, 2), ((1.0, math.radians(10.0)),) * 5, atol=1e-7
    )
    np.testing.assert_array_equal(terminal, np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["reference_index"] == 2


@pytest.mark.parametrize(
    ("observation_kwargs", "expected_main_limit"),
    (
        ({"position_world_xy": (0.0, 0.1)}, 0.5),
        ({"yaw_world_rad": math.radians(20.0)}, 0.25),
        ({"auxiliary_angle_rad": math.radians(30.0)}, 0.0),
    ),
)
def test_dynamic_oracle_caps_main_command_at_frozen_error_thresholds(
    observation_kwargs: dict[str, object], expected_main_limit: float
):
    reference = (
        DynamicWaypoint(0.0, 0.0, 0.0, 1.5, 0.0, 0.5, 0.0, "forward"),
        DynamicWaypoint(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "stopped"),
    )
    policy = benchmark_policies.DynamicOraclePolicy(aux30_protocol())
    policy.reset(dynamic_context(reference))

    action = policy.act(
        valid_observation(**observation_kwargs), pointgoal_base_xy=(2.0, 0.0)
    )

    assert abs(float(action[0])) <= expected_main_limit + 1e-7
    assert abs(float(action[1])) <= math.radians(5.0) + 1e-7


def test_dynamic_oracle_waits_five_stopped_cycles_before_reverse_feedforward():
    reference = (
        DynamicWaypoint(0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward"),
        *tuple(
            DynamicWaypoint(
                -0.05 * index,
                0.0,
                0.0,
                -0.5,
                0.0,
                0.2,
                0.1 * index,
                "reverse",
            )
            for index in range(1, 9)
        ),
        DynamicWaypoint(-0.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9, "stopped"),
    )
    policy = benchmark_policies.DynamicOraclePolicy(aux30_protocol())
    policy.reset(dynamic_context(reference))
    policy.act(valid_observation(), pointgoal_base_xy=(2.0, 0.0))

    moving = policy.act(
        valid_observation(forward_speed_m_s=0.2), pointgoal_base_xy=(2.0, 0.0)
    )
    stopped = [
        policy.act(valid_observation(), pointgoal_base_xy=(2.0, 0.0))
        for _ in range(5)
    ]

    np.testing.assert_array_equal(moving, np.zeros(10, dtype=np.float32))
    for action in stopped[:4]:
        np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert stopped[4][0] < 0.0
    assert policy.diagnostics["active_drive_direction"] == "reverse"


def test_dynamic_oracle_counts_planned_stop_reference_toward_reversal_dwell():
    reference = (
        DynamicWaypoint(0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward"),
        *tuple(
            DynamicWaypoint(
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * index, "stopped"
            )
            for index in range(1, 6)
        ),
        DynamicWaypoint(0.0, 0.0, 0.0, -0.5, 0.0, 0.2, 0.6, "reverse"),
        DynamicWaypoint(-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7, "stopped"),
    )
    policy = benchmark_policies.DynamicOraclePolicy(aux30_protocol())
    policy.reset(dynamic_context(reference))
    policy.act(valid_observation(), pointgoal_base_xy=(2.0, 0.0))

    planned_stop = [
        policy.act(valid_observation(), pointgoal_base_xy=(2.0, 0.0))
        for _ in range(5)
    ]
    reverse = policy.act(valid_observation(), pointgoal_base_xy=(2.0, 0.0))

    for action in planned_stop:
        np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert reverse[0] < 0.0


def test_stop_policy_returns_exactly_zero_v2_block():
    policy = StopPolicy(protocol=aux30_protocol())
    policy.reset(non_privileged_context())

    action = policy.act(valid_observation(), np.asarray((2.0, 0.0)))

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))


def test_random_policy_is_reproducible_and_within_protocol_bounds():
    first = RandomPolicy(protocol=aux30_protocol(), seed=3072)
    second = RandomPolicy(protocol=aux30_protocol(), seed=3072)
    first.reset(non_privileged_context())
    second.reset(non_privileged_context())

    first_actions = [first.act(valid_observation(), np.asarray((2.0, 0.0))) for _ in range(3)]
    second_actions = [second.act(valid_observation(), np.asarray((2.0, 0.0))) for _ in range(3)]
    bounds = aux30_protocol().block_bounds_numpy()

    for left, right in zip(first_actions, second_actions):
        np.testing.assert_array_equal(left, right)
        assert np.all(left >= bounds[:, 0])
        assert np.all(left <= bounds[:, 1])


def test_goal_greedy_changes_turn_sign_with_goal_side():
    policy = GoalGreedyPolicy(protocol=aux30_protocol())
    policy.reset(non_privileged_context())

    left = policy.act(valid_observation(), np.asarray((2.0, 1.0)))
    right = policy.act(valid_observation(), np.asarray((2.0, -1.0)))

    assert np.sign(left[1]) == -np.sign(right[1])
    assert left[1] != 0.0
    # The simulator's auxiliary joint sign is opposite to the base-frame
    # lateral convention: positive auxiliary angle produces a right turn.
    assert left[1] < 0.0
    assert right[1] > 0.0
    assert left.shape == right.shape == (10,)
    np.testing.assert_allclose(left.reshape(5, 2), np.tile(left[:2], (5, 1)))


def test_goal_greedy_slows_inside_sixty_centimeters_and_stops_at_goal():
    policy = GoalGreedyPolicy(protocol=aux30_protocol())
    policy.reset(non_privileged_context())

    far = policy.act(valid_observation(), np.asarray((2.0, 0.0)))
    near = policy.act(valid_observation(), np.asarray((0.3, 0.0)))
    reached = policy.act(valid_observation(), np.asarray((0.1, 0.0)))

    assert 0.0 < near[0] < far[0]
    np.testing.assert_array_equal(reached, np.zeros(10, dtype=np.float32))


def test_calibrated_steering_maps_curvature_to_measured_actuator_convention():
    left = calibrated_auxiliary_angle(0.4, main_velocity_rad_s=1.0)
    right = calibrated_auxiliary_angle(-0.4, main_velocity_rad_s=1.0)

    assert math.radians(-30.0) <= left < 0.0
    assert 0.0 < right <= math.radians(30.0)
    assert math.radians(-10.0) < calibrated_auxiliary_angle(
        0.0, main_velocity_rad_s=1.0
    ) < 0.0


def test_calibrated_stopping_distance_interpolates_and_increases_with_speed():
    distances = [
        calibrated_stopping_distance(speed)
        for speed in (0.0, 0.21, 0.39, 0.55, 0.75)
    ]

    assert distances == sorted(distances)
    assert distances[0] == pytest.approx(0.0)
    assert distances[1] == pytest.approx(0.20, abs=0.03)
    assert distances[3] == pytest.approx(1.09, abs=0.05)
    assert distances[-1] > distances[-2]


def test_adaptive_lookahead_grows_with_speed_but_stays_bounded():
    assert adaptive_lookahead_distance(0.0) == pytest.approx(0.45)
    assert adaptive_lookahead_distance(0.4) > adaptive_lookahead_distance(0.1)
    assert adaptive_lookahead_distance(10.0) == pytest.approx(0.9)


def test_curvature_tracker_slows_for_turn_and_uses_yaw_rate_feedback():
    protocol = aux30_protocol()
    straight = curvature_tracker_action(
        protocol,
        target_base_xy=(1.0, 0.0),
        current_speed_m_s=0.4,
        current_yaw_rate_rad_s=0.0,
        remaining_path_m=5.0,
    )
    turning = curvature_tracker_action(
        protocol,
        target_base_xy=(0.45, 0.25),
        current_speed_m_s=0.4,
        current_yaw_rate_rad_s=0.0,
        remaining_path_m=5.0,
    )
    already_turning = curvature_tracker_action(
        protocol,
        target_base_xy=(0.45, 0.25),
        current_speed_m_s=0.4,
        current_yaw_rate_rad_s=0.8,
        remaining_path_m=5.0,
    )

    assert straight[0] == pytest.approx(2.0)
    assert 0.0 < turning[0] < straight[0]
    assert turning[1] < 0.0
    assert abs(already_turning[1]) < abs(turning[1])


def test_curvature_tracker_commands_braking_before_goal_from_measured_speed():
    action = curvature_tracker_action(
        aux30_protocol(),
        target_base_xy=(0.8, 0.0),
        current_speed_m_s=0.55,
        current_yaw_rate_rad_s=0.0,
        remaining_path_m=1.15,
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))


def test_oracle_requires_privileged_context_and_path():
    policy = OraclePathPolicy(protocol=aux30_protocol())

    with pytest.raises(PermissionError, match="privileged"):
        policy.reset(non_privileged_context())

    missing_path = EpisodeContext(
        episode_id="bad",
        privileged=True,
        oracle_path_xy=None,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
    )
    with pytest.raises(ValueError, match="oracle_path"):
        policy.reset(missing_path)

    missing_reference = EpisodeContext(
        episode_id="bad-reference",
        privileged=True,
        oracle_path_xy=np.asarray(((0.0, 0.0), (1.0, 0.0))),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
    )
    with pytest.raises(ValueError, match="kinodynamic_reference"):
        policy.reset(missing_reference)


def test_non_oracle_policies_reject_privileged_path_context():
    for policy in (
        StopPolicy(aux30_protocol()),
        RandomPolicy(aux30_protocol(), seed=3072),
        GoalGreedyPolicy(aux30_protocol()),
    ):
        with pytest.raises(PermissionError, match="non-privileged"):
            policy.reset(privileged_context())


def test_oracle_path_index_moves_monotonically_forward():
    policy = OraclePathPolicy(protocol=aux30_protocol(), lookahead_m=0.5)
    policy.reset(privileged_context())
    indices: list[int] = []

    for position in ((0.0, 0.0), (0.55, 0.02), (1.05, 0.32), (1.55, 0.82)):
        action = policy.act(
            valid_observation(position_world_xy=position),
            np.asarray((2.0, 1.0)),
        )
        indices.append(policy.path_index)
        assert action.shape == (10,)

    assert indices == sorted(indices)
    assert indices[-1] > indices[0]


def test_oracle_speed_adaptive_lookahead_anticipates_a_nearby_bend():
    path = np.asarray(
        [(0.1 * index, 0.0) for index in range(6)]
        + [(0.5, 0.1 * index) for index in range(1, 21)],
        dtype=np.float64,
    )
    context = EpisodeContext(
        episode_id="bend",
        privileged=True,
        oracle_path_xy=path,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference_from_xy(path),
    )
    slow = OraclePathPolicy(protocol=aux30_protocol())
    fast = OraclePathPolicy(protocol=aux30_protocol())
    slow.reset(context)
    fast.reset(context)

    slow_action = slow.act(
        valid_observation(forward_speed_m_s=0.0),
        pointgoal_base_xy=(0.5, 2.0),
    )
    fast_action = fast.act(
        valid_observation(forward_speed_m_s=0.55),
        pointgoal_base_xy=(0.5, 2.0),
    )

    assert abs(slow_action[1]) < math.radians(3.0)
    assert fast_action[1] < slow_action[1]
    assert fast_action[0] < slow_action[0]


def test_oracle_uses_calibrated_braking_distance_before_path_goal():
    path = np.asarray(
        [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0), (1.15, 0.0)],
        dtype=np.float64,
    )
    policy = OraclePathPolicy(protocol=aux30_protocol())
    policy.reset(
        EpisodeContext(
            episode_id="braking",
            privileged=True,
            oracle_path_xy=path,
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=reference_from_xy(path),
        )
    )

    action = policy.act(
        valid_observation(forward_speed_m_s=0.55),
        pointgoal_base_xy=(1.15, 0.0),
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["braking_state"] == "goal_braking"


def test_oracle_stops_for_five_cycles_before_reversing_main_command_sign():
    policy = OraclePathPolicy(protocol=aux30_protocol())
    policy.reset(
        EpisodeContext(
            episode_id="direction-change",
            privileged=True,
            oracle_path_xy=np.asarray(((0.0, 0.0), (0.6, 0.0), (0.3, -0.1))),
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=bidirectional_reference(),
        )
    )

    moving = policy.act(
        valid_observation(position_world_xy=(0.6, 0.0), forward_speed_m_s=0.2),
        pointgoal_base_xy=(-0.3, -0.1),
    )
    stopped = [
        policy.act(
            valid_observation(position_world_xy=(0.6, 0.0)),
            pointgoal_base_xy=(-0.3, -0.1),
        )
        for _ in range(5)
    ]

    np.testing.assert_array_equal(moving, np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["desired_drive_direction"] == "reverse"
    for action in stopped[:4]:
        np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert stopped[4][0] < 0.0
    assert policy.diagnostics["active_drive_direction"] == "reverse"
    assert policy.diagnostics["cross_track_error_m"] == pytest.approx(0.0)


def test_oracle_resets_stop_counter_if_speed_rebounds_during_direction_change():
    policy = OraclePathPolicy(protocol=aux30_protocol())
    policy.reset(
        EpisodeContext(
            episode_id="direction-rebound",
            privileged=True,
            oracle_path_xy=np.asarray(((0.0, 0.0), (0.6, 0.0), (0.3, -0.1))),
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=bidirectional_reference(),
        )
    )
    for _ in range(4):
        policy.act(
            valid_observation(position_world_xy=(0.6, 0.0)),
            pointgoal_base_xy=(-0.3, -0.1),
        )
    policy.act(
        valid_observation(position_world_xy=(0.6, 0.0), forward_speed_m_s=0.08),
        pointgoal_base_xy=(-0.3, -0.1),
    )
    action = policy.act(
        valid_observation(position_world_xy=(0.6, 0.0)),
        pointgoal_base_xy=(-0.3, -0.1),
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["stopped_cycles"] == 1


def test_oracle_latches_safe_stop_after_ten_off_reference_cycles():
    policy = OraclePathPolicy(protocol=aux30_protocol())
    policy.reset(privileged_context())

    actions = [
        policy.act(
            valid_observation(position_world_xy=(-2.0, 0.0)),
            pointgoal_base_xy=(4.0, 1.0),
        )
        for _ in range(10)
    ]

    assert any(action[0] != 0.0 for action in actions[:9])
    np.testing.assert_array_equal(actions[-1], np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["failure_subreason"] == "tracking_infeasible"


def test_policy_reset_rejects_action_protocol_mismatch():
    context = EpisodeContext(
        episode_id="legacy",
        privileged=False,
        oracle_path_xy=None,
        action_protocol_id="ball_action_v1_aux16",
        kinodynamic_reference=None,
    )

    with pytest.raises(ValueError, match="ball_action_v1_aux16"):
        StopPolicy(aux30_protocol()).reset(context)


@pytest.mark.parametrize(
    "overrides",
    (
        {"stereo_left_rgb": np.zeros((480, 640), dtype=np.uint8)},
        {"stereo_right_rgb": np.zeros((480, 640, 3), dtype=np.float32)},
        {"proprio_history": np.zeros((5, 16), dtype=np.float32)},
        {"action_history": np.zeros((5, 8), dtype=np.float32)},
        {"position_world_xy": np.zeros(3, dtype=np.float64)},
        {"yaw_world_rad": float("nan")},
    ),
)
def test_policy_observation_rejects_malformed_inputs(overrides: dict[str, object]):
    values: dict[str, object] = {
        "stereo_left_rgb": np.zeros((480, 640, 3), dtype=np.uint8),
        "stereo_right_rgb": np.zeros((480, 640, 3), dtype=np.uint8),
        "proprio_history": np.zeros((5, 17), dtype=np.float32),
        "action_history": np.zeros((5, 10), dtype=np.float32),
        "position_world_xy": np.zeros(2, dtype=np.float64),
        "yaw_world_rad": 0.0,
    }
    values.update(overrides)

    with pytest.raises(ValueError):
        PolicyObservation(**values)


def test_trajectory_oracle_has_exact_public_defaults_and_reference_privilege_boundary():
    policy = TrajectoryOraclePolicy(aux30_protocol())

    assert policy.policy_name == "oracle_trajectory_v1"
    assert policy.lookahead_m == pytest.approx(1.0)
    assert policy.smoothing_window_m == pytest.approx(0.40)
    assert policy.curvature_preview_m == pytest.approx(0.80)
    assert policy.lateral_curvature_gain_1_m2 == pytest.approx(0.20)
    assert policy.heading_curvature_gain_1_m == pytest.approx(0.15)
    assert policy.max_feedback_curvature_1_m == pytest.approx(0.20)
    assert policy.yaw_rate_feedback_gain_s == pytest.approx(0.15)
    assert policy.latency_preview_s == pytest.approx(0.0)
    assert "kinodynamic_reference" in policy.online_input_permissions
    assert "oracle_path_xy" not in policy.online_input_permissions
    assert "dynamic_reference" not in policy.online_input_permissions


@pytest.mark.parametrize(
    ("parameter", "value"),
    (
        ("lookahead_m", 0.0),
        ("lookahead_m", -1.0),
        ("smoothing_window_m", -1.0),
        ("curvature_preview_m", -1.0),
        ("lateral_curvature_gain_1_m2", -1.0),
        ("heading_curvature_gain_1_m", -1.0),
        ("max_feedback_curvature_1_m", -1.0),
        ("yaw_rate_feedback_gain_s", -1.0),
        ("latency_preview_s", 1e-9),
        ("lookahead_m", True),
        ("smoothing_window_m", np.bool_(True)),
        ("curvature_preview_m", True),
        ("lateral_curvature_gain_1_m2", True),
        ("heading_curvature_gain_1_m", True),
        ("max_feedback_curvature_1_m", True),
        ("yaw_rate_feedback_gain_s", True),
        ("latency_preview_s", True),
        ("lookahead_m", float("nan")),
        ("smoothing_window_m", float("inf")),
        ("curvature_preview_m", float("-inf")),
        ("lateral_curvature_gain_1_m2", float("nan")),
        ("heading_curvature_gain_1_m", float("inf")),
        ("max_feedback_curvature_1_m", float("nan")),
        ("yaw_rate_feedback_gain_s", float("-inf")),
        ("latency_preview_s", float("nan")),
    ),
)
def test_trajectory_oracle_rejects_invalid_scalar_parameters(
    parameter: str, value: object
):
    with pytest.raises(ValueError, match=parameter):
        TrajectoryOraclePolicy(aux30_protocol(), **{parameter: value})


def test_trajectory_oracle_validates_privileged_kinodynamic_context_and_resets_profile_state():
    policy = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)

    with pytest.raises(PermissionError, match="privileged"):
        policy.reset(non_privileged_context())

    missing_reference = EpisodeContext(
        episode_id="trajectory-missing-reference",
        privileged=True,
        oracle_path_xy=None,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=None,
    )
    with pytest.raises(ValueError, match="kinodynamic_reference"):
        policy.reset(missing_reference)

    mixed = EpisodeContext(
        episode_id="trajectory-mixed-reference",
        privileged=True,
        oracle_path_xy=np.asarray(((0.0, 0.0), (1.0, 0.0))),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=trajectory_line_context().kinodynamic_reference,
        dynamic_reference=dynamic_reference(),
    )
    with pytest.raises(ValueError, match="dynamic_reference"):
        policy.reset(mixed)

    context = trajectory_line_context(include_oracle_path=False)
    policy.reset(context)
    policy.act(valid_observation(position_world_xy=(1.2, 0.0)), (10.0, 0.0))
    assert policy.path_index > 0
    policy.reset(context)
    assert policy.path_index == 0
    assert policy.diagnostics["reference_index"] == 0
    assert policy.diagnostics["active_drive_direction"] == "forward"
    assert policy.diagnostics["pending_drive_direction"] is None
    assert policy.diagnostics["stopped_cycles"] == 0
    assert policy.diagnostics["failure_subreason"] is None
    for name in (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    ):
        assert math.isfinite(float(policy.diagnostics[name]))


def test_trajectory_oracle_straight_reference_has_zero_curvature_feedforward():
    policy = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    policy.reset(trajectory_line_context(include_oracle_path=False))

    action = policy.act(
        valid_observation(), pointgoal_base_xy=(10.0, 0.0)
    )
    diagnostics = policy.diagnostics

    assert diagnostics["reference_curvature_1_m"] == pytest.approx(0.0, abs=1e-12)
    assert diagnostics["feedback_curvature_1_m"] == pytest.approx(0.0, abs=1e-12)
    assert diagnostics["command_curvature_1_m"] == pytest.approx(0.0, abs=1e-12)
    assert diagnostics["main_cap_rad_s"] == pytest.approx(0.50)
    assert action[0] == pytest.approx(0.50)
    assert float(policy.drive_model.curvature(float(action[0]), float(action[1]))) == pytest.approx(
        0.0, abs=1e-8
    )


@pytest.mark.parametrize(
    ("reference", "expected_spatial_curvature", "expected_main_sign"),
    (
        (_constant_curvature_reference(1.0 / 3.0), 1.0 / 3.0, 1.0),
        (_reverse_quarter_circle_reference(), -1.0 / 3.0, -1.0),
    ),
)
def test_trajectory_oracle_r3_feedforward_sign_and_reverse_drive_model_mapping(
    reference: tuple[KinodynamicWaypoint, ...],
    expected_spatial_curvature: float,
    expected_main_sign: float,
):
    policy = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    policy.reset(trajectory_context_from_reference(reference, include_oracle_path=False))

    action = policy.act(valid_observation(), pointgoal_base_xy=(20.0, 0.0))
    diagnostics = policy.diagnostics
    main = float(action[0])
    auxiliary_feedforward = float(diagnostics["auxiliary_feedforward_rad"])
    spatial = float(diagnostics["reference_curvature_1_m"])
    drive_sign = 1.0 if main > 0.0 else -1.0

    assert main * expected_main_sign > 0.0
    assert spatial == pytest.approx(expected_spatial_curvature, abs=2e-3)
    assert float(policy.drive_model.curvature(main, auxiliary_feedforward)) == pytest.approx(
        drive_sign * spatial, abs=2e-3
    )
    if expected_spatial_curvature > 0.0:
        assert auxiliary_feedforward < 0.0


def test_trajectory_oracle_feedback_uses_signed_lateral_error_and_wrapped_body_heading():
    left = TrajectoryOraclePolicy(
        aux30_protocol(),
        smoothing_window_m=0.0,
        curvature_preview_m=0.0,
    )
    right = TrajectoryOraclePolicy(
        aux30_protocol(),
        smoothing_window_m=0.0,
        curvature_preview_m=0.0,
    )
    context = trajectory_line_context(include_oracle_path=False)
    left.reset(context)
    right.reset(context)

    left_action = left.act(
        valid_observation(position_world_xy=(0.0, 0.20)), (10.0, 0.0)
    )
    right_action = right.act(
        valid_observation(position_world_xy=(0.0, -0.20)), (10.0, 0.0)
    )

    assert left.diagnostics["lateral_error_m"] == pytest.approx(0.20)
    assert right.diagnostics["lateral_error_m"] == pytest.approx(-0.20)
    assert left.diagnostics["feedback_curvature_1_m"] < 0.0
    assert right.diagnostics["feedback_curvature_1_m"] > 0.0
    assert left_action[1] > right_action[1]
    assert left.diagnostics["main_cap_rad_s"] == pytest.approx(0.25)

    wrapped = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    wrapped.reset(trajectory_line_context(yaw_rad=3.1, include_oracle_path=False))
    wrapped.act(
        valid_observation(yaw_world_rad=-3.1), pointgoal_base_xy=(10.0, 0.0)
    )
    assert wrapped.diagnostics["heading_error_rad"] == pytest.approx(
        -0.08318530717958605, abs=1e-12
    )
    assert wrapped.diagnostics["feedback_curvature_1_m"] < 0.0


def test_trajectory_oracle_feedback_curvature_clips_at_configured_plus_minus_020():
    positive = TrajectoryOraclePolicy(
        aux30_protocol(),
        smoothing_window_m=0.0,
        curvature_preview_m=0.0,
        lateral_curvature_gain_1_m2=1.0,
    )
    negative = TrajectoryOraclePolicy(
        aux30_protocol(),
        smoothing_window_m=0.0,
        curvature_preview_m=0.0,
        lateral_curvature_gain_1_m2=1.0,
    )
    context = trajectory_line_context(include_oracle_path=False)
    positive.reset(context)
    negative.reset(context)

    positive.act(valid_observation(position_world_xy=(0.0, 1.0)), (10.0, 0.0))
    negative.act(valid_observation(position_world_xy=(0.0, -1.0)), (10.0, 0.0))

    assert positive.diagnostics["feedback_curvature_1_m"] == pytest.approx(-0.20)
    assert negative.diagnostics["feedback_curvature_1_m"] == pytest.approx(0.20)
    assert abs(float(positive.diagnostics["command_curvature_1_m"])) == pytest.approx(
        0.20
    )
    assert abs(float(negative.diagnostics["command_curvature_1_m"])) == pytest.approx(
        0.20
    )


def test_trajectory_oracle_auxiliary_feedback_diagnostic_is_pre_protocol_clip():
    policy = TrajectoryOraclePolicy(
        aux30_protocol(),
        smoothing_window_m=0.0,
        yaw_rate_feedback_gain_s=0.15,
    )
    policy.reset(
        trajectory_context_from_reference(
            _constant_curvature_reference(1.0), include_oracle_path=False
        )
    )

    action = policy.act(
        valid_observation(yaw_rate_rad_s=-1.0), pointgoal_base_xy=(20.0, 0.0)
    )
    diagnostics = policy.diagnostics
    protocol_limit = aux30_protocol().maximum[1]
    preclip_auxiliary = (
        float(diagnostics["auxiliary_feedforward_rad"])
        + float(diagnostics["auxiliary_feedback_rad"])
    )

    assert abs(float(action[1])) == pytest.approx(protocol_limit)
    assert abs(preclip_auxiliary) > protocol_limit


def test_trajectory_oracle_reverse_motion_tangent_flips_lateral_sign_and_main_only():
    forward = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    reverse = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    forward.reset(trajectory_line_context(direction="forward", include_oracle_path=False))
    reverse.reset(trajectory_line_context(direction="reverse", include_oracle_path=False))

    forward_action = forward.act(
        valid_observation(position_world_xy=(0.0, 0.20)), (10.0, 0.0)
    )
    reverse_action = reverse.act(
        valid_observation(position_world_xy=(0.0, 0.20)), (10.0, 0.0)
    )

    assert forward.diagnostics["lateral_error_m"] == pytest.approx(0.20)
    assert reverse.diagnostics["lateral_error_m"] == pytest.approx(-0.20)
    assert forward.diagnostics["feedback_curvature_1_m"] < 0.0
    assert reverse.diagnostics["feedback_curvature_1_m"] > 0.0
    assert reverse_action[0] == pytest.approx(-forward_action[0])
    assert abs(float(reverse_action[0])) == pytest.approx(abs(float(forward_action[0])))


def test_trajectory_oracle_waits_five_stopped_cycles_before_reverse_command():
    policy = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    policy.reset(
        EpisodeContext(
            episode_id="trajectory-direction-change",
            privileged=True,
            oracle_path_xy=np.asarray(((0.0, 0.0), (0.3, 0.0), (0.6, 0.0), (0.3, -0.1))),
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=bidirectional_reference(),
        )
    )

    moving = policy.act(
        valid_observation(position_world_xy=(0.6, 0.0), forward_speed_m_s=0.2),
        pointgoal_base_xy=(-0.3, -0.1),
    )
    stopped = [
        policy.act(
            valid_observation(position_world_xy=(0.6, 0.0)),
            pointgoal_base_xy=(-0.3, -0.1),
        )
        for _ in range(5)
    ]

    np.testing.assert_array_equal(moving, np.zeros(10, dtype=np.float32))
    for action in stopped[:4]:
        np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert stopped[4][0] < 0.0
    assert policy.diagnostics["active_drive_direction"] == "reverse"
    assert policy.diagnostics["desired_drive_direction"] == "reverse"


def test_trajectory_oracle_preserves_goal_and_remaining_distance_braking():
    reached = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    reached.reset(trajectory_line_context(include_oracle_path=False))
    reached_action = reached.act(
        valid_observation(), pointgoal_base_xy=(0.1, 0.0)
    )
    np.testing.assert_array_equal(reached_action, np.zeros(10, dtype=np.float32))
    assert reached.diagnostics["braking_state"] == "goal_reached"

    braking = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    braking.reset(trajectory_line_context(include_oracle_path=False))
    braking_action = braking.act(
        valid_observation(position_world_xy=(3.0, 0.0), forward_speed_m_s=0.55),
        pointgoal_base_xy=(10.0, 0.0),
    )
    np.testing.assert_array_equal(braking_action, np.zeros(10, dtype=np.float32))
    assert braking.diagnostics["braking_state"] == "goal_braking"


def test_trajectory_oracle_latches_off_reference_failure_with_finite_new_diagnostics():
    policy = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    policy.reset(trajectory_line_context(include_oracle_path=False))

    actions = [
        policy.act(
            valid_observation(position_world_xy=(-2.0, 0.0)),
            pointgoal_base_xy=(10.0, 0.0),
        )
        for _ in range(10)
    ]

    np.testing.assert_array_equal(actions[-1], np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["failure_subreason"] == "tracking_infeasible"
    assert policy.diagnostics["braking_state"] == "tracking_infeasible"
    for name in (
        "reference_curvature_1_m",
        "feedback_curvature_1_m",
        "command_curvature_1_m",
        "lateral_error_m",
        "heading_error_rad",
        "main_cap_rad_s",
        "auxiliary_feedforward_rad",
        "auxiliary_feedback_rad",
    ):
        assert math.isfinite(float(policy.diagnostics[name]))


@pytest.mark.parametrize(
    ("curvature", "expected_cap"),
    ((0.19, 0.50), (0.20, 0.35), (0.45, 0.25)),
)
def test_trajectory_oracle_speed_caps_follow_preview_curvature_thresholds(
    curvature: float, expected_cap: float
):
    reference = _constant_curvature_reference(curvature, point_count=5)
    policy = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.80
    )
    policy.reset(trajectory_context_from_reference(reference, include_oracle_path=False))

    action = policy.act(valid_observation(), pointgoal_base_xy=(20.0, 0.0))

    assert policy.diagnostics["main_cap_rad_s"] == pytest.approx(expected_cap)
    assert abs(float(action[0])) == pytest.approx(expected_cap)


@pytest.mark.parametrize(
    "observation",
    (
        valid_observation(position_world_xy=(0.0, 0.15)),
        valid_observation(yaw_world_rad=-math.radians(20.0)),
    ),
)
def test_trajectory_oracle_error_gate_caps_main_at_025(observation: PolicyObservation):
    policy = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    policy.reset(trajectory_line_context(include_oracle_path=False))

    action = policy.act(observation, pointgoal_base_xy=(20.0, 0.0))

    assert policy.diagnostics["main_cap_rad_s"] == pytest.approx(0.25)
    assert abs(float(action[0])) == pytest.approx(0.25)


def test_trajectory_oracle_preview_reduces_speed_before_entering_a_high_curvature_bend():
    turn_curvature = 1.0
    p0 = KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 1.5, 0.0)
    p1 = KinodynamicWaypoint(0.4, 0.0, 0.0, "forward", 1.5, 0.0)
    p2 = KinodynamicWaypoint(0.8, 0.0, 0.0, "forward", 1.5, 0.0)
    yaw3 = turn_curvature * 0.4
    p3 = KinodynamicWaypoint(
        0.8 + math.sin(yaw3) / turn_curvature,
        (-math.cos(yaw3) + 1.0) / turn_curvature,
        yaw3,
        "forward",
        1.5,
        0.0,
    )
    p4 = KinodynamicWaypoint(
        p3.x_m + (math.sin(2.0 * yaw3) - math.sin(yaw3)) / turn_curvature,
        p3.y_m + (-math.cos(2.0 * yaw3) + math.cos(yaw3)) / turn_curvature,
        2.0 * yaw3,
        "forward",
        0.0,
        0.0,
    )
    context = trajectory_context_from_reference((p0, p1, p2, p3, p4), include_oracle_path=False)
    with_preview = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.80
    )
    without_preview = TrajectoryOraclePolicy(
        aux30_protocol(), smoothing_window_m=0.0, curvature_preview_m=0.0
    )
    with_preview.reset(context)
    without_preview.reset(context)

    preview_action = with_preview.act(valid_observation(), (20.0, 0.0))
    current_only_action = without_preview.act(valid_observation(), (20.0, 0.0))

    assert with_preview.diagnostics["reference_curvature_1_m"] > 0.45
    assert with_preview.diagnostics["main_cap_rad_s"] == pytest.approx(0.25)
    assert without_preview.diagnostics["main_cap_rad_s"] == pytest.approx(0.50)
    assert abs(float(preview_action[0])) < abs(float(current_only_action[0]))


def test_oracle_path_policy_frozen_literal_fixture_is_unchanged():
    policy = OraclePathPolicy(aux30_protocol())
    policy.reset(privileged_context())

    first = policy.act(
        valid_observation(yaw_rate_rad_s=0.2), pointgoal_base_xy=(2.0, 1.0)
    )
    first_diagnostics = policy.diagnostics
    second = policy.act(
        valid_observation(position_world_xy=(0.55, 0.02), forward_speed_m_s=0.4, yaw_rate_rad_s=0.2),
        pointgoal_base_xy=(2.0, 1.0),
    )
    second_diagnostics = policy.diagnostics

    np.testing.assert_array_equal(
        first,
        np.asarray(
            [
                1.0,
                -0.018333205953240395,
                1.0,
                -0.018333205953240395,
                1.0,
                -0.018333205953240395,
                1.0,
                -0.018333205953240395,
                1.0,
                -0.018333205953240395,
            ],
            dtype=np.float32,
        ),
    )
    np.testing.assert_array_equal(second, np.zeros(10, dtype=np.float32))
    assert first_diagnostics == {
        "reference_index": 0,
        "active_drive_direction": "forward",
        "pending_drive_direction": None,
        "stopped_cycles": 0,
        "off_reference_cycles": 0,
        "failure_subreason": None,
        "cross_track_error_m": 0.0,
        "desired_drive_direction": "forward",
        "braking_state": "tracking",
    }
    assert second_diagnostics == {
        "reference_index": 1,
        "active_drive_direction": "forward",
        "pending_drive_direction": None,
        "stopped_cycles": 0,
        "off_reference_cycles": 0,
        "failure_subreason": None,
        "cross_track_error_m": 0.053851648071345085,
        "desired_drive_direction": "forward",
        "braking_state": "goal_braking",
    }
    assert policy.policy_name == "oracle"


def test_legacy_policy_golden_actions_remain_unchanged():
    test_oracle_path_policy_frozen_literal_fixture_is_unchanged()


def test_trajectory_v2_is_independent_and_rejects_dynamic_reference():
    context = trajectory_line_context(include_oracle_path=False)
    policy = DelayAwareTrajectoryOraclePolicy(aux30_protocol())

    assert policy.policy_name == "oracle_trajectory_v2"
    assert not isinstance(policy, TrajectoryOraclePolicy)
    with pytest.raises(ValueError, match="dynamic_reference"):
        policy.reset(replace(context, dynamic_reference=dynamic_reference()))


def _terminal_brake_buffer_context() -> EpisodeContext:
    points = tuple(
        KinodynamicWaypoint(
            0.5 * index,
            0.0,
            0.0,
            "forward",
            1.5 if index < 4 else 0.0,
            0.0,
        )
        for index in range(5)
    )
    return EpisodeContext(
        episode_id="terminal-brake-buffer",
        privileged=True,
        oracle_path_xy=np.asarray([(point.x_m, point.y_m) for point in points]),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=points,
    )


def test_trajectory_v2_exposes_auditable_terminal_brake_buffer_default_and_candidate():
    default = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    candidate = DelayAwareTrajectoryOraclePolicy(
        aux30_protocol(), terminal_brake_buffer_m=0.30
    )

    assert default.terminal_brake_buffer_m == pytest.approx(0.10)
    assert candidate.terminal_brake_buffer_m == pytest.approx(0.30)


def test_trajectory_v2_candidate_starts_terminal_braking_earlier_and_outputs_zero():
    context = _terminal_brake_buffer_context()
    default = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    candidate = DelayAwareTrajectoryOraclePolicy(
        aux30_protocol(), terminal_brake_buffer_m=0.30
    )
    default.reset(context)
    candidate.reset(context)
    observation = valid_observation(
        position_world_xy=(1.70, 0.0), forward_speed_m_s=0.40
    )

    default_action = default.act(observation, pointgoal_base_xy=(10.0, 0.0))
    candidate_action = candidate.act(observation, pointgoal_base_xy=(10.0, 0.0))

    assert float(default.diagnostics["terminal_runway_s"]) > -100.0
    assert default_action[0] != pytest.approx(0.0)
    np.testing.assert_array_equal(candidate_action, np.zeros(10, dtype=np.float32))
    assert candidate.diagnostics["braking_state"] == "goal_braking"


def test_trajectory_v2_terminal_buffer_does_not_change_baseline_or_v1_behavior():
    context = _terminal_brake_buffer_context()
    observation = valid_observation(
        position_world_xy=(1.70, 0.0), forward_speed_m_s=0.40
    )
    baseline = OraclePathPolicy(aux30_protocol())
    v1 = TrajectoryOraclePolicy(aux30_protocol(), smoothing_window_m=0.0)
    baseline.reset(context)
    v1.reset(context)

    baseline_action = baseline.act(observation, pointgoal_base_xy=(10.0, 0.0))
    v1_action = v1.act(observation, pointgoal_base_xy=(10.0, 0.0))

    assert not hasattr(baseline, "terminal_brake_buffer_m")
    assert not hasattr(v1, "terminal_brake_buffer_m")
    assert baseline_action[0] != pytest.approx(0.0)
    assert v1_action[0] != pytest.approx(0.0)


def test_trajectory_v2_has_deterministic_reset_and_owns_tracker_state():
    context = trajectory_context_from_reference(
        _constant_curvature_reference(1.0 / 3.0), include_oracle_path=False
    )
    first = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    second = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    observation = valid_observation(yaw_rate_rad_s=0.1)

    first.reset(context)
    first_action = first.act(observation, pointgoal_base_xy=(20.0, 0.0))
    first_diagnostics = first.diagnostics
    first.reset(context)
    reset_action = first.act(observation, pointgoal_base_xy=(20.0, 0.0))
    reset_diagnostics = first.diagnostics

    second.reset(context)
    second.act(observation, pointgoal_base_xy=(20.0, 0.0))

    np.testing.assert_array_equal(first_action, reset_action)
    assert first_diagnostics == reset_diagnostics
    assert first.path_index == second.path_index
    assert first._profile is not second._profile


def test_trajectory_v2_is_not_either_legacy_oracle_and_does_not_share_state():
    context = trajectory_line_context(include_oracle_path=True)
    legacy = OraclePathPolicy(aux30_protocol())
    v2 = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    legacy.reset(context)
    expected = legacy.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))
    expected_index = legacy.path_index
    expected_diagnostics = legacy.diagnostics

    assert not isinstance(v2, OraclePathPolicy)
    assert not isinstance(v2, TrajectoryOraclePolicy)
    v2.reset(context)
    v2.act(valid_observation(position_world_xy=(1.2, 0.1)), (10.0, 0.0))
    v2.reset(context)

    actual = legacy.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))
    fresh_legacy = OraclePathPolicy(aux30_protocol())
    fresh_legacy.reset(context)
    fresh_legacy.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))
    expected_second = fresh_legacy.act(
        valid_observation(), pointgoal_base_xy=(10.0, 0.0)
    )

    np.testing.assert_array_equal(expected, actual)
    np.testing.assert_array_equal(actual, expected_second)
    assert expected_index == 0
    assert legacy.diagnostics == expected_diagnostics


def test_trajectory_v2_forward_and_reverse_curvature_signs_are_drive_model_consistent():
    cases = (
        (_constant_curvature_reference(1.0 / 3.0), 1.0, 1.0 / 3.0),
        (_reverse_quarter_circle_reference(), -1.0, -1.0 / 3.0),
    )
    for reference, drive_sign, expected_profile_sign in cases:
        policy = DelayAwareTrajectoryOraclePolicy(
            aux30_protocol(), profile_window_m=0.0
        )
        policy.reset(
            trajectory_context_from_reference(reference, include_oracle_path=False)
        )
        action = policy.act(valid_observation(), pointgoal_base_xy=(20.0, 0.0))
        diagnostics = policy.diagnostics
        main = float(action[0])
        assert main == pytest.approx(drive_sign * 0.50)
        assert diagnostics["profile_curvature_1_m"] == pytest.approx(
            expected_profile_sign, abs=2e-3
        )
        assert float(
            policy.drive_model.curvature(
                main, float(diagnostics["auxiliary_feedforward_rad"])
            )
        ) == pytest.approx(
            drive_sign * float(diagnostics["profile_curvature_1_m"]), abs=3e-2
        )


def test_trajectory_v2_delay_query_does_not_cross_direction_cusp():
    policy = DelayAwareTrajectoryOraclePolicy(
        aux30_protocol(), profile_window_m=0.0
    )
    context = trajectory_context_from_reference(
        bidirectional_reference(), include_oracle_path=False
    )
    policy.reset(context)

    policy.act(
        valid_observation(position_world_xy=(0.0, 0.0), forward_speed_m_s=0.4),
        pointgoal_base_xy=(-0.3, -0.1),
    )
    diagnostics = policy.diagnostics
    profile = policy._profile
    assert profile is not None
    current_segment = int(profile.segment_id[policy.path_index])
    segment_end = int(np.flatnonzero(profile.segment_id == current_segment)[-1])
    assert diagnostics["query_s_m"] <= profile.s_m[segment_end]
    assert diagnostics["query_s_m"] >= diagnostics["projection_s_m"]
    assert diagnostics["delay_distance_m"] == pytest.approx(0.30)


def test_trajectory_v2_uses_continuous_sparse_segment_projection_and_cusp_clipping():
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 1.5, 0.0),
        KinodynamicWaypoint(2.0, 0.0, 0.0, "forward", 1.5, 0.0),
        KinodynamicWaypoint(2.0, 0.0, 0.0, "reverse", -1.5, 0.0),
        KinodynamicWaypoint(4.0, 0.0, 0.0, "reverse", 0.0, 0.0),
    )
    policy = DelayAwareTrajectoryOraclePolicy(
        aux30_protocol(), profile_window_m=0.30
    )
    policy.reset(trajectory_context_from_reference(reference, include_oracle_path=False))

    policy.act(
        valid_observation(position_world_xy=(1.50, 0.10), forward_speed_m_s=0.4),
        pointgoal_base_xy=(10.0, 0.0),
    )
    sparse = policy.diagnostics
    profile = policy._profile
    assert profile is not None
    assert sparse["projection_s_m"] == pytest.approx(1.50, abs=1e-9)
    assert sparse["query_s_m"] == pytest.approx(1.80, abs=1e-9)
    assert sparse["query_s_m"] - sparse["projection_s_m"] <= 0.30 + 1e-12
    assert policy.path_index == 0

    policy.act(
        valid_observation(position_world_xy=(1.95, 0.0), forward_speed_m_s=0.4),
        pointgoal_base_xy=(10.0, 0.0),
    )
    near_cusp = policy.diagnostics
    forward_end = int(np.flatnonzero(profile.drive_direction == "forward")[-1])
    assert near_cusp["projection_s_m"] == pytest.approx(1.95, abs=1e-9)
    assert near_cusp["query_s_m"] <= float(profile.s_m[forward_end]) + 1e-12
    assert near_cusp["query_s_m"] - near_cusp["projection_s_m"] <= 0.30 + 1e-12
    assert near_cusp["active_drive_direction"] == "forward"
    assert near_cusp["desired_drive_direction"] == "forward"


def test_trajectory_v2_zero_goal_returns_zero_action_and_records_entry():
    policy = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    policy.reset(trajectory_line_context(include_oracle_path=False))

    action = policy.act(
        valid_observation(forward_speed_m_s=0.10),
        pointgoal_base_xy=(0.1, 0.0),
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert policy.diagnostics["braking_state"] == "goal_reached"
    assert float(policy.diagnostics["goal_entry_s"]) == pytest.approx(0.0)
    assert policy.diagnostics["stop_below_0p05_s"] == pytest.approx(-1.0)
    assert policy.diagnostics["hold_complete_s"] == pytest.approx(-1.0)

    for _ in range(4):
        np.testing.assert_array_equal(
            policy.act(valid_observation(), pointgoal_base_xy=(0.1, 0.0)),
            np.zeros(10, dtype=np.float32),
        )
    assert policy.diagnostics["stop_below_0p05_s"] == pytest.approx(0.1)
    assert policy.diagnostics["hold_complete_s"] == pytest.approx(-1.0)

    np.testing.assert_array_equal(
        policy.act(valid_observation(), pointgoal_base_xy=(0.1, 0.0)),
        np.zeros(10, dtype=np.float32),
    )
    assert policy.diagnostics["hold_complete_s"] == pytest.approx(0.5)


def test_trajectory_v2_straight_blend_zero_reduces_command_to_pure_pursuit():
    policy = DelayAwareTrajectoryOraclePolicy(
        aux30_protocol(), profile_window_m=0.0
    )
    policy.reset(trajectory_line_context(include_oracle_path=False))

    policy.act(
        valid_observation(position_world_xy=(0.0, 0.15)),
        pointgoal_base_xy=(10.0, 0.0),
    )
    diagnostics = policy.diagnostics
    assert diagnostics["profile_curvature_1_m"] == pytest.approx(0.0, abs=1e-12)
    assert diagnostics["blend_weight"] == pytest.approx(0.0, abs=1e-12)
    assert diagnostics["command_curvature_1_m"] == pytest.approx(
        diagnostics["pure_pursuit_curvature_1_m"], abs=1e-12
    )


@pytest.mark.parametrize(
    "kwargs",
    (
        {"maximum_delay_distance_m": 0.31},
        {"supported_main_command_rad_s": 0.25},
        {"feedback_lookahead_m": 0.0},
        {"episode_budget_s": float("nan")},
        {"terminal_brake_buffer_m": -0.01},
        {"terminal_brake_buffer_m": float("nan")},
    ),
)
def test_trajectory_v2_rejects_invalid_constructor_parameters(kwargs):
    with pytest.raises(ValueError):
        DelayAwareTrajectoryOraclePolicy(aux30_protocol(), **kwargs)


def test_trajectory_v2_rejects_invalid_reference_before_action_and_diagnostic_corruption():
    valid = trajectory_line_context(include_oracle_path=False)
    policy = DelayAwareTrajectoryOraclePolicy(aux30_protocol())
    policy.reset(valid)
    before = policy.diagnostics
    invalid_reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(float("nan"), 1.0, 0.0, "forward", 0.0, 0.0),
    )
    invalid = replace(valid, kinodynamic_reference=invalid_reference)
    with pytest.raises(ValueError):
        policy.reset(invalid)
    assert policy.diagnostics == before

    policy.profile_window_m = float("nan")
    with pytest.raises(ValueError, match="delay-query|profile_window_m"):
        policy.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))


def test_trajectory_v2_budget_infeasible_is_diagnostic_only():
    context = trajectory_line_context(length_m=4.0, include_oracle_path=False)
    feasible = DelayAwareTrajectoryOraclePolicy(aux30_protocol(), episode_budget_s=35.0)
    infeasible = DelayAwareTrajectoryOraclePolicy(aux30_protocol(), episode_budget_s=0.1)
    feasible.reset(context)
    infeasible.reset(context)

    feasible_action = feasible.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))
    infeasible_action = infeasible.act(valid_observation(), pointgoal_base_xy=(10.0, 0.0))

    np.testing.assert_array_equal(feasible_action, infeasible_action)
    assert feasible.diagnostics["budget_infeasible"] is False
    assert infeasible.diagnostics["budget_infeasible"] is True
    assert math.isfinite(float(infeasible.diagnostics["terminal_runway_s"]))


def test_trajectory_v2_waits_five_stopped_cycles_before_reverse_command():
    policy = DelayAwareTrajectoryOraclePolicy(aux30_protocol(), profile_window_m=0.0)
    policy.reset(
        EpisodeContext(
            episode_id="trajectory-v2-direction-change",
            privileged=True,
            oracle_path_xy=np.asarray(
                ((0.0, 0.0), (0.3, 0.0), (0.6, 0.0), (0.3, -0.1))
            ),
            action_protocol_id="ball_action_v2_aux30",
            kinodynamic_reference=bidirectional_reference(),
        )
    )

    moving = policy.act(
        valid_observation(position_world_xy=(0.6, 0.0), forward_speed_m_s=0.2),
        pointgoal_base_xy=(-0.3, -0.1),
    )
    stopped = [
        policy.act(
            valid_observation(position_world_xy=(0.6, 0.0)),
            pointgoal_base_xy=(-0.3, -0.1),
        )
        for _ in range(5)
    ]

    np.testing.assert_array_equal(moving, np.zeros(10, dtype=np.float32))
    for action in stopped[:4]:
        np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert stopped[4][0] == pytest.approx(-0.50)
    assert policy.diagnostics["active_drive_direction"] == "reverse"
    assert policy.diagnostics["desired_drive_direction"] == "reverse"
