"""CPU contracts for the Round 8 Oracle tracking audit."""

from __future__ import annotations

import inspect
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from benchmark.action_layers import normalize_action_request
from benchmark.drive_model import DriveModel
from benchmark.kinodynamic_planning import KinodynamicWaypoint, sample_motion_primitive
from benchmark.policies import (
    EpisodeContext,
    OraclePathPolicy,
    PolicyObservation,
    Round8ProfileBlendOraclePolicy,
)
from training.action_protocol import ActionProtocol

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from analyze_round8_tracking_audit import project_trace_on_segment  # noqa: E402


PROFILE = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)


def _protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -math.radians(30.0)),
        maximum=(2.0, math.radians(30.0)),
        controls_per_block=5,
    )


def _observation(
    *,
    position: tuple[float, float] = (0.0, 0.0),
    yaw: float = 0.0,
    velocity: tuple[float, float] = (0.2, 0.0),
    yaw_rate: float = 0.0,
    actual_auxiliary: float = 0.0,
) -> PolicyObservation:
    proprio = np.zeros((5, 17), dtype=np.float32)
    proprio[-1, :2] = velocity
    proprio[-1, 5] = yaw_rate
    proprio[-1, 10] = actual_auxiliary
    proprio[-1, 12] = velocity[0]
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray(position, dtype=np.float64),
        yaw_world_rad=yaw,
    )


def _context(reference: tuple[KinodynamicWaypoint, ...]) -> EpisodeContext:
    xy = np.asarray([(point.x_m, point.y_m) for point in reference], dtype=np.float64)
    return EpisodeContext(
        episode_id="round8-cpu",
        privileged=True,
        oracle_path_xy=xy,
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference,
    )


def _straight_reference(direction: str = "forward") -> tuple[KinodynamicWaypoint, ...]:
    sign = 1.0 if direction == "forward" else -1.0
    return (
        KinodynamicWaypoint(0.0, 0.0, 0.0, direction, sign * 0.5, 0.0),
        KinodynamicWaypoint(sign * 0.6, 0.0, 0.0, direction, sign * 0.5, 0.0),
        KinodynamicWaypoint(sign * 1.2, 0.0, 0.0, direction, 0.0, 0.0),
    )


def test_formal_oracle_factory_resolves_to_oracle_path_policy():
    from scripts.run_pointgoal_benchmark import _make_policy

    policy = _make_policy("oracle", _protocol(), seed=7)

    assert type(policy) is OraclePathPolicy
    assert type(policy).mro()[:2] == [OraclePathPolicy, OraclePathPolicy.__base__]
    assert inspect.getattr_static(type(policy), "act") is OraclePathPolicy.act
    assert inspect.getattr_static(type(policy), "_project_to_reference") is (
        OraclePathPolicy._project_to_reference
    )


def test_tracking_audit_is_single_call_aligned_and_reports_real_request_layers():
    policy = OraclePathPolicy(_protocol(), drive_model=DriveModel.from_json(PROFILE))
    policy.reset(_context(_straight_reference()))

    action = policy.act(_observation(), pointgoal_base_xy=(2.0, 0.0))

    assert len(policy.tracking_records) == 1
    record = policy.tracking_records[0]
    assert record["valid"] is True
    assert record["source"] == "tracking"
    assert record["curvature_composition"] == "pure_pursuit_only"
    assert record["profile_curvature_role"] == "diagnostic_only"
    assert record["policy_protocol_main_command_rad_s"] == pytest.approx(action[0])
    assert record["policy_protocol_auxiliary_target_rad"] == pytest.approx(action[1])
    assert record["actual_auxiliary_angle_rad"] == pytest.approx(0.0)
    assert record["velocity_course_valid"] is True


def test_early_return_invalidates_geometry_instead_of_reusing_stale_values():
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.4, 0.0, 0.0, "reverse", -0.5, 0.0),
        KinodynamicWaypoint(0.0, 0.0, 0.0, "reverse", 0.0, 0.0),
    )
    policy = OraclePathPolicy(_protocol())
    policy.reset(_context(reference))
    policy.act(_observation(position=(0.0, 0.0)), pointgoal_base_xy=(2.0, 0.0))
    policy.act(
        _observation(position=(0.4, 0.0), velocity=(0.2, 0.0)),
        pointgoal_base_xy=(-1.0, 0.0),
    )

    record = policy.tracking_records[-1]
    assert record["valid"] is False
    assert record["source"] == "direction_change_braking"
    assert "requested_curvature_1_m" not in record
    assert record["policy_protocol_auxiliary_target_rad"] == pytest.approx(0.0)


@pytest.mark.parametrize("direction", ("forward", "reverse"))
def test_straight_ideal_tracking_has_zero_geometric_turn(direction: str):
    policy = OraclePathPolicy(_protocol(), yaw_rate_feedback_gain_s=0.0)
    policy.reset(_context(_straight_reference(direction)))

    policy.act(
        _observation(velocity=((0.2 if direction == "forward" else -0.2), 0.0)),
        pointgoal_base_xy=((2.0 if direction == "forward" else -2.0), 0.0),
    )

    record = policy.tracking_records[-1]
    assert record["geometric_pure_pursuit_curvature_1_m"] == pytest.approx(0.0)


def test_forward_circle_pure_pursuit_recovers_ideal_curvature():
    curvature = 0.5
    poses = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=1.2,
        curvature_1_m=curvature,
        sample_spacing_m=0.6,
    )
    reference = tuple(
        KinodynamicWaypoint(
            float(x), float(y), float(yaw), "forward", 0.5 if i < 2 else 0.0, 0.0
        )
        for i, (x, y, yaw) in enumerate(poses)
    )
    policy = OraclePathPolicy(_protocol(), lookahead_m=0.5, yaw_rate_feedback_gain_s=0.0)
    policy.reset(_context(reference))

    policy.act(_observation(), pointgoal_base_xy=(2.0, 0.0))

    assert policy.tracking_records[-1][
        "geometric_pure_pursuit_curvature_1_m"
    ] == pytest.approx(curvature, abs=2e-3)


@pytest.mark.parametrize(
    ("position_y", "yaw", "expected_sign"),
    ((0.1, 0.0, -1.0), (-0.1, 0.0, 1.0), (0.0, 0.1, -1.0), (0.0, -0.1, 1.0)),
)
def test_forward_small_position_and_heading_errors_correct_toward_line(
    position_y: float, yaw: float, expected_sign: float
):
    policy = OraclePathPolicy(_protocol(), yaw_rate_feedback_gain_s=0.0)
    policy.reset(_context(_straight_reference("forward")))

    policy.act(
        _observation(position=(0.0, position_y), yaw=yaw),
        pointgoal_base_xy=(2.0, 0.0),
    )

    curvature = float(policy.tracking_records[-1]["requested_curvature_1_m"])
    assert math.copysign(1.0, curvature) == expected_sign


@pytest.mark.xfail(
    strict=True,
    reason="frozen Oracle omits the reverse direction sign in Pure Pursuit curvature",
)
def test_reverse_circle_uses_waypoint_order_curvature_sign():
    poses = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=-1.2,
        curvature_1_m=0.5,
        sample_spacing_m=0.6,
    )
    reference = tuple(
        KinodynamicWaypoint(
            float(x), float(y), float(yaw), "reverse", -0.5 if i < 2 else 0.0, 0.0
        )
        for i, (x, y, yaw) in enumerate(poses)
    )
    policy = OraclePathPolicy(_protocol(), lookahead_m=0.5, yaw_rate_feedback_gain_s=0.0)
    policy.reset(_context(reference))

    policy.act(
        _observation(velocity=(-0.2, 0.0)), pointgoal_base_xy=(-2.0, 0.0)
    )

    # Waypoint-order yaw decreases by 0.5 rad per travelled metre.  The
    # frozen baseline currently returns +0.5 because it omits the direction
    # conversion used by the newer trajectory trackers.
    assert policy.tracking_records[-1]["requested_curvature_1_m"] == pytest.approx(
        -0.5, abs=2e-3
    )


def test_circle_curvature_is_stable_under_same_direction_waypoint_resampling():
    values = []
    for spacing in (0.30, 0.15):
        poses = sample_motion_primitive(
            start_pose=(0.0, 0.0, 0.0),
            signed_arc_length_m=1.2,
            curvature_1_m=0.5,
            sample_spacing_m=spacing,
        )
        reference = tuple(
            KinodynamicWaypoint(
                float(x), float(y), float(yaw), "forward", 0.5 if i < len(poses) - 1 else 0.0, 0.0
            )
            for i, (x, y, yaw) in enumerate(poses)
        )
        policy = OraclePathPolicy(_protocol(), lookahead_m=0.5, yaw_rate_feedback_gain_s=0.0)
        policy.reset(_context(reference))
        policy.act(_observation(), pointgoal_base_xy=(2.0, 0.0))
        values.append(float(policy.tracking_records[-1]["requested_curvature_1_m"]))

    np.testing.assert_allclose(values, (0.5, 0.5), atol=2e-3)


def test_v1_forward_inverse_is_self_consistent_inside_each_signed_support():
    model = DriveModel.from_json(PROFILE)
    for main in (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5):
        samples = [row for row in model.steering_samples if row.main_command_rad_s == main]
        support = sorted(row.curvature_1_m for row in samples)
        for curvature in np.linspace(support[1], support[-2], 9):
            auxiliary = model.auxiliary_for_curvature(main, float(curvature))
            reconstructed = model.curvature(main, auxiliary)
            assert reconstructed == pytest.approx(curvature, abs=2e-9)


def test_early_auxiliary_clip_is_behaviorally_equivalent_when_protocol_is_unchanged():
    raw = normalize_action_request(
        _protocol(),
        main_velocity=0.5,
        auxiliary_angle=math.radians(35.0),
        source="tracking",
    )
    preclipped = normalize_action_request(
        _protocol(),
        main_velocity=0.5,
        auxiliary_angle=math.radians(30.0),
        source="tracking",
    )

    np.testing.assert_array_equal(raw.protocol_block, preclipped.protocol_block)
    assert raw.clipped is True
    assert preclipped.clipped is False


def test_round8_candidate_keeps_b0_prefix_and_activates_only_after_handoff_progress():
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "reverse", -0.5, 0.0),
        KinodynamicWaypoint(-0.3, 0.0, 0.0, "reverse", -0.5, 0.0),
        KinodynamicWaypoint(-0.6, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(-0.3, 0.1, 0.3, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.0, 0.3, 0.5, "forward", 0.0, 0.0),
    )
    baseline = OraclePathPolicy(_protocol())
    candidate = Round8ProfileBlendOraclePolicy(_protocol())
    context = _context(reference)
    baseline.reset(context)
    candidate.reset(context)

    prefix = [
        _observation(position=(0.0, 0.0), velocity=(-0.2, 0.0)),
        _observation(position=(-0.6, 0.0), velocity=(-0.2, 0.0)),
        *[_observation(position=(-0.6, 0.0), velocity=(0.0, 0.0)) for _ in range(5)],
    ]
    for observation in prefix:
        expected = baseline.act(observation, pointgoal_base_xy=(2.0, 0.0))
        actual = candidate.act(observation, pointgoal_base_xy=(2.0, 0.0))
        np.testing.assert_array_equal(actual, expected)
        assert candidate.tracking_records[-1].get("candidate_active", False) is False

    # The switch is complete at index 2, but the candidate remains dormant
    # until projection advances on the new same-direction segment.
    expected = baseline.act(
        _observation(position=(-0.3, 0.1), yaw=0.3), pointgoal_base_xy=(2.0, 0.0)
    )
    actual = candidate.act(
        _observation(position=(-0.3, 0.1), yaw=0.3), pointgoal_base_xy=(2.0, 0.0)
    )

    assert candidate.tracking_records[-1]["candidate_active"] is True
    assert not np.array_equal(actual, expected)


def test_round8_candidate_does_not_change_five_sample_rebound_semantics():
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.3, 0.0, 0.0, "reverse", -0.5, 0.0),
        KinodynamicWaypoint(0.0, 0.0, 0.0, "reverse", 0.0, 0.0),
    )
    policy = Round8ProfileBlendOraclePolicy(_protocol())
    policy.reset(_context(reference))
    observations = [
        _observation(position=(0.3, 0.0), velocity=(0.0, 0.0)),
        _observation(position=(0.3, 0.0), velocity=(0.0, 0.0)),
        _observation(position=(0.3, 0.0), velocity=(0.06, 0.0)),
        *[_observation(position=(0.3, 0.0), velocity=(0.0, 0.0)) for _ in range(4)],
    ]
    actions = [policy.act(item, pointgoal_base_xy=(-2.0, 0.0)) for item in observations]

    assert all(np.array_equal(action, np.zeros(10, dtype=np.float32)) for action in actions)
    assert policy.diagnostics["active_drive_direction"] == "forward"
    assert policy.diagnostics["stopped_cycles"] == 4


def test_independent_evaluator_uses_frozen_segment_and_preserves_real_regress():
    reference = np.asarray(((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)))
    positions = np.asarray(((0.2, 0.1), (1.2, -0.2), (0.9, 0.3), (1.8, 0.0)))

    result = project_trace_on_segment(
        positions,
        reference,
        segment_start=0,
        segment_end=3,
    )

    np.testing.assert_allclose(result["progress_s_m"], (0.2, 1.2, 0.9, 1.8))
    np.testing.assert_allclose(result["signed_cross_track_m"], (0.1, -0.2, 0.3, 0.0))
    assert np.any(np.diff(result["progress_s_m"]) < 0.0)
