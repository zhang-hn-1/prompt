"""CPU-only contracts for the Round 6 action boundary and online shadow."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

import sys

from benchmark.action_layers import classify_signed_direction, normalize_action_request
from benchmark.boundary_shadow import (
    BoundaryContext,
    M1TransitionPredictor,
    ReadOnlyBoundaryShadowPolicy,
    SupportEnvelope,
)
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.policies import EpisodeContext, OraclePathPolicy, PolicyObservation
from training.action_protocol import ActionProtocol

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import analyze_round6_reference_handoff as round6_analysis  # noqa: E402


def _protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -math.radians(30.0)),
        maximum=(2.0, math.radians(30.0)),
        controls_per_block=5,
    )


def _observation(*, position=(0.0, 0.0), speed=0.0, previous=(0.0, 0.0)):
    proprio = np.zeros((5, 17), dtype=np.float32)
    proprio[-1, 0] = speed
    proprio[-1, 12] = speed
    proprio[-1, 15:17] = previous
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=proprio,
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.asarray(position, dtype=np.float64),
        yaw_world_rad=0.0,
    )


def _context() -> EpisodeContext:
    reference = (
        KinodynamicWaypoint(0.0, 0.0, 0.0, "forward", 0.5, 0.0),
        KinodynamicWaypoint(0.4, 0.0, 0.0, "forward", 0.5, math.radians(-35.0)),
        KinodynamicWaypoint(0.6, -0.1, 0.2, "reverse", -0.5, math.radians(35.0)),
    )
    return EpisodeContext(
        episode_id="round6-boundary",
        privileged=True,
        oracle_path_xy=np.asarray([(p.x_m, p.y_m) for p in reference]),
        action_protocol_id="ball_action_v2_aux30",
        kinodynamic_reference=reference,
    )


def test_raw_protocol_normalization_retains_plus_minus_35_degree_intent():
    positive = normalize_action_request(
        _protocol(), main_velocity=0.5, auxiliary_angle=math.radians(35.0), source="tracking"
    )
    negative = normalize_action_request(
        _protocol(), main_velocity=-0.5, auxiliary_angle=math.radians(-35.0), source="tracking"
    )

    assert positive.raw_component[1] == pytest.approx(math.radians(35.0))
    assert positive.protocol_component[1] == pytest.approx(math.radians(30.0))
    assert negative.raw_component[1] == pytest.approx(math.radians(-35.0))
    assert negative.protocol_component[1] == pytest.approx(math.radians(-30.0))
    assert positive.clipped and negative.clipped
    assert positive.clip_reason == "auxiliary_above_maximum"
    assert negative.clip_reason == "auxiliary_below_minimum"
    np.testing.assert_array_equal(positive.protocol_block, _protocol().clip_block(positive.raw_block))


@pytest.mark.parametrize(
    ("value", "deadband", "expected"),
    [(0.2, 0.05, "forward"), (-0.2, 0.05, "reverse"), (0.01, 0.05, "near_zero"), (math.nan, 0.05, "unknown")],
)
def test_direction_classifier_keeps_near_zero_and_unknown(value, deadband, expected):
    assert classify_signed_direction(value, deadband=deadband) == expected


def test_oracle_early_direction_change_records_zero_request_instead_of_stale_tracking_command():
    policy = OraclePathPolicy(_protocol())
    policy.reset(_context())

    action = policy.act(
        _observation(position=(0.6, -0.1), speed=0.2),
        pointgoal_base_xy=np.asarray((-0.3, -0.1)),
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    diagnostics = policy.request_diagnostics
    assert diagnostics["request_source"] == "direction_change_braking"
    assert diagnostics["policy_raw_main_command_rad_s"] == pytest.approx(0.0)
    assert diagnostics["policy_protocol_main_command_rad_s"] == pytest.approx(0.0)
    assert diagnostics["policy_raw_auxiliary_target_rad"] == pytest.approx(0.0)
    assert diagnostics["policy_request_clipped"] is False


def test_m1_rollout_is_recursive_and_uses_only_supplied_request_plan(tmp_path: Path):
    # State delta is request_main * 0.02 in main speed and otherwise zero.
    coefficients = np.zeros((15, 8), dtype=float)
    coefficients[9, 0] = 0.02
    model = {
        "schema_version": "round5_linear_state_delta_v1",
        "state_names": [
            "main_joint_velocity_rad_s", "auxiliary_joint_rad",
            "auxiliary_joint_velocity_rad_s", "body_forward_velocity_m_s",
            "body_lateral_velocity_m_s", "world_z_angular_velocity_rad_s",
            "final_main_rad_s", "final_auxiliary_rad",
        ],
        "request_names": ["requested_main_rad_s", "requested_auxiliary_rad"],
        "dt_s": 0.02,
        "coefficient_feature_by_state": coefficients.tolist(),
        "causal_contract": {
            "initialized_from_current_measured_state_only": True,
            "future_requested_commands_only": True,
            "future_measured_state_used": False,
            "future_supervisor_output_used": False,
            "truth_reset_inside_window": False,
        },
    }
    path = tmp_path / "m1.json"
    path.write_text(__import__("json").dumps(model), encoding="utf-8")
    predictor = M1TransitionPredictor.from_json(path)
    state = np.zeros(8, dtype=float)

    first = predictor.rollout(state, np.asarray([[0.5, 0.0]] * 3), start_yaw_rad=0.0)
    replaced_future_log = {"future_measured_state": np.full((100, 8), 999.0)}
    second = predictor.rollout(state, np.asarray([[0.5, 0.0]] * 3), start_yaw_rad=0.0)

    assert replaced_future_log  # deliberately inaccessible to the predictor API
    np.testing.assert_array_equal(first.state_trace, second.state_trace)
    assert first.state_trace[-1, 0] == pytest.approx(0.03)


class _CountingOracle(OraclePathPolicy):
    def __init__(self, protocol):
        super().__init__(protocol)
        self.calls = 0

    def act(self, observation, pointgoal_base_xy):
        self.calls += 1
        return super().act(observation, pointgoal_base_xy)


def test_shadow_calls_b0_once_and_returns_exact_action_block():
    direct = OraclePathPolicy(_protocol())
    wrapped_base = _CountingOracle(_protocol())
    direct.reset(_context())
    shadow = ReadOnlyBoundaryShadowPolicy(wrapped_base, predictor=None)
    shadow.reset(_context())
    observation = _observation(position=(0.35, 0.0), speed=0.1)

    expected = direct.act(observation, np.asarray((1.0, 0.0)))
    actual = shadow.act(observation, np.asarray((1.0, 0.0)))

    np.testing.assert_array_equal(actual, expected)
    assert wrapped_base.calls == 1
    assert len(shadow.contexts) == 1
    context = shadow.contexts[0]
    assert isinstance(context, BoundaryContext)
    assert context.policy_action_block_sha256
    assert context.reference_progress_s_m > 0.0
    assert context.value_time_classification["request_direction"] == "current_known"


def test_boundary_context_is_frozen_and_serializes_distinct_directions():
    context = BoundaryContext(
        control_block_index=4,
        control_time_s=0.4,
        measurement_timestamp_s=0.4,
        reference_index=2,
        reference_direction="reverse",
        active_reference_direction="forward",
        pending_reference_direction="reverse",
        reference_progress_s_m=0.6,
        reference_segment_progress_s_m=0.2,
        request_direction="near_zero",
        applied_direction="forward",
        main_joint_direction="reverse",
        body_motion_direction="near_zero",
        policy_raw_component=(0.0, 0.0),
        protocol_request_component=(0.0, 0.0),
        previous_applied_component=(0.1, 0.0),
        smoother_current_action_component=(0.1, 0.0),
        position_world_xy_m=(1.0, 2.0),
        root_yaw_rad=0.2,
        measured_state=(0.0,) * 8,
        braking_state="direction_change",
        stopped_cycles=2,
        request_source="direction_change_braking",
        request_clipped=False,
        request_clip_reason=None,
        policy_action_block_sha256="abc",
    )

    payload = context.to_dict()
    assert payload["reference_direction"] == "reverse"
    assert payload["request_direction"] == "near_zero"
    assert payload["applied_direction"] == "forward"
    with pytest.raises(Exception):
        context.stopped_cycles = 3  # type: ignore[misc]


def test_support_envelope_requires_box_and_nearest_neighbour_gate():
    cloud = np.asarray(
        [np.zeros(10), np.ones(10), np.full(10, 2.0)], dtype=float
    )
    envelope = SupportEnvelope.from_points(cloud, maximum_distance=1.75)

    supported = envelope.check(np.zeros(8), np.zeros(2))
    outside = envelope.check(np.full(8, 3.0), np.zeros(2))

    assert supported["supported"] is True
    assert supported["standardized_nearest_neighbor_distance"] == pytest.approx(0.0)
    assert outside["supported"] is False
    assert outside["inside_feature_box"] is False


def test_support_envelope_checks_every_recursive_rollout_tick():
    cloud = np.vstack((np.full(10, -1.0), np.zeros(10), np.full(10, 1.0)))
    envelope = SupportEnvelope.from_points(cloud, maximum_distance=100.0)
    states = np.zeros((3, 8), dtype=float)
    states[2, 0] = 2.0
    requests = np.zeros((3, 2), dtype=float)

    result = envelope.check_trajectory(states, requests)

    assert result["supported"] is False
    assert result["checked_tick_count"] == 3
    assert result["first_unsupported_tick"] == 2
    assert result["inside_feature_box_all_ticks"] is False


def test_online_shadow_builds_plans_now_and_abstains_when_model_is_rejected(tmp_path: Path):
    coefficients = np.zeros((15, 8), dtype=float)
    model = {
        "schema_version": "round5_linear_state_delta_v1",
        "state_names": list(__import__("benchmark.boundary_shadow", fromlist=["STATE_NAMES"]).STATE_NAMES),
        "request_names": ["requested_main_rad_s", "requested_auxiliary_rad"],
        "dt_s": 0.02,
        "coefficient_feature_by_state": coefficients.tolist(),
        "causal_contract": {
            "initialized_from_current_measured_state_only": True,
            "future_requested_commands_only": True,
            "future_measured_state_used": False,
            "future_supervisor_output_used": False,
            "truth_reset_inside_window": False,
        },
    }
    path = tmp_path / "m1.json"
    path.write_text(__import__("json").dumps(model), encoding="utf-8")
    predictor = M1TransitionPredictor.from_json(path)
    envelope = SupportEnvelope.from_points(
        np.vstack((np.full(10, -2.0), np.zeros(10), np.full(10, 2.0))),
        maximum_distance=100.0,
    )
    base = _CountingOracle(_protocol())
    shadow = ReadOnlyBoundaryShadowPolicy(
        base,
        predictor=predictor,
        support_envelope=envelope,
        model_accepted=False,
        horizon_s=1.5,
    )
    shadow.reset(_context())

    action = shadow.act(
        _observation(position=(0.6, -0.1), speed=0.2),
        np.asarray((-0.3, -0.1)),
    )

    np.testing.assert_array_equal(action, np.zeros(10, dtype=np.float32))
    assert base.calls == 1
    record = shadow.records[-1]
    assert record["boundary_active"] is True
    assert record["advice"] == "abstain_model_rejected"
    assert set(record["plans"]) == {"continue_b0", "handoff_candidate"}
    assert all(plan["request_tick_count"] == 75 for plan in record["plans"].values())
    assert all(plan["input_time_classification"] == "future_planned_now" for plan in record["plans"].values())
    assert all(plan["request_plan_sha256"] for plan in record["plans"].values())
    assert all(len(plan["predicted_state_trace"]) == 76 for plan in record["plans"].values())
    assert all(len(plan["predicted_local_trajectory_xy_m"]) == 76 for plan in record["plans"].values())
    assert record["invalid_reason"] == "model_rejected_by_validation_gate"


def test_emitted_shadow_record_is_invariant_to_future_suffix_replacement_or_truncation(tmp_path: Path):
    coefficients = np.zeros((15, 8), dtype=float)
    model = {
        "schema_version": "round5_linear_state_delta_v1",
        "state_names": list(__import__("benchmark.boundary_shadow", fromlist=["STATE_NAMES"]).STATE_NAMES),
        "request_names": ["requested_main_rad_s", "requested_auxiliary_rad"],
        "dt_s": 0.02,
        "coefficient_feature_by_state": coefficients.tolist(),
        "causal_contract": {
            "initialized_from_current_measured_state_only": True,
            "future_requested_commands_only": True,
            "future_measured_state_used": False,
            "future_supervisor_output_used": False,
            "truth_reset_inside_window": False,
        },
    }
    path = tmp_path / "m1.json"
    path.write_text(__import__("json").dumps(model), encoding="utf-8")
    predictor = M1TransitionPredictor.from_json(path)
    envelope = SupportEnvelope.from_points(
        np.vstack((np.full(10, -10.0), np.zeros(10), np.full(10, 10.0))),
        maximum_distance=100.0,
    )
    wrappers = [
        ReadOnlyBoundaryShadowPolicy(
            _CountingOracle(_protocol()), predictor=predictor,
            support_envelope=envelope, model_accepted=False,
        )
        for _ in range(3)
    ]
    for wrapper in wrappers:
        wrapper.reset(_context())

    current = _observation(position=(0.6, -0.1), speed=0.2)
    current_goal = np.asarray((-0.3, -0.1))
    initial_actions = [wrapper.act(current, current_goal).copy() for wrapper in wrappers]
    before = [
        __import__("json").dumps(
            {"context": wrapper.contexts[0].to_dict(), "record": wrapper.records[0]},
            sort_keys=True,
        )
        for wrapper in wrappers
    ]

    # wrappers[0] is the truncated log. The other two receive different future suffixes.
    wrappers[1].act(_observation(position=(0.7, -0.2), speed=-0.1), np.asarray((-1.0, 0.2)))
    wrappers[2].act(_observation(position=(-4.0, 3.0), speed=1.7), np.asarray((5.0, -2.0)))
    after = [
        __import__("json").dumps(
            {"context": wrapper.contexts[0].to_dict(), "record": wrapper.records[0]},
            sort_keys=True,
        )
        for wrapper in wrappers
    ]

    assert before[0] == before[1] == before[2]
    assert after == before
    assert all(np.array_equal(action, initial_actions[0]) for action in initial_actions)
    assert [wrapper.base_policy.calls for wrapper in wrappers] == [1, 2, 2]


def test_round6_model_gate_rejects_terminal_error_even_with_good_trajectory():
    rows = [
        {
            "sequence_id": f"heldout-{index}",
            "m1_trajectory_rms_error_m": 0.03,
            "m1_terminal_state_error": {
                "main_joint_velocity_rad_s": 0.20,
                "auxiliary_joint_rad": 0.08,
            },
            "main_reversal_required": False,
            "m1_main_zero_crossing_error_s": None,
            "auxiliary_reversal_required": False,
            "m1_auxiliary_zero_crossing_error_s": None,
        }
        for index in range(4)
    ]

    decision = round6_analysis.evaluate_local_gate(rows)

    assert decision["checks"]["maximum_trajectory_rms_error_m_at_1p5s"] is True
    assert decision["checks"]["maximum_terminal_main_error_rad_s"] is False
    assert decision["checks"]["maximum_terminal_auxiliary_error_rad"] is False
    assert decision["accepted"] is False


def test_combined_model_gate_requires_round5_regression_and_round6_local_gate():
    decision = round6_analysis.combined_candidate_decision(
        {"accepted": True}, {"accepted": False}, {"accepted": True}
    )

    assert decision["accepted"] is False
    assert decision["round6_local_gate_passed"] is True
    assert decision["round5_regression_gate_passed"] is False


def test_round5_relative_regression_rejects_expanded_model_degradation():
    frozen = {
        "m1": {
            "endpoint_position_mae_m": 0.05,
            "endpoint_position_p90_m_at_6s": 0.10,
            "root_heading_change_mae_rad": 0.03,
        }
    }
    expanded = {
        "m1": {
            "endpoint_position_mae_m": 0.06,
            "endpoint_position_p90_m_at_6s": 0.09,
            "root_heading_change_mae_rad": 0.03,
        }
    }

    result = round6_analysis.evaluate_round5_relative_regression(frozen, expanded)

    assert result["accepted"] is False
    assert result["checks"]["expanded_no_worse_endpoint_position_mae_m"] is False
