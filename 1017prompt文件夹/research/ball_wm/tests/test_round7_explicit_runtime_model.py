"""Contracts for Round 7 explicit execution state, events, and M2 rollout."""

from __future__ import annotations

import math

import numpy as np
import pytest

from benchmark.explicit_runtime_model import (
    ExplicitRuntimeChain,
    M2PhysicalPredictor,
    StopReadyCounter,
    classify_event_pair,
    matched_request_prefix_ticks,
)
from scripts.analyze_round7_stop_events_and_m2 import _event_gate_summary
from scripts.analyze_round7_reference_failure_stages import (
    _project_to_active_direction_run,
    _sustained_growth_index,
    _sustained_loss_index,
)
from scripts.analyze_round7_legacy_error_layers import _transition_group


def _chain() -> ExplicitRuntimeChain:
    return ExplicitRuntimeChain(
        dt_s=0.02,
        action_minimum=(-2.0, -math.radians(30.0)),
        action_maximum=(2.0, math.radians(30.0)),
        max_main_acceleration_rad_s2=0.5,
        max_auxiliary_rate_rad_s=math.radians(30.0),
        max_auxiliary_velocity_rad_s=0.30,
    )


def test_execution_chain_clone_has_independent_mutable_state():
    chain = _chain()
    chain.reset(applied_action=(-0.5, 0.2), last_smoothed_target=(-0.5, 0.3), tick_phase=2)
    clone = chain.clone()

    clone.step((0.5, -0.4), auxiliary_position_rad=0.1, auxiliary_velocity_rad_s=0.0)

    np.testing.assert_array_equal(chain.snapshot().current_applied, (-0.5, 0.2))
    np.testing.assert_array_equal(chain.snapshot().last_smoothed_target, (-0.5, 0.3))
    assert chain.snapshot().tick_phase == 2
    assert not np.array_equal(clone.snapshot().current_applied, chain.snapshot().current_applied)


def test_execution_chain_preserves_smoothed_and_supervisor_applied_layers():
    chain = _chain()
    chain.reset(applied_action=(0.0, 0.0))

    step = chain.step(
        (0.5, math.radians(30.0)),
        auxiliary_position_rad=0.07,
        auxiliary_velocity_rad_s=0.31,
    )

    assert step.smoothed_target[0] == pytest.approx(0.01)
    assert step.smoothed_target[1] == pytest.approx(math.radians(0.6))
    assert step.applied_target[0] == pytest.approx(0.01)
    assert step.applied_target[1] == pytest.approx(0.07)
    assert step.supervisor_active is True


def test_stop_ready_uses_strict_threshold_resets_and_fifth_sample():
    counter = StopReadyCounter(threshold_m_s=0.05, required_samples=5, initial_count=0)
    assert counter.update(0.049) is False
    assert counter.update(0.049) is False
    assert counter.update(0.050) is False
    assert counter.count == 0
    for _ in range(4):
        assert counter.update(0.0) is False
    assert counter.update(0.0) is True
    assert counter.count == 5


@pytest.mark.parametrize(
    ("actual", "predicted", "applicable", "expected"),
    [
        (None, None, False, "not_applicable"),
        (None, None, True, "both_not_observed"),
        (1.0, None, True, "missed_event"),
        (None, 1.0, True, "false_positive"),
        (1.0, 1.1, True, "correct_detection"),
    ],
)
def test_event_pair_classification_distinguishes_censoring_and_errors(
    actual, predicted, applicable, expected
):
    result = classify_event_pair(
        actual_time_s=actual,
        predicted_time_s=predicted,
        applicable=applicable,
        horizon_s=1.5,
        available_duration_s=6.0,
    )
    assert result["comparison"] == expected


def test_event_pair_reports_invalid_measurement_when_window_is_truncated():
    result = classify_event_pair(
        actual_time_s=None,
        predicted_time_s=None,
        applicable=True,
        horizon_s=3.0,
        available_duration_s=1.0,
    )
    assert result["actual_status"] == "invalid_measurement"
    assert result["comparison"] == "invalid_measurement"


def test_matched_prefix_stops_before_first_request_divergence():
    planned = np.asarray([[0.0, 0.0]] * 5 + [[0.5, 0.0]] * 3)
    actual = np.asarray([[0.0, 0.0]] * 6 + [[-0.5, 0.0]] * 2)
    assert matched_request_prefix_ticks(planned, actual) == 5


def test_m2_rollout_uses_explicit_chain_without_future_measurements():
    coefficients = np.zeros((13, 6), dtype=float)
    coefficients[7, 0] = 0.1  # applied main -> next measured main delta
    model = {
        "schema_version": "round7_m2_explicit_runtime_physical_delta_v1",
        "physical_state_names": [
            "main_joint_velocity_rad_s", "auxiliary_joint_rad",
            "auxiliary_joint_velocity_rad_s", "body_forward_velocity_m_s",
            "body_lateral_velocity_m_s", "world_z_angular_velocity_rad_s",
        ],
        "feature_count": 13,
        "coefficient_feature_by_physical_state": coefficients.tolist(),
        "dt_s": 0.02,
        "causal_contract": {
            "future_measured_state_used": False,
            "future_applied_action_used": False,
            "future_supervisor_output_used": False,
        },
    }
    predictor = M2PhysicalPredictor(model, _chain())
    snapshot = _chain().snapshot()
    requests = np.asarray([[0.5, 0.0]] * 3)

    rollout = predictor.rollout(
        current_physical_state=np.zeros(6),
        execution_snapshot=snapshot,
        future_protocol_requests=requests,
        start_yaw_rad=0.0,
    )

    assert rollout.physical_state_trace.shape == (4, 6)
    assert rollout.applied_target_trace.shape == (3, 2)
    assert rollout.physical_state_trace[-1, 0] == pytest.approx(0.006)


def test_m2_prediction_prefix_is_unchanged_by_future_replacement_or_truncation():
    coefficients = np.zeros((13, 6), dtype=float)
    coefficients[7, 0] = 0.1
    model = {
        "schema_version": "round7_m2_explicit_runtime_physical_delta_v1",
        "physical_state_names": [
            "main_joint_velocity_rad_s", "auxiliary_joint_rad",
            "auxiliary_joint_velocity_rad_s", "body_forward_velocity_m_s",
            "body_lateral_velocity_m_s", "world_z_angular_velocity_rad_s",
        ],
        "feature_count": 13,
        "coefficient_feature_by_physical_state": coefficients.tolist(),
        "dt_s": 0.02,
        "causal_contract": {
            "future_measured_state_used": False,
            "future_applied_action_used": False,
            "future_supervisor_output_used": False,
        },
    }
    chain = _chain()
    predictor = M2PhysicalPredictor(model, chain)
    common = np.asarray([[0.5, 0.0]] * 4)
    plan_a = np.vstack((common, [[0.5, 0.2]] * 3))
    plan_b = np.vstack((common, [[-0.5, -0.2]] * 3))

    truncated = predictor.rollout(
        current_physical_state=np.zeros(6),
        execution_snapshot=chain.snapshot(),
        future_protocol_requests=common,
        start_yaw_rad=0.0,
    )
    rollout_a = predictor.rollout(
        current_physical_state=np.zeros(6),
        execution_snapshot=chain.snapshot(),
        future_protocol_requests=plan_a,
        start_yaw_rad=0.0,
    )
    rollout_b = predictor.rollout(
        current_physical_state=np.zeros(6),
        execution_snapshot=chain.snapshot(),
        future_protocol_requests=plan_b,
        start_yaw_rad=0.0,
    )

    np.testing.assert_array_equal(rollout_a.physical_state_trace[:5], truncated.physical_state_trace)
    np.testing.assert_array_equal(rollout_b.physical_state_trace[:5], truncated.physical_state_trace)
    np.testing.assert_array_equal(rollout_a.applied_target_trace[:4], truncated.applied_target_trace)
    np.testing.assert_array_equal(rollout_b.applied_target_trace[:4], truncated.applied_target_trace)


def test_event_gate_applies_threshold_to_mean_absolute_error():
    rows = [
        {
            "actual_status": "observed_within_horizon",
            "comparison": "correct_detection",
            "timing_error_s": error,
        }
        for error in (0.14, 0.24)
    ]

    passed, summary = _event_gate_summary(rows, minimum=2, maximum_mae=0.20)

    assert passed is True
    assert summary["mae_s"] == pytest.approx(0.19)
    assert summary["maximum_error_s"] == pytest.approx(0.24)


def test_reference_projection_is_constrained_to_active_direction_run():
    reference = np.asarray(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 0.1, math.pi]]
    )
    directions = np.asarray([-1, -1, 1, 1])

    progress, cross_track, _ = _project_to_active_direction_run(
        np.asarray([1.1, 0.08]), reference, directions, reference_index=1, active_direction=-1
    )

    assert progress == pytest.approx(1.0)
    assert cross_track == pytest.approx(math.hypot(0.1, 0.08))


def test_sustained_loss_requires_ten_consecutive_samples():
    values = np.asarray([0.6] * 9 + [0.1] + [0.7] * 10)
    assert _sustained_loss_index(values) == 10


def test_sustained_growth_requires_net_growth_and_eight_non_decreasing_steps():
    values = np.asarray([0.2] * 10 + [0.10, 0.12, 0.14, 0.13, 0.16, 0.18, 0.19, 0.20, 0.22, 0.23])
    assert _sustained_growth_index(values) == 10


@pytest.mark.parametrize(
    ("initial", "target", "expected"),
    [
        ((-0.5, -0.2), (0.0, 0.0), "braking_to_zero"),
        ((-0.5, -0.2), (0.5, 0.2), "joint_main_and_auxiliary_reversal"),
        ((-0.5, 0.2), (0.5, 0.2), "main_reversal"),
        ((0.5, -0.2), (0.5, 0.2), "auxiliary_reversal"),
        ((0.5, 0.2), (0.5, 0.3), "same_direction_tracking"),
    ],
)
def test_transition_group_is_frozen_from_initial_state_and_target(initial, target, expected):
    state = np.zeros(8)
    state[:2] = initial
    assert _transition_group(state, np.asarray(target)) == expected
