"""CPU-only contract tests for Round 5 motion measurement and transitions."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "research" / "ball_wm" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import analyze_round5_motion_transitions as round5  # noqa: E402
import probe_round5_transitions as round5_probe  # noqa: E402


PROTOCOL = (
    ROOT
    / "research"
    / "ball_wm"
    / "configs"
    / "benchmark_v1"
    / "round5_measurement_protocol.json"
)
MANIFEST = (
    ROOT
    / "research"
    / "ball_wm"
    / "configs"
    / "benchmark_v1"
    / "round5_transition_manifest.json"
)


def _yaw_quaternion(yaw: np.ndarray) -> np.ndarray:
    return np.column_stack(
        (np.cos(0.5 * yaw), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(0.5 * yaw))
    )


def _circle(*, dt: float, reverse: bool = False, rotation: float = 0.0):
    radius = 2.0
    angular_rate = 0.4
    time = np.arange(0.0, 8.0 + 0.5 * dt, dt)
    theta = angular_rate * time
    position = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
    velocity = np.column_stack(
        (-radius * angular_rate * np.sin(theta), radius * angular_rate * np.cos(theta))
    )
    yaw = theta + math.pi / 2.0
    world_z = np.full(time.shape, angular_rate)
    if reverse:
        time = time.copy()
        position = position[::-1].copy()
        velocity = -velocity[::-1].copy()
        yaw = np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))
        world_z = np.full(time.shape, -angular_rate)
    c, s = math.cos(rotation), math.sin(rotation)
    matrix = np.asarray([[c, -s], [s, c]])
    position = position @ matrix.T + np.asarray([4.0, -3.0])
    velocity = velocity @ matrix.T
    yaw = yaw + rotation
    angular = np.column_stack((np.zeros_like(world_z), np.zeros_like(world_z), world_z))
    return time, position, velocity, yaw, _yaw_quaternion(yaw), angular


@pytest.mark.parametrize("dt", [0.02, 0.05])
def test_motion_measurement_recovers_circle_sign_units_and_sampling(dt: float):
    time, position, velocity, yaw, quaternion, angular = _circle(dt=dt)

    result = round5.measure_motion_quantities(
        time_s=time,
        position_xy_m=position,
        linear_velocity_world_xy_m_s=velocity,
        root_quaternion_world_wxyz=quaternion,
        angular_velocity_world_rad_s=angular,
        minimum_planar_speed_m_s=0.05,
    )

    valid = result["course_valid"]
    assert np.median(result["yaw_derivative_rad_s"][2:-2]) == pytest.approx(0.4, abs=2e-3)
    assert np.median(result["world_z_angular_velocity_rad_s"]) == pytest.approx(0.4)
    assert np.median(result["course_rate_rad_s"][valid][2:-2]) == pytest.approx(0.4, abs=2e-3)
    assert np.median(result["kinematic_curvature_1_m"][valid][2:-2]) == pytest.approx(0.5, abs=4e-3)
    assert np.median(result["three_point_curvature_1_m"][2:-2]) == pytest.approx(0.5, abs=4e-3)


def test_geometric_curvature_flips_for_reverse_traversal_and_is_rigid_transform_invariant():
    forward = _circle(dt=0.02, reverse=False, rotation=0.0)
    reverse = _circle(dt=0.02, reverse=True, rotation=0.8)
    forward_result = round5.measure_motion_quantities(
        time_s=forward[0],
        position_xy_m=forward[1],
        linear_velocity_world_xy_m_s=forward[2],
        root_quaternion_world_wxyz=forward[4],
        angular_velocity_world_rad_s=forward[5],
        minimum_planar_speed_m_s=0.05,
    )
    reverse_result = round5.measure_motion_quantities(
        time_s=reverse[0],
        position_xy_m=reverse[1],
        linear_velocity_world_xy_m_s=reverse[2],
        root_quaternion_world_wxyz=reverse[4],
        angular_velocity_world_rad_s=reverse[5],
        minimum_planar_speed_m_s=0.05,
    )

    assert np.nanmedian(forward_result["three_point_curvature_1_m"]) == pytest.approx(0.5, abs=4e-3)
    assert np.nanmedian(reverse_result["three_point_curvature_1_m"]) == pytest.approx(-0.5, abs=4e-3)
    assert np.nanmedian(reverse_result["kinematic_curvature_1_m"]) == pytest.approx(-0.5, abs=4e-3)


def test_left_right_straight_and_independent_root_yaw_are_distinguished():
    time, position, velocity, yaw, quaternion, angular = _circle(dt=0.02)
    right_position = position.copy()
    right_position[:, 1] *= -1.0
    right_velocity = velocity.copy()
    right_velocity[:, 1] *= -1.0
    right_yaw = -yaw
    right_angular = angular.copy()
    right_angular[:, 2] *= -1.0
    right = round5.measure_motion_quantities(
        time_s=time,
        position_xy_m=right_position,
        linear_velocity_world_xy_m_s=right_velocity,
        root_quaternion_world_wxyz=_yaw_quaternion(right_yaw),
        angular_velocity_world_rad_s=right_angular,
        minimum_planar_speed_m_s=0.05,
    )
    straight_position = np.column_stack((time, np.zeros_like(time)))
    yawing = round5.measure_motion_quantities(
        time_s=time,
        position_xy_m=straight_position,
        linear_velocity_world_xy_m_s=np.column_stack((np.ones_like(time), np.zeros_like(time))),
        root_quaternion_world_wxyz=_yaw_quaternion(0.2 * time),
        angular_velocity_world_rad_s=np.column_stack(
            (np.zeros_like(time), np.zeros_like(time), np.full_like(time, 0.2))
        ),
        minimum_planar_speed_m_s=0.05,
    )

    assert np.nanmedian(right["three_point_curvature_1_m"]) == pytest.approx(-0.5, abs=4e-3)
    assert np.nanmedian(yawing["three_point_curvature_1_m"]) == pytest.approx(0.0, abs=1e-9)
    assert np.nanmedian(yawing["course_rate_rad_s"]) == pytest.approx(0.0, abs=1e-9)
    assert np.median(yawing["yaw_derivative_rad_s"]) == pytest.approx(0.2, abs=2e-3)


def test_low_speed_course_and_curvature_are_invalid_not_zero():
    time = np.arange(6, dtype=float) * 0.02
    result = round5.measure_motion_quantities(
        time_s=time,
        position_xy_m=np.zeros((6, 2)),
        linear_velocity_world_xy_m_s=np.zeros((6, 2)),
        root_quaternion_world_wxyz=_yaw_quaternion(np.zeros(6)),
        angular_velocity_world_rad_s=np.zeros((6, 3)),
        minimum_planar_speed_m_s=0.05,
    )

    assert not np.any(result["course_valid"])
    assert np.isnan(result["course_rad"]).all()
    assert np.isnan(result["course_rate_rad_s"]).all()
    assert np.isnan(result["kinematic_curvature_1_m"]).all()
    assert np.isnan(result["three_point_curvature_1_m"]).all()


def test_target_reached_stationary_geometry_oscillation_and_transition_are_independent():
    protocol = round5.load_measurement_protocol(PROTOCOL)
    time, position, velocity, yaw, quaternion, angular = _circle(dt=0.02)
    count = len(time)
    samples = {
        "time_s": time,
        "phase": np.asarray(["steer"] * count),
        "requested_auxiliary_rad": np.full(count, math.radians(30.0)),
        "auxiliary_joint_rad": np.full(count, math.radians(25.0)),
        "auxiliary_joint_velocity_rad_s": np.zeros(count),
        "position_xy_m": position,
        "linear_velocity_world_xy_m_s": velocity,
        "root_quaternion_world_wxyz": quaternion,
        "angular_velocity_world_rad_s": angular,
    }

    result = round5.classify_measurement_window(samples, protocol)

    assert result["target_reached"] is False
    assert result["state_stationary"] is True
    assert result["geometry_identifiable"] is True
    assert result["transition_valid"] is True
    assert isinstance(result["oscillatory"], bool)


def test_t90_keeps_requested_and_observed_final_definitions_separate():
    result = round5.measure_t90(
        time_s=np.arange(7, dtype=float),
        requested_target=np.asarray([0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        final_command=np.asarray([0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.0]),
        measured=np.asarray([0.0, 0.1, 0.3, 0.5, 0.65, 0.75, 0.8]),
        fraction=0.9,
    )

    assert result["measured_t90_requested_target_from_request_s"] is None
    assert result["measured_t90_observed_final_from_request_s"] == pytest.approx(4.0)
    assert result["final_command_t90_requested_target_from_request_s"] == pytest.approx(4.0)
    assert result["request_change_time_s"] == pytest.approx(1.0)
    assert result["final_command_change_time_s"] == pytest.approx(1.0)


def test_phase_step_window_excludes_later_return_to_zero():
    samples = [
        {"time_s": 0.0, "phase": "approach", "requested_auxiliary_rad": 0.0},
        {"time_s": 1.0, "phase": "steer", "requested_auxiliary_rad": 0.5},
        {"time_s": 2.0, "phase": "steer", "requested_auxiliary_rad": 0.5},
        {"time_s": 3.0, "phase": "return", "requested_auxiliary_rad": 0.0},
    ]

    selected = round5.phase_step_samples(samples, "steer")

    assert [sample["time_s"] for sample in selected] == [0.0, 1.0, 2.0]
    assert selected[-1]["requested_auxiliary_rad"] == pytest.approx(0.5)


def test_manifest_uses_whole_sequence_splits_and_stays_within_caps():
    protocol = round5.load_measurement_protocol(PROTOCOL)
    manifest = round5_probe.load_transition_manifest(MANIFEST, protocol)
    roles = [sequence.dataset_role for sequence in manifest.sequences]

    assert roles.count("fit_core") == 12
    assert roles.count("heldout_validation") == 4
    assert roles.count("repeat_validation") == 2
    assert len({sequence.sequence_id for sequence in manifest.sequences}) == 18
    assert all(sequence.stages[-1].duration_s >= 6.0 for sequence in manifest.sequences)


def test_schedule_resets_only_at_sequence_start_and_preserves_smoother_at_switch():
    protocol = round5.load_measurement_protocol(PROTOCOL)
    manifest = round5_probe.load_transition_manifest(MANIFEST, protocol)
    sequence = next(item for item in manifest.sequences if item.sequence_id == "core_main_reverse_to_forward_a0")

    schedule = round5_probe.build_sequence_schedule(sequence, physics_hz=50)
    transition_start = next(index for index, row in enumerate(schedule) if row["phase"] == "transition")

    assert schedule[0]["reset_flag"] is True
    assert sum(bool(row["reset_flag"]) for row in schedule) == 1
    assert schedule[transition_start]["reset_flag"] is False
    assert schedule[transition_start]["smoother_reset_flag"] is False
    assert schedule[transition_start - 1]["main_request_rad_s"] == pytest.approx(-0.5)
    assert schedule[transition_start]["main_request_rad_s"] == pytest.approx(0.5)


def test_transition_windows_use_last_precondition_state_and_fixed_start_frame():
    protocol = round5.load_measurement_protocol(PROTOCOL)
    time = np.arange(0.0, 7.02, 0.02)
    transition = time >= 1.0
    x = np.where(transition, time - 1.0, 0.0)
    y = np.zeros_like(x)
    yaw = np.full_like(x, math.pi / 2.0)
    samples = []
    for index, stamp in enumerate(time):
        samples.append(
            {
                "time_s": float(stamp),
                "phase": "transition" if transition[index] else "precondition",
                "requested_main_rad_s": 0.5 if transition[index] else 0.0,
                "requested_auxiliary_rad": 0.0,
                "final_main_rad_s": 0.5 if transition[index] else 0.0,
                "final_auxiliary_rad": 0.0,
                "main_joint_velocity_rad_s": 0.5 if transition[index] else 0.0,
                "auxiliary_joint_rad": 0.0,
                "auxiliary_joint_velocity_rad_s": 0.0,
                "navigation_heading_rad": float(yaw[index]),
                "root_quaternion_world_wxyz": _yaw_quaternion(yaw[index:index+1])[0].tolist(),
                "position_xy_m": [float(x[index]), float(y[index])],
                "linear_velocity_world_xy_m_s": [1.0 if transition[index] else 0.0, 0.0],
                "angular_velocity_world_rad_s": [0.0, 0.0, 0.0],
                "auxiliary_velocity_supervisor_active": False,
            }
        )
    trace = {"sequence":{"sequence_id":"synthetic","dataset_role":"fit_core"}, "samples":samples}

    windows = round5.extract_transition_windows(trace, protocol)
    at_half = next(row for row in windows if row["horizon_s"] == 0.5)

    assert at_half["start_sample_index"] == 49
    assert at_half["world_delta_xy_m"] == pytest.approx([0.5, 0.0], abs=0.03)
    assert at_half["local_delta_xy_m"] == pytest.approx([0.0, -0.5], abs=0.03)
    assert at_half["start_navigation_heading_rad"] == pytest.approx(math.pi / 2.0)


def test_transition_course_change_is_invalid_if_window_crosses_low_speed():
    protocol = round5.load_measurement_protocol(PROTOCOL)
    time = np.arange(0.0, 7.02, 0.02)
    transition = time >= 1.0
    velocity_x = np.where(time < 2.0, -0.2, np.where(time < 2.4, 0.0, 0.2))
    position_x = np.cumsum(velocity_x) * 0.02
    samples = []
    for index, stamp in enumerate(time):
        samples.append(
            {
                "time_s": float(stamp),
                "phase": "transition" if transition[index] else "precondition",
                "requested_main_rad_s": 0.5 if transition[index] else -0.5,
                "requested_auxiliary_rad": 0.0,
                "final_main_rad_s": 0.5 if transition[index] else -0.5,
                "final_auxiliary_rad": 0.0,
                "main_joint_velocity_rad_s": float(velocity_x[index]),
                "auxiliary_joint_rad": 0.0,
                "auxiliary_joint_velocity_rad_s": 0.0,
                "navigation_heading_rad": 0.0,
                "root_quaternion_world_wxyz": [1.0, 0.0, 0.0, 0.0],
                "position_xy_m": [float(position_x[index]), 0.0],
                "linear_velocity_world_xy_m_s": [float(velocity_x[index]), 0.0],
                "angular_velocity_world_rad_s": [0.0, 0.0, 0.0],
                "auxiliary_velocity_supervisor_active": False,
            }
        )
    trace = {"sequence": {"sequence_id": "reversal", "dataset_role": "fit_core"}, "samples": samples}

    windows = round5.extract_transition_windows(trace, protocol)

    assert next(row for row in windows if row["horizon_s"] == 0.5)["course_change_valid"] is True
    assert next(row for row in windows if row["horizon_s"] == 1.5)["course_change_valid"] is False
    assert next(row for row in windows if row["horizon_s"] == 3.0)["course_change_valid"] is False


def test_trace_samples_normalize_collector_request_field_names_without_mutating_input():
    source_sample = {
        "phase": "transition",
        "main_request_rad_s": -0.5,
        "auxiliary_request_rad": 0.25,
    }
    trace = {"samples": [source_sample]}

    normalized = round5._trace_samples(trace)

    assert normalized[0]["requested_main_rad_s"] == pytest.approx(-0.5)
    assert normalized[0]["requested_auxiliary_rad"] == pytest.approx(0.25)
    assert "requested_main_rad_s" not in source_sample
    assert "requested_auxiliary_rad" not in source_sample


def test_physics_diagnostics_conversion_uses_current_state_and_next_tick_requests():
    arrays = {
        "simulation_time_s": np.asarray([0.0, 0.02, 0.04]),
        "raw_command_target": np.asarray([[0.0, 0.0], [0.5, 0.1], [0.5, 0.2]]),
        "applied_action": np.asarray([[0.0, 0.0], [0.01, 0.01], [0.02, 0.02]]),
        "joint_position_rad": np.zeros((3, 3)),
        "joint_velocity_rad_s": np.asarray([[0.0, 0.0, 0.0], [0.1, 0.2, 0.0], [0.2, 0.3, 0.0]]),
        "position_world_xy": np.asarray([[0.0, 0.0], [0.01, 0.0], [0.03, 0.0]]),
        "linear_velocity_world_m_s": np.asarray([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]]),
        "angular_velocity_world_rad_s": np.zeros((3, 3)),
        "yaw_world_rad": np.zeros(3),
    }

    samples = round5.physics_diagnostics_to_samples(arrays)

    assert samples[0]["requested_main_rad_s"] == pytest.approx(0.0)
    assert samples[1]["requested_main_rad_s"] == pytest.approx(0.5)
    assert samples[1]["final_main_rad_s"] == pytest.approx(0.01)
    assert samples[1]["auxiliary_joint_rad"] == pytest.approx(0.0)
    assert samples[1]["auxiliary_velocity_supervisor_active"] is False


def test_predictor_contract_rejects_future_measured_inputs():
    round5.validate_predictor_payload(
        {
            "current_state": {"main_joint_velocity_rad_s": 0.0},
            "future_requests": [{"main_request_rad_s": 0.5, "auxiliary_request_rad": 0.0}],
        }
    )

    with pytest.raises(ValueError, match="future measured"):
        round5.validate_predictor_payload(
            {
                "current_state": {"main_joint_velocity_rad_s": 0.0},
                "future_requests": [{"main_request_rad_s": 0.5}],
                "future_measured_velocity": [0.1],
            }
        )


def test_zero_crossing_interpolates_reversal_and_is_not_applicable_without_reversal():
    elapsed = np.asarray([0.0, 0.5, 1.0, 1.5])
    values = np.asarray([-0.5, -0.2, 0.1, 0.4])

    crossing = round5._target_sign_zero_crossing_time(values, elapsed, target=0.5)

    assert crossing == pytest.approx(5.0 / 6.0)
    assert round5._target_sign_zero_crossing_time(values, elapsed, target=-0.5) is None


def test_protocol_and_manifest_are_json_and_preregister_candidate_gate():
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    assert protocol["round4_decision_is_immutable"] is True
    assert protocol["candidate_gates"]["minimum_endpoint_mae_improvement_fraction_over_m0"] == pytest.approx(0.20)
    assert manifest["procedure"]["maximum_total_new_sequences"] == 20
    assert manifest["drive_profile"]["sha256"] == "2de28cdfe98ecd6d45d5158261efb436acabaf31c1df4774f75b708a9086ddfd"
