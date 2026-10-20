"""CPU-only tests for deterministic tracking-acceptance geometry and metrics."""

from __future__ import annotations

import math
import json
from pathlib import Path

import numpy as np
import pytest

from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.tracking_acceptance import (
    TrackingCase,
    TrackingReference,
    build_tracking_reference,
    heading_errors_rad,
    load_tracking_cases,
    polyline_cross_track_errors,
    summarize_tracking_trace,
)


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "tracking_acceptance.json"
)


def test_config_has_eight_stable_cases_in_controller_order():
    cases = load_tracking_cases(CONFIG_PATH)

    assert [case.case_id for case in cases] == [
        "line_forward_0p5",
        "line_forward_1p0",
        "arc_left_r3_90_0p5",
        "arc_right_r3_90_0p5",
        "arc_left_r1p5_90_probe",
        "s_curve_0p5",
        "line_reverse_0p5",
        "forward_stop_reverse_stop_forward",
    ]
    assert cases[4].probe is True
    assert all(case.budget_s > 0.0 for case in cases)


def test_config_declares_frozen_trajectory_v2_policy_parameters():
    payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    policy = payload["trajectory_policy_v2"]
    assert policy["name"] == "oracle_trajectory_v2"
    assert policy["parameters"] == pytest.approx(
        {
            "auxiliary_latency_s": 1.0,
            "maximum_delay_distance_m": 0.30,
            "profile_window_m": 0.30,
            "profile_blend_start_1_m": 0.05,
            "profile_blend_full_1_m": 0.20,
            "feedback_lookahead_m": 1.0,
                "maximum_feedback_curvature_1_m": 0.20,
                "supported_main_command_rad_s": 0.50,
                "episode_budget_s": 35.0,
                "terminal_brake_buffer_m": 0.10,
            }
    )


def test_loader_enforces_fixed_acceptance_metadata(tmp_path: Path):
    payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    assert payload["goal_tolerance_m"] == pytest.approx(0.2)
    assert payload["stop_speed_m_s"] == pytest.approx(0.05)
    assert payload["stop_hold_s"] == pytest.approx(0.5)
    assert payload["replan_hz"] == 10
    assert payload["settling_budget_s"] == pytest.approx(3.0)

    payload["settling_budget_s"] = 2.0
    mutated = tmp_path / "tracking_acceptance.json"
    mutated.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="settling_budget_s"):
        load_tracking_cases(mutated)


def test_tracking_case_and_reference_are_immutable():
    case = TrackingCase(
        "line_forward_0p5",
        "line",
        0.5,
        12.0,
        {"length_m": 1.0},
    )
    assert case.parameters["length_m"] == pytest.approx(1.0)
    with pytest.raises(TypeError):
        case.parameters["length_m"] = 2.0  # type: ignore[index]

    reference = build_tracking_reference(case)
    assert isinstance(reference, TrackingReference)
    assert isinstance(reference.waypoints[0], KinodynamicWaypoint)
    assert reference.points_xy.flags.writeable is False
    assert reference.yaw_rad.flags.writeable is False
    with pytest.raises(ValueError):
        reference.points_xy[0, 0] = 4.0


def test_line_arc_and_reverse_endpoints_and_yaws_are_exact():
    cases = load_tracking_cases(CONFIG_PATH)

    forward = build_tracking_reference(cases[0])
    np.testing.assert_allclose(forward.points_xy[0], (0.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(forward.points_xy[-1], (3.0, 0.0), atol=1e-12)
    assert forward.yaw_rad[0] == pytest.approx(0.0, abs=1e-12)
    assert forward.yaw_rad[-1] == pytest.approx(0.0, abs=1e-12)

    left = build_tracking_reference(cases[2])
    np.testing.assert_allclose(left.points_xy[-1], (3.0, 3.0), atol=1e-9)
    assert left.yaw_rad[-1] == pytest.approx(math.pi / 2.0, abs=1e-9)

    right = build_tracking_reference(cases[3])
    np.testing.assert_allclose(right.points_xy[-1], (3.0, -3.0), atol=1e-9)
    assert right.yaw_rad[-1] == pytest.approx(-math.pi / 2.0, abs=1e-9)

    reverse = build_tracking_reference(cases[6])
    np.testing.assert_allclose(reverse.points_xy[-1], (-3.0, 0.0), atol=1e-12)
    assert reverse.yaw_rad[-1] == pytest.approx(0.0, abs=1e-12)
    assert reverse.waypoints[0].drive_direction == "reverse"
    assert reverse.waypoints[0].main_command_rad_s == pytest.approx(-0.5)


@pytest.mark.parametrize("case_index", range(8))
def test_reference_spacing_is_dense_and_monotonic(case_index: int):
    case = load_tracking_cases(CONFIG_PATH)[case_index]
    reference = build_tracking_reference(case)

    deltas = np.linalg.norm(np.diff(reference.points_xy, axis=0), axis=1)
    assert np.all(np.isfinite(deltas))
    assert np.all(deltas >= 0.0)
    assert float(np.max(deltas)) <= 0.05 + 1e-12


def test_s_curve_has_a_continuous_tangent_and_changes_curvature_sign():
    case = load_tracking_cases(CONFIG_PATH)[5]
    reference = build_tracking_reference(case)

    # The analytic S uses a continuous heading profile; the finite-difference
    # curvature must contain both left- and right-turn samples.
    heading_delta = np.unwrap(reference.yaw_rad)
    curvature = np.diff(heading_delta) / np.maximum(
        np.linalg.norm(np.diff(reference.points_xy, axis=0), axis=1), 1e-12
    )
    assert np.any(curvature > 1e-4)
    assert np.any(curvature < -1e-4)
    assert abs(float(reference.yaw_rad[0] - reference.yaw_rad[-1])) < 1e-9


def test_mixed_direction_reference_has_ordered_boundaries_and_distinct_goal():
    reference = build_tracking_reference(load_tracking_cases(CONFIG_PATH)[7])

    assert math.dist(reference.points_xy[0], reference.points_xy[-1]) > 1.0
    assert tuple(reference.segment_boundaries) == tuple(
        sorted(reference.segment_boundaries)
    )
    assert len(reference.segment_boundaries) == 3
    assert reference.segment_directions == ("forward", "reverse", "forward")
    assert len(reference.stop_markers) == 2
    assert all(marker.main_command_rad_s == 0.0 for marker in reference.stop_markers)
    np.testing.assert_allclose(
        [marker.x_m for marker in reference.stop_markers], (2.0, 0.75), atol=1e-12
    )
    # The only zero-command waypoint is the terminal planner-style marker;
    # cusp markers stay metadata so OraclePathPolicy can advance past them.
    zero_waypoints = [
        point for point in reference.waypoints if point.main_command_rad_s == 0.0
    ]
    assert len(zero_waypoints) == 1
    directions = [point.drive_direction for point in reference.waypoints]
    assert "forward" in directions and "reverse" in directions


def test_cross_track_uses_projection_onto_segments_not_nearest_vertices():
    errors = polyline_cross_track_errors(
        np.asarray([[0.5, 0.2], [1.0, -0.25]]),
        np.asarray([[0.0, 0.0], [1.0, 0.0]]),
    )

    np.testing.assert_allclose(errors, (0.2, 0.25), atol=1e-12)


def test_heading_error_wraps_to_signed_shortest_angle():
    errors = heading_errors_rad(
        np.asarray([-math.pi + 0.1, math.pi - 0.1]),
        np.asarray([math.pi - 0.1, -math.pi + 0.1]),
    )

    np.testing.assert_allclose(errors, (0.2, -0.2), atol=1e-12)
    assert np.all(errors >= -math.pi)
    assert np.all(errors < math.pi)


@pytest.mark.parametrize(
    "function,args",
    (
        (polyline_cross_track_errors, ([[0.0, math.nan]], [[0.0, 0.0], [1.0, 0.0]])),
        (polyline_cross_track_errors, ([[0.0, 0.0]], [[0.0, 0.0]])),
        (heading_errors_rad, ([[0.0]], [0.0, 1.0])),
        (heading_errors_rad, ([[0.0, math.inf]], [0.0, 0.0])),
    ),
)
def test_metrics_reject_nonfinite_or_malformed_arrays(function, args):
    with pytest.raises(ValueError):
        function(*args)


def _summary_inputs():
    return dict(
        actual_positions_xy=np.asarray([[0.0, 0.0], [1.0, 0.1], [2.0, 0.0]]),
        reference_xy=np.asarray([[0.0, 0.0], [2.0, 0.0]]),
        actual_yaw_rad=np.asarray([0.0, 0.02, 0.0]),
        reference_yaw_rad=np.asarray([0.0, 0.0]),
        goal_xy=np.asarray([2.0, 0.0]),
        speed_m_s=np.asarray([0.2, 0.1, 0.01]),
        main_velocity_rad_s=np.asarray([0.5, 0.5, 0.0]),
        auxiliary_angle_rad=np.asarray([0.0, 0.1, 0.0]),
        roll_world_rad=np.asarray([0.0, 0.01, 0.0]),
    )


def test_summary_reports_descriptive_metrics_without_acceptance_verdicts():
    summary = summarize_tracking_trace(**_summary_inputs())

    assert summary["cross_track_m"]["mean"] == pytest.approx(0.1 / 3.0)
    assert summary["cross_track_m"]["max"] == pytest.approx(0.1)
    assert summary["heading_error_rad"]["max"] == pytest.approx(0.02)
    assert summary["final_goal_error_m"] == pytest.approx(0.0)
    assert "acceptance" not in summary
    assert "stop_hold" not in summary
    assert "complete" not in str(summary)
    assert "verified" not in str(summary)
    assert "all_below_threshold" not in str(summary)


def test_summary_rejects_detached_speed_trace():
    inputs = _summary_inputs()
    inputs["speed_m_s"] = np.asarray([0.2, 0.1, 0.01, 0.01, 0.01])

    with pytest.raises(ValueError, match="speed_m_s.*align"):
        summarize_tracking_trace(**inputs)


def test_summary_requires_reference_yaw_to_align_reference_polyline():
    inputs = _summary_inputs()
    inputs["reference_yaw_rad"] = np.asarray([0.0, 0.0, 0.0])

    with pytest.raises(ValueError, match="reference_yaw_rad.*reference_xy"):
        summarize_tracking_trace(**inputs)


@pytest.mark.parametrize("field", ("main_velocity_rad_s", "auxiliary_angle_rad", "roll_world_rad"))
def test_summary_rejects_optional_trace_with_wrong_state_length(field: str):
    inputs = _summary_inputs()
    inputs[field] = np.zeros(4)

    with pytest.raises(ValueError, match="align"):
        summarize_tracking_trace(**inputs)


def test_summary_records_progression_without_verifying_or_completing_it():
    inputs = _summary_inputs()
    inputs["segment_boundaries"] = np.asarray([1, 2])
    inputs["segment_progression"] = np.asarray([2, 2, 2])

    summary = summarize_tracking_trace(**inputs)

    assert summary["segment_progression"] == {
        "boundaries": [1, 2],
        "values": [2, 2, 2],
    }
    assert "verified" not in str(summary)
