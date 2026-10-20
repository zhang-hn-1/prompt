"""Hand-derived CPU tests for the spatial trajectory reference profile."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from benchmark.drive_model import DriveModel
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.kinodynamic_planning import (
    KinodynamicPlanRequest,
    plan_kinodynamic_path,
)
from benchmark.trajectory_tracking import SpatialReferenceProfile
from benchmark.trajectory_tracking import blend_tracking_curvature
from benchmark.trajectory_tracking import curvature_blend_weight
from benchmark.trajectory_tracking import query_delay_aware_reference


DRIVE_PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)


def _waypoint(
    x: float,
    y: float,
    yaw: float,
    direction: str = "forward",
    command: float = 0.5,
) -> KinodynamicWaypoint:
    return KinodynamicWaypoint(x, y, yaw, direction, command, 0.0)  # type: ignore[arg-type]


def _line(*, direction: str = "forward", command: float = 0.5):
    return (
        _waypoint(0.0, 0.0, 0.0, direction, command),
        _waypoint(1.0, 0.0, 0.0, direction, command),
        _waypoint(2.0, 0.0, 0.0, direction, command),
        _waypoint(3.0, 0.0, 0.0, direction, command),
    )


def _quarter_circle(turn_sign: int = 1):
    # Independent hand-derived samples from a radius-3, 90-degree arc.
    samples = (
        (0.000000, 0.000000, 0.000000),
        (0.776457, 0.102809, 0.261799),
        (1.500000, 0.401924, 0.523599),
        (2.121320, 0.878680, 0.785398),
        (2.598076, 1.500000, 1.047198),
        (2.897777, 2.223543, 1.308997),
        (3.000000, 3.000000, 1.570796),
    )
    return tuple(
        _waypoint(x, turn_sign * y, turn_sign * yaw)
        for x, y, yaw in samples
    )


def test_straight_line_has_zero_curvature_and_monotonic_arc_length():
    profile = SpatialReferenceProfile.from_waypoints(_line())

    np.testing.assert_allclose(profile.xy, [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    np.testing.assert_allclose(profile.s_m, [0.0, 1.0, 2.0, 3.0])
    np.testing.assert_allclose(profile.yaw_unwrapped, 0.0, atol=1e-12)
    np.testing.assert_allclose(profile.curvature_1_m, 0.0, atol=1e-12)
    assert np.all(np.diff(profile.s_m) >= 0.0)


@pytest.mark.parametrize("turn_sign", (1, -1))
def test_radius_three_quarter_circle_has_expected_signed_curvature(turn_sign: int):
    profile = SpatialReferenceProfile.from_waypoints(
        _quarter_circle(turn_sign=turn_sign), smoothing_window_m=0.0
    )

    assert float(np.median(profile.curvature_1_m[1:-1])) == pytest.approx(
        turn_sign / 3.0, abs=1e-2
    )


def test_s_curve_curvature_changes_sign_without_losing_samples():
    waypoints = (
        _waypoint(0.0, 0.0, 0.00),
        _waypoint(1.0, 0.2, 0.20),
        _waypoint(2.0, 0.6, 0.40),
        _waypoint(3.0, 0.8, 0.00),
        _waypoint(4.0, 0.4, -0.40),
        _waypoint(5.0, 0.0, -0.20),
    )

    profile = SpatialReferenceProfile.from_waypoints(waypoints)

    assert len(profile.curvature_1_m) == 6
    assert np.any(profile.curvature_1_m > 1e-4)
    assert np.any(profile.curvature_1_m < -1e-4)


def test_yaw_unwrap_is_per_direction_segment_across_plus_minus_pi():
    waypoints = (
        _waypoint(0.0, 0.0, 3.0),
        _waypoint(1.0, 0.0, -3.0),
        _waypoint(2.0, 0.0, -2.8),
    )

    profile = SpatialReferenceProfile.from_waypoints(waypoints, smoothing_window_m=0.0)

    np.testing.assert_allclose(
        profile.yaw_unwrapped,
        [3.0, 2.0 * math.pi - 3.0, 2.0 * math.pi - 2.8],
        atol=1e-12,
    )

    reverse_wrap = SpatialReferenceProfile.from_waypoints(
        (
            _waypoint(0.0, 0.0, -3.0, "reverse", -0.5),
            _waypoint(1.0, 0.0, 3.0, "reverse", -0.5),
            _waypoint(2.0, 0.0, 3.2, "reverse", 0.0),
        ),
        smoothing_window_m=0.0,
    )
    np.testing.assert_allclose(
        reverse_wrap.yaw_unwrapped,
        [-3.0, -math.pi - (math.pi - 3.0), -math.pi - (math.pi - 3.2)],
        atol=1e-12,
    )


def test_duplicate_terminal_point_inherits_adjacent_finite_curvature():
    waypoints = (
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(1.0, 0.0, 0.5),
        _waypoint(1.0, 0.0, 0.5, command=0.0),
    )

    profile = SpatialReferenceProfile.from_waypoints(waypoints, smoothing_window_m=0.0)

    np.testing.assert_allclose(profile.s_m, [0.0, 1.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(profile.curvature_1_m, [0.5, 0.5, 0.5], atol=1e-12)
    assert profile.xy.shape == (3, 2)


def test_real_planner_singleton_direction_run_is_zero_curvature_metadata():
    grid = np.zeros((81, 81), dtype=np.uint8)
    request = KinodynamicPlanRequest(
        occupancy_grid=grid,
        resolution_m=0.1,
        origin_xy=(-4.0, -4.0),
        start_xy=(0.0, 0.2),
        start_yaw_rad=0.0,
        goal_xy=(0.0, 0.6),
        node_budget=250_000,
    )
    result = plan_kinodynamic_path(request, DriveModel.from_json(DRIVE_PROFILE_PATH))

    assert result.status == "success"
    assert [point.drive_direction for point in result.waypoints] == [
        "reverse",
        "forward",
        "forward",
        "forward",
    ]
    profile = SpatialReferenceProfile.from_waypoints(result.waypoints)

    expected_s = np.concatenate(
        (
            [0.0],
            np.cumsum(
                [
                    math.dist(
                        (left.x_m, left.y_m), (right.x_m, right.y_m)
                    )
                    for left, right in zip(result.waypoints, result.waypoints[1:])
                ]
            ),
        )
    )
    np.testing.assert_allclose(profile.s_m, expected_s, atol=1e-12)
    np.testing.assert_allclose(
        profile.yaw_unwrapped,
        [point.yaw_rad for point in result.waypoints],
        atol=1e-12,
    )
    assert profile.segment_id.tolist() == [0, 1, 1, 1]
    assert profile.curvature_1_m[0] == pytest.approx(0.0, abs=0.0)
    assert profile.preview_max_abs_curvature(0, 10.0) == pytest.approx(0.0, abs=0.0)
    assert float(np.max(np.abs(profile.curvature_1_m[1:]))) > 1e-6


def test_profile_arrays_have_exact_public_dtypes():
    profile = SpatialReferenceProfile.from_waypoints(_line())

    assert profile.xy.dtype == np.dtype(np.float64)
    assert profile.yaw_unwrapped.dtype == np.dtype(np.float64)
    assert profile.s_m.dtype == np.dtype(np.float64)
    assert profile.curvature_1_m.dtype == np.dtype(np.float64)
    assert profile.segment_id.dtype == np.dtype(np.int64)
    assert profile.drive_direction.dtype == np.dtype("<U7")
    assert profile.main_command_rad_s.dtype == np.dtype(np.float64)


def test_direction_change_starts_new_segment_without_smoothing_or_preview_leakage():
    first = [
        _waypoint(0.0, 0.0, 0.0, "forward", 0.5),
        _waypoint(0.2, 0.0, 0.2, "forward", 0.5),
        _waypoint(0.4, 0.0, 0.4, "forward", 0.5),
        _waypoint(0.6, 0.0, 0.6, "forward", 0.5),
    ]
    second = [
        _waypoint(0.8, 0.0, 1.6, "reverse", -0.5),
        _waypoint(1.0, 0.0, 1.4, "reverse", -0.5),
        _waypoint(1.2, 0.0, 1.2, "reverse", -0.5),
        _waypoint(1.4, 0.0, 1.0, "reverse", 0.0),
    ]
    profile = SpatialReferenceProfile.from_waypoints(
        tuple(first + second), smoothing_window_m=0.4
    )

    assert profile.segment_id.tolist() == [0, 0, 0, 0, 1, 1, 1, 1]
    assert profile.drive_direction.tolist() == ["forward"] * 4 + ["reverse"] * 4
    np.testing.assert_allclose(
        profile.main_command_rad_s,
        [0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, 0.0],
        atol=1e-12,
    )
    np.testing.assert_allclose(profile.curvature_1_m[:4], 1.0, atol=1e-12)
    np.testing.assert_allclose(profile.curvature_1_m[4:], -1.0, atol=1e-12)
    np.testing.assert_allclose(
        profile.s_m,
        [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4],
        atol=1e-12,
    )
    assert profile.preview_max_abs_curvature(3, 0.8) == pytest.approx(1.0)
    assert profile.preview_max_abs_curvature(4, 0.8) == pytest.approx(1.0)


def test_preview_max_curvature_stops_at_current_direction_segment():
    first = [
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.2, 0.0, 0.02),
        _waypoint(0.4, 0.0, 0.04),
        _waypoint(0.6, 0.0, 0.06),
    ]
    second = [
        _waypoint(0.8, 0.0, 0.22, "reverse", -0.5),
        _waypoint(1.0, 0.0, 0.38, "reverse", -0.5),
        _waypoint(1.2, 0.0, 0.54, "reverse", -0.5),
    ]
    profile = SpatialReferenceProfile.from_waypoints(
        tuple(first + second), smoothing_window_m=0.0
    )

    assert profile.preview_max_abs_curvature(3, 0.8) == pytest.approx(0.1, abs=1e-12)
    assert profile.preview_max_abs_curvature(4, 0.8) == pytest.approx(0.8, abs=1e-12)


def test_preview_distance_zero_and_inclusive_endpoint_are_deterministic():
    profile = SpatialReferenceProfile.from_waypoints(
        (
            _waypoint(0.0, 0.0, 0.00),
            _waypoint(0.2, 0.0, 0.02),
            _waypoint(0.4, 0.0, 0.10, command=0.0),
        ),
        smoothing_window_m=0.0,
    )

    np.testing.assert_allclose(profile.curvature_1_m, [0.1, 0.25, 0.4], atol=1e-12)
    assert profile.preview_max_abs_curvature(1, 0.0) == pytest.approx(0.25, abs=1e-12)
    assert profile.preview_max_abs_curvature(0, 0.4) == pytest.approx(0.4, abs=1e-12)
    assert profile.preview_max_abs_curvature(2, 0.0) == pytest.approx(0.4, abs=1e-12)


def test_arc_length_weighted_smoothing_is_local_and_hand_derived():
    waypoints = (
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(1.0, 0.0, 1.0),
        _waypoint(2.0, 0.0, 3.0),
    )

    profile = SpatialReferenceProfile.from_waypoints(waypoints, smoothing_window_m=2.0)

    # Raw endpoint/center differences are [1, 1.5, 2].  The centered 2 m
    # window uses Voronoi arc-length weights [0.5, 1, 0.5], yielding [1.25,
    # 1.5, 1.75] after clipping at the endpoints.
    np.testing.assert_allclose(profile.curvature_1_m, [1.25, 1.5, 1.75], atol=1e-12)


def test_irregular_spacing_uses_voronoi_arc_length_weights_not_unweighted_means():
    waypoints = (
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(0.1, 0.0, 0.1),
        _waypoint(0.4, 0.0, 0.7),
        _waypoint(0.9, 0.0, 1.3, command=0.0),
    )

    profile = SpatialReferenceProfile.from_waypoints(waypoints, smoothing_window_m=0.4)

    # Raw differences are [1.0, 1.75, 1.5, 1.2].  The clipped 0.40 m
    # windows have Voronoi overlap weights that produce these literals; a
    # plain equal-sample mean gives different values at the first three rows.
    np.testing.assert_allclose(
        profile.curvature_1_m,
        [1.5625, 1.5833333333333333, 1.53125, 1.2],
        atol=1e-12,
    )


def test_profile_arrays_are_independent_copies_and_read_only():
    profile = SpatialReferenceProfile.from_waypoints(_line())

    arrays = (
        profile.xy,
        profile.yaw_unwrapped,
        profile.s_m,
        profile.curvature_1_m,
        profile.segment_id,
        profile.drive_direction,
        profile.main_command_rad_s,
    )
    assert all(array.flags.writeable is False for array in arrays)
    for left_index, left in enumerate(arrays):
        for right_index, right in enumerate(arrays):
            if left_index != right_index:
                assert not np.shares_memory(left, right)
    with pytest.raises(ValueError):
        profile.xy[0, 0] = 99.0
    with pytest.raises(ValueError):
        profile.main_command_rad_s[0] = 99.0


def _direct_profile_kwargs(**overrides):
    values = dict(
        xy=np.asarray([[0.0, 0.0], [1.0, 0.0]], dtype=np.float64),
        yaw_unwrapped=np.asarray([0.0, 0.0], dtype=np.float64),
        s_m=np.asarray([0.0, 1.0], dtype=np.float64),
        curvature_1_m=np.asarray([0.0, 0.0], dtype=np.float64),
        segment_id=np.asarray([0, 0], dtype=np.int64),
        drive_direction=np.asarray(["forward", "forward"], dtype="<U7"),
        main_command_rad_s=np.asarray([0.5, 0.5], dtype=np.float64),
    )
    values.update(overrides)
    return values


@pytest.mark.parametrize(
    "overrides",
    (
        dict(
            segment_id=np.asarray([0, 1], dtype=np.int64),
            drive_direction=np.asarray(["forward", "forward"], dtype="<U7"),
        ),
        dict(
            segment_id=np.asarray([0, 0], dtype=np.int64),
            drive_direction=np.asarray(["forward", "reverse"], dtype="<U7"),
            main_command_rad_s=np.asarray([0.5, -0.5], dtype=np.float64),
        ),
    ),
)
def test_direct_constructor_rejects_segment_id_direction_mismatch(overrides):
    with pytest.raises(ValueError, match="segment_id.*direction|direction.*segment_id"):
        SpatialReferenceProfile(**_direct_profile_kwargs(**overrides))


@pytest.mark.parametrize(
    "overrides",
    (
        dict(xy=np.asarray([[0.0 + 1.0j, 0.0], [1.0, 0.0]], dtype=np.complex128)),
        dict(yaw_unwrapped=np.asarray([0.0 + 1.0j, 0.0], dtype=np.complex128)),
        dict(segment_id=np.asarray([0.5, 0.0], dtype=np.float64)),
        dict(yaw_unwrapped=np.asarray(["0", "0"], dtype="<U1")),
    ),
)
def test_direct_constructor_rejects_unsafe_or_complex_numeric_coercion(overrides):
    with pytest.raises(ValueError):
        SpatialReferenceProfile(**_direct_profile_kwargs(**overrides))


@pytest.mark.parametrize(
    "waypoints",
    (
        (),
        (_waypoint(0.0, 0.0, 0.0),),
        (_waypoint(0.0, 0.0, 0.0), _waypoint(0.0, 0.0, 0.0)),
        (
            _waypoint(0.0, 0.0, 0.0),
            _waypoint(0.0, 0.0, 0.0),
            _waypoint(0.0, 0.0, 0.0),
        ),
    ),
)
def test_rejects_too_few_or_degenerate_arc_length_support(waypoints):
    with pytest.raises(ValueError):
        SpatialReferenceProfile.from_waypoints(waypoints)


def test_rejects_an_interior_duplicate_as_ambiguous_arc_length():
    waypoints = (
        _waypoint(0.0, 0.0, 0.0),
        _waypoint(1.0, 0.0, 0.1),
        _waypoint(1.0, 0.0, 0.2),
        _waypoint(2.0, 0.0, 0.3),
    )

    with pytest.raises(ValueError, match="zero-length|arc length"):
        SpatialReferenceProfile.from_waypoints(waypoints)


@pytest.mark.parametrize(
    "waypoints",
    (
        (_waypoint(math.nan, 0.0, 0.0), _waypoint(1.0, 0.0, 0.0)),
        (_waypoint(0.0, math.inf, 0.0), _waypoint(1.0, 0.0, 0.0)),
        (_waypoint(0.0, 0.0, math.nan), _waypoint(1.0, 0.0, 0.0)),
        (_waypoint(0.0, 0.0, 0.0), _waypoint(1.0, 0.0, math.inf)),
        (_waypoint(0.0, 0.0, 0.0, command=math.nan), _waypoint(1.0, 0.0, 0.0)),
        (_waypoint(0.0, 0.0, 0.0), _waypoint(1.0, 0.0, 0.0, command=math.inf)),
    ),
)
def test_rejects_nonfinite_waypoint_fields(waypoints):
    with pytest.raises(ValueError):
        SpatialReferenceProfile.from_waypoints(waypoints)


def test_rejects_unknown_direction_and_command_direction_mismatch():
    with pytest.raises(ValueError, match="drive_direction"):
        SpatialReferenceProfile.from_waypoints(
            (_waypoint(0.0, 0.0, 0.0, "sideways"), _waypoint(1.0, 0.0, 0.0, "sideways"))
        )
    with pytest.raises(ValueError, match="command"):
        SpatialReferenceProfile.from_waypoints(
            (_waypoint(0.0, 0.0, 0.0, "reverse", 0.5), _waypoint(1.0, 0.0, 0.0, "reverse", -0.5))
        )


@pytest.mark.parametrize("smoothing_window_m", (math.nan, math.inf, -0.1))
def test_rejects_invalid_smoothing_window(smoothing_window_m: float):
    with pytest.raises(ValueError):
        SpatialReferenceProfile.from_waypoints(_line(), smoothing_window_m=smoothing_window_m)


@pytest.mark.parametrize(
    "index,distance_m",
    ((-1, 0.8), (99, 0.8), (True, 0.8), (0, math.nan), (0, math.inf), (0, -0.1)),
)
def test_rejects_invalid_preview_queries(index, distance_m):
    profile = SpatialReferenceProfile.from_waypoints(_line())
    with pytest.raises((TypeError, ValueError)):
        profile.preview_max_abs_curvature(index, distance_m)


def _make_forward_then_reverse_profile() -> SpatialReferenceProfile:
    return SpatialReferenceProfile(**_direct_profile_kwargs(
        xy=np.asarray(
            [
                [0.0, 0.0],
                [0.2, 0.0],
                [0.4, 0.0],
                [0.6, 0.0],
                [0.8, 0.0],
                [1.0, 0.0],
                [1.2, 0.0],
            ],
            dtype=np.float64,
        ),
        yaw_unwrapped=np.zeros(7, dtype=np.float64),
        s_m=np.asarray([0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2], dtype=np.float64),
        curvature_1_m=np.asarray(
            [0.1, 0.2, 0.3, 0.4, -0.5, -0.6, -0.7], dtype=np.float64
        ),
        segment_id=np.asarray([0, 0, 0, 0, 1, 1, 1], dtype=np.int64),
        drive_direction=np.asarray(
            ["forward", "forward", "forward", "forward", "reverse", "reverse", "reverse"],
            dtype="<U7",
        ),
        main_command_rad_s=np.asarray(
            [0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5], dtype=np.float64
        ),
    ))


def _last_forward_index(profile: SpatialReferenceProfile) -> int:
    return int(np.flatnonzero(profile.drive_direction == "forward")[-1])


def test_delay_query_interpolates_and_never_crosses_direction_cusp():
    profile = _make_forward_then_reverse_profile()

    query = query_delay_aware_reference(
        profile,
        current_index=1,
        speed_m_s=0.4,
        auxiliary_latency_s=1.0,
        profile_window_m=0.30,
        maximum_delay_distance_m=0.30,
    )

    assert query.delay_distance_m == pytest.approx(0.30)
    assert query.projection_s_m == pytest.approx(0.20)
    assert query.query_s_m == pytest.approx(0.50)
    assert query.index == 3
    assert query.index <= _last_forward_index(profile)
    assert query.query_s_m <= profile.s_m[_last_forward_index(profile)]
    # The [0.35, 0.65] window overlaps the 0.4 m and 0.6 m Voronoi cells by
    # 0.15 m and 0.10 m, respectively, yielding a weighted mean of 0.34.
    assert query.curvature_1_m == pytest.approx(0.34)


def test_delay_query_uses_a_continuous_projection_within_current_segment():
    profile = _make_forward_then_reverse_profile()

    query = query_delay_aware_reference(
        profile,
        current_index=1,
        projection_s_m=0.35,
        speed_m_s=0.4,
        auxiliary_latency_s=1.0,
        profile_window_m=0.30,
        maximum_delay_distance_m=0.30,
    )

    assert query.projection_s_m == pytest.approx(0.35)
    assert query.query_s_m == pytest.approx(0.60)
    assert query.query_s_m - query.projection_s_m <= 0.30 + 1e-12
    assert query.index <= _last_forward_index(profile)

    with pytest.raises(ValueError, match="projection_s_m"):
        query_delay_aware_reference(
            profile,
            current_index=1,
            projection_s_m=0.70,
            speed_m_s=0.0,
            auxiliary_latency_s=1.0,
            profile_window_m=0.30,
            maximum_delay_distance_m=0.30,
        )


def test_delay_query_rejects_a_lead_cap_above_the_frozen_limit():
    profile = _make_forward_then_reverse_profile()

    with pytest.raises(ValueError, match="maximum_delay_distance_m.*0.30"):
        query_delay_aware_reference(
            profile,
            current_index=1,
            speed_m_s=0.4,
            auxiliary_latency_s=1.0,
            profile_window_m=0.30,
            maximum_delay_distance_m=0.31,
        )


def test_delay_query_uses_arc_length_weights_for_irregular_spacing():
    profile = SpatialReferenceProfile(**_direct_profile_kwargs(
        xy=np.asarray(
            [[0.0, 0.0], [0.1, 0.0], [0.4, 0.0], [0.9, 0.0]],
            dtype=np.float64,
        ),
        yaw_unwrapped=np.zeros(4, dtype=np.float64),
        s_m=np.asarray([0.0, 0.1, 0.4, 0.9], dtype=np.float64),
        curvature_1_m=np.asarray([0.0, 1.0, 3.0, 5.0], dtype=np.float64),
        segment_id=np.zeros(4, dtype=np.int64),
        drive_direction=np.asarray(["forward"] * 4, dtype="<U7"),
        main_command_rad_s=np.full(4, 0.5, dtype=np.float64),
    ))

    query = query_delay_aware_reference(
        profile,
        current_index=0,
        speed_m_s=0.2,
        auxiliary_latency_s=1.0,
        profile_window_m=0.40,
        maximum_delay_distance_m=0.30,
    )

    # The [0.0, 0.4] window has Voronoi-cell overlap weights
    # [0.05, 0.20, 0.15, 0.0], so the hand-derived weighted mean is 1.625.
    assert query.curvature_1_m == pytest.approx(1.625)


def test_delay_query_rejects_nonfinite_weighted_curvature_aggregation():
    profile = SpatialReferenceProfile(**_direct_profile_kwargs(
        xy=np.asarray(
            [[0.0, 0.0], [0.1, 0.0], [1.1, 0.0], [2.1, 0.0]],
            dtype=np.float64,
        ),
        yaw_unwrapped=np.zeros(4, dtype=np.float64),
        s_m=np.asarray([0.0, 0.1, 1.1, 2.1], dtype=np.float64),
        curvature_1_m=np.full(4, 1.0e308, dtype=np.float64),
        segment_id=np.zeros(4, dtype=np.int64),
        drive_direction=np.asarray(["forward"] * 4, dtype="<U7"),
        main_command_rad_s=np.full(4, 0.5, dtype=np.float64),
    ))

    with pytest.raises(ValueError, match="curvature.*finite"):
        query_delay_aware_reference(
            profile,
            current_index=0,
            speed_m_s=0.1,
            auxiliary_latency_s=1.0,
            profile_window_m=4.0,
            maximum_delay_distance_m=0.30,
        )


@pytest.mark.parametrize(
    ("profile_curvature", "expected_weight"),
    ((0.0, 0.0), (0.05, 0.0), (0.125, 0.5), (0.20, 1.0), (-0.20, 1.0)),
)
def test_curvature_blend_weight_is_symmetric_and_bounded(
    profile_curvature, expected_weight
):
    assert curvature_blend_weight(
        profile_curvature,
        blend_start_1_m=0.05,
        blend_full_1_m=0.20,
    ) == pytest.approx(expected_weight)


def test_curvature_blend_weight_rejects_nonfinite_values_and_invalid_range():
    with pytest.raises(ValueError):
        curvature_blend_weight(
            math.nan,
            blend_start_1_m=0.05,
            blend_full_1_m=0.20,
        )
    with pytest.raises(ValueError):
        curvature_blend_weight(
            0.1,
            blend_start_1_m=0.20,
            blend_full_1_m=0.20,
        )


def test_blend_tracking_curvature_preserves_pure_and_full_profile_limits():
    assert blend_tracking_curvature(0.23, 0.0, -0.17, weight=0.0) == pytest.approx(0.23)
    assert blend_tracking_curvature(0.23, 0.4, 0.05, weight=1.0) == pytest.approx(0.45)


def test_blend_tracking_curvature_rejects_nonfinite_values_and_out_of_range_weight():
    with pytest.raises(ValueError):
        blend_tracking_curvature(math.inf, 0.0, 0.0, weight=0.0)
    with pytest.raises(ValueError):
        blend_tracking_curvature(0.0, 0.0, 0.0, weight=1.1)
