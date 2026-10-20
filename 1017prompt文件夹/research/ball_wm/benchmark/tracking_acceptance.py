"""Deterministic, Isaac-free references and metrics for tracker acceptance.

The module deliberately contains no simulator or planner execution.  It only
describes finite geometric references using the project's existing
``KinodynamicWaypoint`` record and computes metrics that can be replayed from a
runtime trace.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import Literal, Mapping

import numpy as np

from .kinodynamic_planning import KinodynamicWaypoint


TRACKING_ACCEPTANCE_SCHEMA_VERSION = "tracking_acceptance_v1"
MAX_SAMPLE_SPACING_M = 0.05
_ALLOWED_MAIN_MAGNITUDES = (0.5, 1.0)
_DRIVE_DIRECTIONS = frozenset(("forward", "reverse"))
_CASE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
TRACKING_CASE_IDS = (
    "line_forward_0p5",
    "line_forward_1p0",
    "arc_left_r3_90_0p5",
    "arc_right_r3_90_0p5",
    "arc_left_r1p5_90_probe",
    "s_curve_0p5",
    "line_reverse_0p5",
    "forward_stop_reverse_stop_forward",
)

_FIXED_ACCEPTANCE_METADATA = {
    "sample_spacing_m": 0.05,
    "goal_tolerance_m": 0.2,
    "stop_speed_m_s": 0.05,
    "stop_hold_s": 0.5,
    "replan_hz": 10,
    "settling_budget_s": 3.0,
    "formal_budget_s": 35.0,
}
_FIXED_CONTROLLER_METADATA = {
    "main_acceleration_limit_rad_s2": 0.5,
    "main_derivative_gain": 80.0,
    "auxiliary_kp": 300.0,
    "auxiliary_kd": 60.0,
    "auxiliary_target_rate_limit_rad_s": math.radians(30.0),
}

DriveDirection = Literal["forward", "reverse"]


def _finite_float(value: object, *, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite numeric data")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite numeric data") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite numeric data")
    return result


def _normalise_angle(angle_rad: object) -> float:
    angle = _finite_float(angle_rad, name="angle_rad")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _finite_points(value: object, *, name: str, minimum_rows: int = 1) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must have shape (N, 2) and finite values") from error
    if array.ndim != 2 or array.shape[1] != 2 or array.shape[0] < minimum_rows:
        raise ValueError(f"{name} must have shape (N, 2) and finite values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must have shape (N, 2) and finite values")
    return array


def _finite_vector(value: object, *, name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain finite numeric values") from error
    if array.size == 0 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite numeric values")
    return array


def _freeze_parameters(value: object) -> MappingProxyType:
    if not isinstance(value, Mapping):
        raise ValueError("parameters must be a mapping of finite numeric values")
    frozen: dict[str, float] = {}
    for key, item in value.items():
        if not isinstance(key, str) or not key.strip():
            raise ValueError("parameter names must be non-empty strings")
        frozen[key] = _finite_float(item, name=f"parameters[{key!r}]")
    return MappingProxyType(frozen)


def _kind_alias(kind: str) -> str:
    aliases = {
        "line": "line",
        "line_forward": "line",
        "line_reverse": "line_reverse",
        "arc_left": "arc_left",
        "arc_right": "arc_right",
        "s": "s_curve",
        "s_curve": "s_curve",
        "mixed": "mixed",
        "mixed_direction": "mixed",
        "forward_stop_reverse_stop_forward": "mixed",
    }
    try:
        return aliases[kind]
    except KeyError as error:
        raise ValueError(f"unsupported tracking case kind: {kind!r}") from error


@dataclass(frozen=True)
class TrackingCase:
    """Immutable description of one deterministic tracker reference."""

    case_id: str
    kind: str
    main_command_rad_s: float
    budget_s: float
    parameters: Mapping[str, float]
    probe: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.case_id, str) or _CASE_ID_RE.fullmatch(self.case_id) is None:
            raise ValueError("case_id must be a portable non-empty identifier")
        if not isinstance(self.kind, str) or not self.kind.strip():
            raise ValueError("kind must be a non-empty string")
        _kind_alias(self.kind)
        command = _finite_float(
            self.main_command_rad_s, name="main_command_rad_s"
        )
        if not any(
            math.isclose(abs(command), allowed, rel_tol=0.0, abs_tol=1e-12)
            for allowed in _ALLOWED_MAIN_MAGNITUDES
        ):
            raise ValueError("main_command_rad_s magnitude must be 0.5 or 1.0")
        budget = _finite_float(self.budget_s, name="budget_s")
        if budget <= 0.0:
            raise ValueError("budget_s must be positive")
        if not isinstance(self.probe, bool):
            raise ValueError("probe must be a boolean")
        parameters = _freeze_parameters(self.parameters)
        object.__setattr__(self, "main_command_rad_s", command)
        object.__setattr__(self, "budget_s", budget)
        object.__setattr__(self, "parameters", parameters)

    @property
    def start_xy(self) -> tuple[float, float]:
        return (
            float(self.parameters.get("start_x_m", 0.0)),
            float(self.parameters.get("start_y_m", 0.0)),
        )

    @property
    def start_yaw_rad(self) -> float:
        return float(self.parameters.get("start_yaw_rad", 0.0))


@dataclass(frozen=True)
class TrackingStopMarker:
    """A zero-command cusp marker kept outside policy waypoints.

    ``OraclePathPolicy`` consumes planner-style waypoints and intentionally has
    no zero-command waypoint at a direction cusp.  This record preserves the
    required physical stop location for the diagnostic harness without adding
    a same-position waypoint that would prevent that policy from advancing.
    """

    point_index: int
    segment_index: int
    x_m: float
    y_m: float
    yaw_rad: float
    drive_direction: DriveDirection
    main_command_rad_s: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.point_index, bool) or not isinstance(
            self.point_index, (int, np.integer)
        ):
            raise ValueError("point_index must be an integer")
        if isinstance(self.segment_index, bool) or not isinstance(
            self.segment_index, (int, np.integer)
        ):
            raise ValueError("segment_index must be an integer")
        if int(self.segment_index) < 0:
            raise ValueError("segment_index must be non-negative")
        if self.drive_direction not in _DRIVE_DIRECTIONS:
            raise ValueError("drive_direction must be forward or reverse")
        if _finite_float(self.main_command_rad_s, name="main_command_rad_s") != 0.0:
            raise ValueError("stop markers must have a zero main command")
        object.__setattr__(self, "point_index", int(self.point_index))
        object.__setattr__(self, "segment_index", int(self.segment_index))
        object.__setattr__(self, "x_m", _finite_float(self.x_m, name="x_m"))
        object.__setattr__(self, "y_m", _finite_float(self.y_m, name="y_m"))
        object.__setattr__(self, "yaw_rad", _normalise_angle(self.yaw_rad))
        object.__setattr__(self, "main_command_rad_s", 0.0)

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x_m, self.y_m)

    def as_waypoint(self) -> KinodynamicWaypoint:
        return KinodynamicWaypoint(
            self.x_m,
            self.y_m,
            self.yaw_rad,
            self.drive_direction,
            0.0,
            0.0,
        )


@dataclass(frozen=True)
class TrackingReference:
    """Finite dense geometry plus planner-compatible waypoint records."""

    case: TrackingCase
    points_xy: np.ndarray
    yaw_rad: np.ndarray
    waypoints: tuple[KinodynamicWaypoint, ...]
    segment_boundaries: tuple[int, ...] = ()
    segment_directions: tuple[DriveDirection, ...] = ()
    stop_markers: tuple[TrackingStopMarker, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.case, TrackingCase):
            raise TypeError("case must be a TrackingCase")
        points = _finite_points(self.points_xy, name="points_xy", minimum_rows=2).copy()
        try:
            yaw = np.asarray(self.yaw_rad, dtype=np.float64)
        except (TypeError, ValueError) as error:
            raise ValueError("yaw_rad must be a finite vector") from error
        if yaw.ndim != 1 or yaw.shape[0] != points.shape[0] or not np.all(np.isfinite(yaw)):
            raise ValueError("yaw_rad must have one finite value per point")
        yaw = np.asarray([_normalise_angle(value) for value in yaw], dtype=np.float64)
        points.setflags(write=False)
        yaw.setflags(write=False)

        try:
            waypoints = tuple(self.waypoints)
        except TypeError as error:
            raise ValueError("waypoints must be an iterable of KinodynamicWaypoint") from error
        if len(waypoints) != points.shape[0] or not all(
            isinstance(item, KinodynamicWaypoint) for item in waypoints
        ):
            raise ValueError("waypoints must match points and use KinodynamicWaypoint")
        waypoint_xy = np.asarray([(item.x_m, item.y_m) for item in waypoints], dtype=np.float64)
        waypoint_yaw = np.asarray([item.yaw_rad for item in waypoints], dtype=np.float64)
        if not np.all(np.isfinite(waypoint_xy)) or not np.all(np.isfinite(waypoint_yaw)):
            raise ValueError("waypoints must contain finite values")
        if not np.allclose(waypoint_xy, points, rtol=0.0, atol=1e-9):
            raise ValueError("waypoint coordinates must match points_xy")
        yaw_delta = np.asarray(
            [_normalise_angle(a - b) for a, b in zip(waypoint_yaw, yaw)],
            dtype=np.float64,
        )
        if not np.allclose(yaw_delta, 0.0, rtol=0.0, atol=1e-9):
            raise ValueError("waypoint yaw values must match yaw_rad")
        for waypoint in waypoints:
            if waypoint.drive_direction not in _DRIVE_DIRECTIONS:
                raise ValueError("waypoint drive_direction must be forward or reverse")
            main = _finite_float(
                waypoint.main_command_rad_s, name="waypoint.main_command_rad_s"
            )
            if abs(main) > 2.0 + 1e-12:
                raise ValueError("waypoint main command exceeds action bounds")
            if main > 0.0 and waypoint.drive_direction != "forward":
                raise ValueError("positive waypoint command must be forward")
            if main < 0.0 and waypoint.drive_direction != "reverse":
                raise ValueError("negative waypoint command must be reverse")
            _finite_float(waypoint.auxiliary_angle_rad, name="waypoint.auxiliary_angle_rad")

        boundaries = tuple(self.segment_boundaries)
        if not boundaries:
            boundaries = (points.shape[0] - 1,)
        if any(
            isinstance(index, bool)
            or not isinstance(index, (int, np.integer))
            or int(index) < 0
            or int(index) >= points.shape[0]
            for index in boundaries
        ):
            raise ValueError("segment_boundaries must contain valid point indices")
        boundaries = tuple(int(index) for index in boundaries)
        if any(later <= earlier for earlier, later in zip(boundaries, boundaries[1:])):
            raise ValueError("segment_boundaries must be strictly increasing")
        directions = tuple(self.segment_directions)
        if not directions:
            directions = (waypoints[0].drive_direction,)
        if len(directions) != len(boundaries) or any(
            direction not in _DRIVE_DIRECTIONS for direction in directions
        ):
            raise ValueError("segment_directions must align with boundaries")
        markers = tuple(self.stop_markers)
        if not all(isinstance(marker, TrackingStopMarker) for marker in markers):
            raise ValueError("stop_markers must contain TrackingStopMarker values")
        for marker in markers:
            if not 0 <= marker.point_index < points.shape[0]:
                raise ValueError("stop marker point_index is outside points_xy")
            point = points[marker.point_index]
            if not np.allclose(point, marker.xy, rtol=0.0, atol=1e-9):
                raise ValueError("stop marker pose must match points_xy")
        object.__setattr__(self, "points_xy", points)
        object.__setattr__(self, "yaw_rad", yaw)
        object.__setattr__(self, "waypoints", waypoints)
        object.__setattr__(self, "segment_boundaries", boundaries)
        object.__setattr__(self, "segment_directions", directions)
        object.__setattr__(self, "stop_markers", markers)

    @property
    def segment_boundary_indices(self) -> tuple[int, ...]:
        """Alias emphasizing that boundaries refer to point-array indices."""

        return self.segment_boundaries

    @property
    def segment_end_indices(self) -> tuple[int, ...]:
        return self.segment_boundaries

    @property
    def segment_start_indices(self) -> tuple[int, ...]:
        starts = [0]
        starts.extend(index + 1 for index in self.segment_boundaries[:-1])
        return tuple(starts)

    @property
    def ordered_segments(self) -> tuple[tuple[int, int, DriveDirection], ...]:
        return tuple(
            (start, end, direction)
            for start, end, direction in zip(
                self.segment_start_indices,
                self.segment_boundaries,
                self.segment_directions,
            )
        )

    @property
    def stop_marker_indices(self) -> tuple[int, ...]:
        return tuple(marker.point_index for marker in self.stop_markers)

    @property
    def stop_waypoints(self) -> tuple[KinodynamicWaypoint, ...]:
        """Planner-shaped zero-command views of transition stop markers."""

        return tuple(marker.as_waypoint() for marker in self.stop_markers)

    @property
    def boundary_waypoints(self) -> tuple[KinodynamicWaypoint, ...]:
        """Alias for consumers that call transition markers boundaries."""

        return self.stop_waypoints

    @property
    def path_length_m(self) -> float:
        return polyline_length_m(self.points_xy)

    @property
    def goal_xy(self) -> tuple[float, float]:
        return (float(self.points_xy[-1, 0]), float(self.points_xy[-1, 1]))


def _parameter(case: TrackingCase, name: str, default: float) -> float:
    value = case.parameters.get(name, default)
    return _finite_float(value, name=f"parameters[{name!r}]")


def _sample_count(length_m: float, spacing_m: float) -> int:
    if not math.isfinite(length_m) or length_m <= 0.0:
        raise ValueError("geometry length must be finite and positive")
    spacing = _finite_float(spacing_m, name="sample_spacing_m")
    if spacing <= 0.0 or spacing > MAX_SAMPLE_SPACING_M + 1e-12:
        raise ValueError("sample_spacing_m must be in (0, 0.05]")
    # Add one interval when the quotient is exact in decimal but rounds just
    # above the requested spacing in binary (for example, 3.0 / 0.05).
    return max(1, int(math.ceil(length_m / spacing)) + 1)


def _sample_line(
    *,
    start_pose: tuple[float, float, float],
    length_m: float,
    spacing_m: float,
) -> np.ndarray:
    length = _finite_float(length_m, name="length_m")
    if length <= 0.0:
        raise ValueError("line length must be positive")
    count = _sample_count(length, spacing_m)
    distance = np.linspace(0.0, length, count + 1, dtype=np.float64)
    x0, y0, yaw0 = (_finite_float(value, name="start_pose") for value in start_pose)
    return np.column_stack(
        (
            x0 + distance * math.cos(yaw0),
            y0 + distance * math.sin(yaw0),
            np.full(distance.shape, _normalise_angle(yaw0), dtype=np.float64),
        )
    )


def _sample_arc(
    *,
    start_pose: tuple[float, float, float],
    radius_m: float,
    turn_sign: int,
    turn_angle_rad: float,
    spacing_m: float,
) -> np.ndarray:
    radius = _finite_float(radius_m, name="radius_m")
    angle = _finite_float(turn_angle_rad, name="turn_angle_rad")
    if radius <= 0.0 or angle <= 0.0:
        raise ValueError("arc radius and turn angle must be positive")
    if turn_sign not in (-1, 1):
        raise ValueError("turn_sign must be -1 or 1")
    arc_length = radius * angle
    count = _sample_count(arc_length, spacing_m)
    distance = np.linspace(0.0, arc_length, count + 1, dtype=np.float64)
    x0, y0, yaw0 = (_finite_float(value, name="start_pose") for value in start_pose)
    curvature = float(turn_sign) / radius
    yaw = yaw0 + curvature * distance
    x = x0 + (np.sin(yaw) - math.sin(yaw0)) / curvature
    y = y0 + (-np.cos(yaw) + math.cos(yaw0)) / curvature
    return np.column_stack(
        (x, y, np.asarray([_normalise_angle(value) for value in yaw], dtype=np.float64))
    )


def _sample_s_curve(
    *,
    start_pose: tuple[float, float, float],
    length_m: float,
    heading_amplitude_rad: float,
    spacing_m: float,
) -> np.ndarray:
    length = _finite_float(length_m, name="length_m")
    amplitude = _finite_float(
        heading_amplitude_rad, name="heading_amplitude_rad"
    )
    if length <= 0.0 or amplitude <= 0.0 or amplitude >= math.pi / 2.0:
        raise ValueError("S-curve length and amplitude are out of range")
    count = _sample_count(length, spacing_m)
    distance = np.linspace(0.0, length, count + 1, dtype=np.float64)
    # A smooth heading profile starts and ends tangent to the x-axis while
    # its derivative changes sign twice.  Integrating it by trapezoids keeps
    # the point spacing bounded by the chosen arc-length spacing.
    local_yaw = amplitude * np.sin(2.0 * math.pi * distance / length)
    ds = np.diff(distance)
    local_x = np.concatenate(
        ([0.0], np.cumsum(0.5 * ds * (np.cos(local_yaw[:-1]) + np.cos(local_yaw[1:]))))
    )
    local_y = np.concatenate(
        ([0.0], np.cumsum(0.5 * ds * (np.sin(local_yaw[:-1]) + np.sin(local_yaw[1:]))))
    )
    measured_length = float(
        np.linalg.norm(
            np.diff(np.column_stack((local_x, local_y)), axis=0), axis=1
        ).sum()
    )
    if measured_length <= 0.0 or not math.isfinite(measured_length):
        raise ValueError("S-curve integration produced an invalid length")
    # Trapezoidal integration is intentionally used for a deterministic dense
    # trace; normalize the resulting chord length back to the requested
    # arc-length parameter so metadata and diagnostics agree exactly.
    local_x *= length / measured_length
    local_y *= length / measured_length
    x0, y0, yaw0 = (_finite_float(value, name="start_pose") for value in start_pose)
    cosine = math.cos(yaw0)
    sine = math.sin(yaw0)
    world_x = x0 + cosine * local_x - sine * local_y
    world_y = y0 + sine * local_x + cosine * local_y
    world_yaw = np.asarray(
        [_normalise_angle(yaw0 + value) for value in local_yaw], dtype=np.float64
    )
    return np.column_stack((world_x, world_y, world_yaw))


def _assemble_reference(
    case: TrackingCase,
    segments: list[tuple[np.ndarray, DriveDirection, float]],
    *,
    spacing_m: float,
) -> TrackingReference:
    if not segments:
        raise ValueError("at least one geometry segment is required")
    spacing = _finite_float(spacing_m, name="sample_spacing_m")
    if spacing <= 0.0 or spacing > MAX_SAMPLE_SPACING_M + 1e-12:
        raise ValueError("sample_spacing_m must be in (0, 0.05]")
    point_parts: list[np.ndarray] = []
    direction_parts: list[str] = []
    command_parts: list[float] = []
    boundaries: list[int] = []
    stop_markers: list[TrackingStopMarker] = []
    previous_end_index: int | None = None
    for segment_index, (poses, direction, command) in enumerate(segments):
        poses = np.asarray(poses, dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1] != 3 or poses.shape[0] < 2:
            raise ValueError("each geometry segment must have shape (N, 3)")
        if direction not in _DRIVE_DIRECTIONS:
            raise ValueError("segment direction must be forward or reverse")
        if (direction == "forward" and command <= 0.0) or (
            direction == "reverse" and command >= 0.0
        ):
            raise ValueError("segment command sign does not match direction")
        if not np.all(np.isfinite(poses)):
            raise ValueError("geometry segment must be finite")
        first_segment = not point_parts
        if first_segment:
            part = poses
        else:
            # The segment sampler includes its start pose; the shared cusp is
            # represented once, with the next moving sample carrying the new
            # direction just as planner reconstruction does.
            part = poses[1:]
        if part.shape[0] == 0:
            raise ValueError("each geometry segment must contribute a moving sample")
        point_parts.append(part)
        direction_parts.extend([direction] * part.shape[0])
        command_parts.extend([float(command)] * part.shape[0])
        current_end = sum(part.shape[0] for part in point_parts) - 1
        boundaries.append(current_end)
        if previous_end_index is not None:
            marker_pose = np.asarray(point_parts[-2][-1], dtype=np.float64)
            # The previous endpoint is the last point before this segment's
            # first moving sample.  Its zero-command marker is metadata only.
            stop_markers.append(
                TrackingStopMarker(
                    point_index=previous_end_index,
                    segment_index=segment_index - 1,
                    x_m=float(marker_pose[0]),
                    y_m=float(marker_pose[1]),
                    yaw_rad=float(marker_pose[2]),
                    drive_direction=segments[segment_index - 1][1],
                )
            )
        previous_end_index = current_end

    # A final planner-style stop is a real waypoint.  Its duplicate position
    # keeps the endpoint exact while retaining a zero-command terminal marker.
    final_pose = np.asarray(point_parts[-1][-1], dtype=np.float64)
    point_parts.append(final_pose.reshape(1, 3))
    direction_parts.append(segments[-1][1])
    command_parts.append(0.0)
    boundaries[-1] = sum(part.shape[0] for part in point_parts) - 1

    poses = np.concatenate(point_parts, axis=0)
    commands = np.asarray(command_parts, dtype=np.float64)
    directions = tuple(direction_parts)
    waypoints = tuple(
        KinodynamicWaypoint(
            float(pose[0]),
            float(pose[1]),
            _normalise_angle(float(pose[2])),
            direction,  # type: ignore[arg-type]
            float(command),
            0.0,
        )
        for pose, direction, command in zip(poses, directions, commands)
    )
    return TrackingReference(
        case=case,
        points_xy=poses[:, :2],
        yaw_rad=poses[:, 2],
        waypoints=waypoints,
        segment_boundaries=tuple(boundaries),
        segment_directions=tuple(segment[1] for segment in segments),
        stop_markers=tuple(stop_markers),
    )


def build_tracking_reference(case: TrackingCase) -> TrackingReference:
    """Build one deterministic dense reference from an immutable case."""

    if not isinstance(case, TrackingCase):
        raise TypeError("case must be a TrackingCase")
    kind = _kind_alias(case.kind)
    spacing = _parameter(case, "sample_spacing_m", MAX_SAMPLE_SPACING_M)
    start = (
        _parameter(case, "start_x_m", 0.0),
        _parameter(case, "start_y_m", 0.0),
        _parameter(case, "start_yaw_rad", 0.0),
    )
    command = abs(case.main_command_rad_s)
    if kind == "line_reverse":
        length = _parameter(case, "length_m", 3.0)
        poses = _sample_line(start_pose=start, length_m=length, spacing_m=spacing)
        poses[:, :2] = start[:2] + (poses[:, :2] - start[:2]) * -1.0
        return _assemble_reference(case, [(poses, "reverse", -command)], spacing_m=spacing)
    if kind == "line":
        length = _parameter(case, "length_m", 3.0)
        poses = _sample_line(start_pose=start, length_m=length, spacing_m=spacing)
        direction: DriveDirection = "forward"
        if case.main_command_rad_s < 0.0:
            # A generic ``line`` with a negative command is interpreted as a
            # reverse line while preserving the body's heading.
            poses[:, :2] = start[:2] + (poses[:, :2] - start[:2]) * -1.0
            direction = "reverse"
        return _assemble_reference(
            case,
            [
                (
                    poses,
                    direction,
                    case.main_command_rad_s
                    if direction == "forward"
                    else -command,
                )
            ],
            spacing_m=spacing,
        )
    if kind in ("arc_left", "arc_right"):
        radius = _parameter(case, "radius_m", 3.0)
        angle = _parameter(case, "turn_angle_rad", math.pi / 2.0)
        sign = 1 if kind == "arc_left" else -1
        poses = _sample_arc(
            start_pose=start,
            radius_m=radius,
            turn_sign=sign,
            turn_angle_rad=angle,
            spacing_m=spacing,
        )
        return _assemble_reference(case, [(poses, "forward", command)], spacing_m=spacing)
    if kind == "s_curve":
        poses = _sample_s_curve(
            start_pose=start,
            length_m=_parameter(case, "length_m", 4.0),
            heading_amplitude_rad=_parameter(case, "heading_amplitude_rad", 0.35),
            spacing_m=spacing,
        )
        return _assemble_reference(case, [(poses, "forward", command)], spacing_m=spacing)
    if kind == "mixed":
        first_length = _parameter(case, "forward_length_1_m", 1.5)
        reverse_length = _parameter(case, "reverse_length_m", 0.75)
        final_length = _parameter(case, "forward_length_2_m", 2.25)
        first = _sample_line(start_pose=start, length_m=first_length, spacing_m=spacing)
        reverse_start = tuple(float(value) for value in first[-1])
        reverse = _sample_line(
            start_pose=reverse_start, length_m=reverse_length, spacing_m=spacing
        )
        reverse[:, :2] = reverse_start[:2] + (reverse[:, :2] - reverse_start[:2]) * -1.0
        final_start = tuple(float(value) for value in reverse[-1])
        final = _sample_line(start_pose=final_start, length_m=final_length, spacing_m=spacing)
        return _assemble_reference(
            case,
            [
                (first, "forward", command),
                (reverse, "reverse", -command),
                (final, "forward", command),
            ],
            spacing_m=spacing,
        )
    raise AssertionError(f"unhandled tracking case kind: {kind}")


def load_tracking_cases(path: str | Path) -> tuple[TrackingCase, ...]:
    """Load and validate the fixed ordered diagnostic case set."""

    config_path = Path(path).expanduser()
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"could not read tracking acceptance config: {config_path}") from error
    if not isinstance(payload, Mapping):
        raise ValueError("tracking acceptance config must be a JSON object")
    if payload.get("schema_version") != TRACKING_ACCEPTANCE_SCHEMA_VERSION:
        raise ValueError("unsupported tracking acceptance schema")
    if payload.get("action_protocol_id") != "ball_action_v2_aux30":
        raise ValueError("action_protocol_id is fixed for tracking acceptance")
    for key, expected in _FIXED_ACCEPTANCE_METADATA.items():
        if key not in payload:
            raise ValueError(f"tracking config is missing fixed metadata {key}")
        actual = payload[key]
        if key == "replan_hz":
            if isinstance(actual, bool) or not isinstance(actual, int) or actual != expected:
                raise ValueError(f"{key} is fixed at {expected}")
        elif _finite_float(actual, name=key) != float(expected):
            raise ValueError(f"{key} is fixed at {expected}")
    controller = payload.get("controller")
    if not isinstance(controller, Mapping):
        raise ValueError("tracking config is missing fixed controller metadata")
    for key, expected in _FIXED_CONTROLLER_METADATA.items():
        actual = controller.get(key)
        if _finite_float(actual, name=f"controller.{key}") != expected:
            raise ValueError(f"controller.{key} is fixed at {expected}")
    bounds = controller.get("auxiliary_bounds_rad")
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
        raise ValueError("controller.auxiliary_bounds_rad must contain two values")
    expected_bounds = (-math.radians(30.0), math.radians(30.0))
    if any(
        _finite_float(actual, name="controller.auxiliary_bounds_rad") != expected
        for actual, expected in zip(bounds, expected_bounds)
    ):
        raise ValueError("controller.auxiliary_bounds_rad is fixed at +/-30 degrees")
    entries = payload.get("cases")
    if not isinstance(entries, list) or len(entries) != len(TRACKING_CASE_IDS):
        raise ValueError(f"tracking config must contain {len(TRACKING_CASE_IDS)} cases")
    cases: list[TrackingCase] = []
    default_spacing = payload["sample_spacing_m"]
    for expected_id, entry in zip(TRACKING_CASE_IDS, entries):
        if not isinstance(entry, Mapping):
            raise ValueError("each tracking case must be a JSON object")
        if entry.get("case_id") != expected_id:
            raise ValueError("tracking cases must use the fixed controller order")
        budget_value = entry.get("diagnostic_budget_s", entry.get("budget_s"))
        if budget_value is None:
            raise ValueError(f"{expected_id} is missing diagnostic_budget_s")
        parameters_value = entry.get("parameters", {})
        if not isinstance(parameters_value, Mapping):
            raise ValueError(f"{expected_id}.parameters must be an object")
        parameters = dict(parameters_value)
        if "sample_spacing_m" not in parameters:
            parameters["sample_spacing_m"] = default_spacing
        cases.append(
            TrackingCase(
                case_id=expected_id,
                kind=entry.get("kind", ""),  # type: ignore[arg-type]
                main_command_rad_s=entry.get("main_command_rad_s"),  # type: ignore[arg-type]
                budget_s=budget_value,
                parameters=parameters,
                probe=entry.get("probe", False),  # type: ignore[arg-type]
            )
        )
    return tuple(cases)


def polyline_length_m(points_xy: object) -> float:
    points = _finite_points(points_xy, name="points_xy", minimum_rows=2)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def polyline_cross_track_errors(points_xy: object, reference_xy: object) -> np.ndarray:
    """Return point-to-segment distances for every actual XY point."""

    points = _finite_points(points_xy, name="points_xy")
    reference = _finite_points(reference_xy, name="reference_xy", minimum_rows=2)
    starts = reference[:-1]
    vectors = reference[1:] - starts
    lengths_squared = np.einsum("ij,ij->i", vectors, vectors)
    delta = points[:, None, :] - starts[None, :, :]
    numerators = np.einsum("nsi,si->ns", delta, vectors)
    fractions = np.divide(
        numerators,
        lengths_squared[None, :],
        out=np.zeros_like(numerators),
        where=lengths_squared[None, :] > 1e-24,
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    nearest = starts[None, :, :] + fractions[:, :, None] * vectors[None, :, :]
    distances = np.linalg.norm(points[:, None, :] - nearest, axis=2)
    result = np.min(distances, axis=1)
    if not np.all(np.isfinite(result)):
        raise ValueError("cross-track computation produced non-finite values")
    return result.astype(np.float64, copy=False)


def heading_errors_rad(actual_yaw_rad: object, reference_yaw_rad: object) -> np.ndarray:
    """Return signed shortest-angle errors in ``[-pi, pi)``."""

    actual = _finite_vector(actual_yaw_rad, name="actual_yaw_rad")
    reference = _finite_vector(reference_yaw_rad, name="reference_yaw_rad")
    if actual.ndim != 1 or reference.ndim != 1 or actual.shape != reference.shape:
        raise ValueError("actual_yaw_rad and reference_yaw_rad must have equal shape")
    result = np.remainder(actual - reference + math.pi, 2.0 * math.pi) - math.pi
    if not np.all(np.isfinite(result)):
        raise ValueError("heading error computation produced non-finite values")
    return np.asarray(result, dtype=np.float64)


def _trace_array(
    value: object,
    *,
    name: str,
    expected_rows: int,
    norm_rows: bool = False,
    component_index: int | None = None,
) -> np.ndarray:
    """Return a finite state-aligned one-dimensional trace."""

    array = _finite_vector(value, name=name)
    if array.ndim == 2 and component_index is not None:
        if array.shape[1] <= component_index:
            raise ValueError(f"{name} does not contain component {component_index}")
        array = array[:, component_index]
    elif norm_rows and array.ndim == 2:
        if array.shape[1] not in (2, 3):
            raise ValueError(f"{name} must be a vector or 2/3D row trace")
        array = np.linalg.norm(array, axis=1)
    elif array.ndim != 1:
        raise ValueError(f"{name} must be a one-dimensional trace")
    if array.shape[0] != expected_rows:
        raise ValueError(f"{name} must align with the actual state trace")
    return np.asarray(array, dtype=np.float64)


def _trace_summary(
    value: object | None,
    *,
    name: str,
    expected_rows: int,
    component_index: int | None = None,
) -> dict[str, object] | None:
    if value is None:
        return None
    array = _trace_array(
        value,
        name=name,
        expected_rows=expected_rows,
        component_index=component_index,
    )
    absolute = np.abs(array)
    return {
        "mean": float(np.mean(absolute)),
        "p95": float(np.percentile(absolute, 95.0)),
        "max": float(np.max(absolute)),
        "final": float(array[-1]),
        "values": [float(item) for item in array],
    }


def _projected_reference_yaw(
    points: np.ndarray,
    reference: np.ndarray,
    reference_yaw: np.ndarray,
) -> np.ndarray:
    """Interpolate reference yaw on the nearest projected reference segment."""

    starts = reference[:-1]
    vectors = reference[1:] - starts
    lengths_squared = np.einsum("ij,ij->i", vectors, vectors)
    delta = points[:, None, :] - starts[None, :, :]
    numerators = np.einsum("nsi,si->ns", delta, vectors)
    fractions = np.divide(
        numerators,
        lengths_squared[None, :],
        out=np.zeros_like(numerators),
        where=lengths_squared[None, :] > 1e-24,
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    projected = starts[None, :, :] + fractions[:, :, None] * vectors[None, :, :]
    nearest_segment = np.argmin(
        np.einsum("nsi,nsi->ns", points[:, None, :] - projected, points[:, None, :] - projected),
        axis=1,
    )
    fraction = fractions[np.arange(points.shape[0]), nearest_segment]
    left = nearest_segment
    right = nearest_segment + 1
    yaw_delta = np.asarray(
        [_normalise_angle(reference_yaw[r] - reference_yaw[l]) for l, r in zip(left, right)],
        dtype=np.float64,
    )
    return np.asarray(
        [
            _normalise_angle(reference_yaw[l] + t * delta_angle)
            for l, t, delta_angle in zip(left, fraction, yaw_delta)
        ],
        dtype=np.float64,
    )


def _integer_trace(value: object, *, name: str, expected_rows: int) -> list[int]:
    array = _finite_vector(value, name=name)
    if array.ndim != 1 or array.shape[0] != expected_rows:
        raise ValueError(f"{name} must align with the actual state trace")
    rounded = np.rint(array)
    if not np.allclose(array, rounded, rtol=0.0, atol=0.0):
        raise ValueError(f"{name} must contain integer values")
    return [int(value) for value in rounded]


def _boundary_indices(value: object, *, name: str) -> list[int]:
    array = _finite_vector(value, name=name)
    if array.ndim != 1 or array.size == 0:
        raise ValueError(f"{name} must be a non-empty vector")
    rounded = np.rint(array)
    if not np.allclose(array, rounded, rtol=0.0, atol=0.0):
        raise ValueError(f"{name} must contain integer values")
    result = [int(value) for value in rounded]
    if any(later <= earlier for earlier, later in zip(result, result[1:])):
        raise ValueError(f"{name} must be strictly increasing")
    return result


def summarize_tracking_trace(
    actual_positions_xy: object,
    reference_xy: object,
    actual_yaw_rad: object,
    reference_yaw_rad: object,
    goal_xy: object,
    speed_m_s: object,
    *,
    main_velocity_rad_s: object | None = None,
    auxiliary_angle_rad: object | None = None,
    roll_world_rad: object | None = None,
    segment_progression: object | None = None,
    segment_boundaries: object | None = None,
) -> dict[str, object]:
    """Return descriptive, state-aligned tracking metrics.

    This function intentionally does not decide goal success, stop-hold
    success, or mixed-direction completion.  Those claims require aligned
    timestamps/actions and belong to the diagnostic harness.
    """

    positions = _finite_points(actual_positions_xy, name="actual_positions_xy")
    reference = _finite_points(reference_xy, name="reference_xy", minimum_rows=2)
    state_count = positions.shape[0]
    actual_yaw = _trace_array(
        actual_yaw_rad,
        name="actual_yaw_rad",
        expected_rows=state_count,
    )
    reference_yaw = _finite_vector(reference_yaw_rad, name="reference_yaw_rad")
    if reference_yaw.ndim != 1 or reference_yaw.shape[0] != reference.shape[0]:
        raise ValueError("reference_yaw_rad must align with reference_xy")
    reference_yaw = np.asarray(
        [_normalise_angle(value) for value in reference_yaw], dtype=np.float64
    )
    projected_yaw = _projected_reference_yaw(positions, reference, reference_yaw)
    cross_track = polyline_cross_track_errors(positions, reference)
    heading = heading_errors_rad(actual_yaw, projected_yaw)
    goal = _finite_vector(goal_xy, name="goal_xy")
    if goal.shape != (2,):
        raise ValueError("goal_xy must contain exactly two values")
    goal_errors = np.linalg.norm(positions - goal[None, :], axis=1)
    speeds = _trace_array(
        speed_m_s,
        name="speed_m_s",
        expected_rows=state_count,
        norm_rows=True,
    )

    def distribution(values: np.ndarray) -> dict[str, object]:
        absolute = np.abs(values)
        return {
            "mean": float(np.mean(absolute)),
            "p95": float(np.percentile(absolute, 95.0)),
            "max": float(np.max(absolute)),
            "final": float(values[-1]),
            "values": [float(item) for item in values],
        }

    result: dict[str, object] = {
        "cross_track_m": distribution(cross_track),
        "heading_error_rad": distribution(heading),
        "goal_error_m": [float(item) for item in goal_errors],
        "final_goal_error_m": float(goal_errors[-1]),
        "speed_m_s": distribution(speeds),
        "main_velocity_rad_s": _trace_summary(
            main_velocity_rad_s,
            name="main_velocity_rad_s",
            expected_rows=state_count,
            component_index=0,
        ),
        "auxiliary_angle_rad": _trace_summary(
            auxiliary_angle_rad,
            name="auxiliary_angle_rad",
            expected_rows=state_count,
            component_index=1,
        ),
        "roll_world_rad": _trace_summary(
            roll_world_rad,
            name="roll_world_rad",
            expected_rows=state_count,
        ),
        # Flat descriptive aliases are retained for tabular consumers.
        "cross_track_mean_m": float(np.mean(cross_track)),
        "cross_track_p95_m": float(np.percentile(cross_track, 95.0)),
        "cross_track_max_m": float(np.max(cross_track)),
        "heading_error_mean_rad": float(np.mean(np.abs(heading))),
        "heading_error_p95_rad": float(np.percentile(np.abs(heading), 95.0)),
        "heading_error_max_rad": float(np.max(np.abs(heading))),
    }
    if segment_progression is not None:
        result["segment_progression"] = {
            "boundaries": _boundary_indices(
                segment_boundaries,
                name="segment_boundaries",
            )
            if segment_boundaries is not None
            else [],
            "values": _integer_trace(
                segment_progression,
                name="segment_progression",
                expected_rows=state_count,
            ),
        }
    elif segment_boundaries is not None:
        result["segment_progression"] = {
            "boundaries": _boundary_indices(
                segment_boundaries,
                name="segment_boundaries",
            ),
            "values": None,
        }
    return result


__all__ = [
    "MAX_SAMPLE_SPACING_M",
    "TRACKING_ACCEPTANCE_SCHEMA_VERSION",
    "TRACKING_CASE_IDS",
    "TrackingCase",
    "TrackingReference",
    "TrackingStopMarker",
    "build_tracking_reference",
    "heading_errors_rad",
    "load_tracking_cases",
    "polyline_cross_track_errors",
    "polyline_length_m",
    "summarize_tracking_trace",
]
