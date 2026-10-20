"""Pure-CPU spatial references for dynamics-aware trajectory tracking.

The profile is deliberately independent of simulator state.  It turns planner
waypoint poses into immutable geometry and keeps every drive-direction segment
separate while unwrapping, differentiating, smoothing, and previewing.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .kinodynamic_planning import KinodynamicWaypoint


_DIRECTIONS = frozenset(("forward", "reverse"))
_DEFAULT_SMOOTHING_WINDOW_M = 0.40
_MAX_DELAY_DISTANCE_M = 0.30


def _finite_float(value: object, *, name: str) -> float:
    """Convert one scalar to a finite float without accepting booleans."""

    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be finite numeric data")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite numeric data") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite numeric data")
    return result


def _segment_ids(directions: tuple[str, ...]) -> np.ndarray:
    """Number runs of equal direction in waypoint order."""

    result = np.zeros(len(directions), dtype=np.int64)
    for index in range(1, len(directions)):
        result[index] = result[index - 1] + int(
            directions[index] != directions[index - 1]
        )
    return result


def _segment_ranges(segment_id: np.ndarray) -> tuple[tuple[int, int], ...]:
    """Return inclusive ``(start, end)`` ranges for contiguous segment IDs."""

    if segment_id.size == 0:
        return ()
    starts = [0]
    ends: list[int] = []
    for index in range(1, segment_id.shape[0]):
        if int(segment_id[index]) != int(segment_id[index - 1]):
            ends.append(index - 1)
            starts.append(index)
    ends.append(segment_id.shape[0] - 1)
    return tuple(zip(starts, ends))


def _raw_segment_curvature(s_m: np.ndarray, yaw_unwrapped: np.ndarray) -> np.ndarray:
    """Differentiate yaw by arc length with finite endpoint fallbacks.

    A zero-length suffix is the planner's duplicate terminal marker.  Its
    curvature is filled from the nearest preceding finite sample, preserving
    the marker row in the profile.  Zero-length samples elsewhere are
    ambiguous for a spatial derivative and are rejected by the caller.
    """

    count = int(s_m.shape[0])
    if count < 2:
        raise ValueError("each direction segment must contain at least two points")
    intervals = np.diff(s_m)
    if not np.all(np.isfinite(intervals)) or np.any(intervals < 0.0):
        raise ValueError("arc length must be finite and monotonic")
    positive = intervals > 0.0
    if not np.any(positive):
        raise ValueError("each direction segment needs positive arc-length support")

    # A terminal duplicate is well-defined by inheritance.  A zero interval
    # before the final positive interval has no unique local derivative.
    last_positive_interval = int(np.flatnonzero(positive)[-1])
    if np.any(~positive[:last_positive_interval]):
        raise ValueError(
            "zero-length waypoint samples are only allowed at a segment terminal"
        )

    curvature = np.full(count, np.nan, dtype=np.float64)
    if positive[0]:
        curvature[0] = (yaw_unwrapped[1] - yaw_unwrapped[0]) / intervals[0]
    if positive[-1]:
        curvature[-1] = (
            yaw_unwrapped[-1] - yaw_unwrapped[-2]
        ) / intervals[-1]

    # Interior points use the requested central difference.  A duplicate
    # terminal point may leave the preceding point finite through this path.
    for index in range(1, count - 1):
        denominator = s_m[index + 1] - s_m[index - 1]
        if denominator > 0.0:
            curvature[index] = (
                yaw_unwrapped[index + 1] - yaw_unwrapped[index - 1]
            ) / denominator

    # Backfill terminal duplicates (and any endpoint whose one-sided interval
    # is zero) from the nearest finite derivative in the same direction run.
    finite_indices = np.flatnonzero(np.isfinite(curvature))
    if finite_indices.size == 0:
        raise ValueError("curvature derivative has no finite support")
    for index in range(count):
        if not math.isfinite(float(curvature[index])):
            prior = finite_indices[finite_indices < index]
            following = finite_indices[finite_indices > index]
            if prior.size:
                curvature[index] = curvature[int(prior[-1])]
            elif following.size:
                curvature[index] = curvature[int(following[0])]
    if not np.all(np.isfinite(curvature)):
        raise ValueError("curvature derivative produced non-finite values")
    return curvature


def _sample_support_edges(s_m: np.ndarray) -> np.ndarray:
    """Return Voronoi cell edges used as arc-length averaging weights."""

    count = int(s_m.shape[0])
    edges = np.empty(count + 1, dtype=np.float64)
    edges[0] = s_m[0]
    edges[-1] = s_m[-1]
    if count > 1:
        edges[1:-1] = 0.5 * (s_m[:-1] + s_m[1:])
    return edges


def _smooth_segment_curvature(
    s_m: np.ndarray,
    curvature: np.ndarray,
    *,
    smoothing_window_m: float,
) -> np.ndarray:
    """Locally average curvature over a centered arc-length window.

    Each sample represents its Voronoi cell along the segment.  The output at
    sample ``i`` is the weighted mean of raw curvature over the overlap of
    those cells and ``[s[i]-window/2, s[i]+window/2]``, clipped to the segment.
    Thus weights are lengths in metres, duplicate terminal cells have zero
    weight, and no sample outside the current direction segment contributes.
    A zero window explicitly disables smoothing.
    """

    if smoothing_window_m == 0.0:
        return curvature.copy()
    edges = _sample_support_edges(s_m)
    result = np.empty_like(curvature)
    half_window = 0.5 * smoothing_window_m
    segment_start = float(s_m[0])
    segment_end = float(s_m[-1])
    for index, center in enumerate(s_m):
        lower = max(segment_start, float(center) - half_window)
        upper = min(segment_end, float(center) + half_window)
        overlap_lower = np.maximum(edges[:-1], lower)
        overlap_upper = np.minimum(edges[1:], upper)
        weights = np.maximum(0.0, overlap_upper - overlap_lower)
        total_weight = float(np.sum(weights))
        if total_weight > 0.0 and math.isfinite(total_weight):
            result[index] = float(np.dot(curvature, weights) / total_weight)
        else:
            result[index] = curvature[index]
    # A duplicate terminal schema row has no arc-length support of its own.
    # Keep its final value as the adjacent smoothed value as well as inheriting
    # the adjacent finite raw derivative above.
    positive_intervals = np.flatnonzero(np.diff(s_m) > 0.0)
    if positive_intervals.size:
        duplicate_start = int(positive_intervals[-1]) + 2
        if duplicate_start < result.shape[0]:
            result[duplicate_start:] = result[duplicate_start - 1]
    if not np.all(np.isfinite(result)):
        raise ValueError("curvature smoothing produced non-finite values")
    return result


def _immutable_copy(value: object, *, dtype: np.dtype, name: str) -> np.ndarray:
    """Copy one profile array and make the copy read-only."""

    try:
        result = np.array(value, dtype=dtype, copy=True)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} has an invalid array representation") from error
    result.setflags(write=False)
    return result


def _validate_numeric_source(
    value: object, *, name: str, require_integer: bool = False
) -> np.ndarray:
    """Reject lossy numeric sources before NumPy performs any coercion."""

    try:
        source = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a numeric array") from error
    if source.dtype.kind in ("b", "O", "S", "U"):
        raise ValueError(f"{name} must be a numeric array")
    if np.iscomplexobj(source):
        raise ValueError(f"{name} must not contain complex values")
    if not np.issubdtype(source.dtype, np.number):
        raise ValueError(f"{name} must be a numeric array")
    if source.size and not np.all(np.isfinite(source)):
        raise ValueError(f"{name} must contain finite values")
    if require_integer and source.size:
        as_float = np.asarray(source, dtype=np.float64)
        int_limits = np.iinfo(np.int64)
        # ``float(int_limits.max)`` rounds to 2**63, which is outside the
        # representable int64 range.  Use the preceding float instead so an
        # out-of-range source is rejected before NumPy's int cast can warn.
        max_exact_int = np.nextafter(
            np.float64(int_limits.max), -np.inf
        )
        if (
            not np.all(np.isfinite(as_float))
            or np.any(as_float < int_limits.min)
            or np.any(as_float > max_exact_int)
            or not np.all(np.equal(as_float, np.floor(as_float)))
        ):
            raise ValueError(f"{name} must contain integer values")
    return source


@dataclass(frozen=True)
class SpatialReferenceProfile:
    """Immutable geometry and curvature sampled at planner waypoints.

    ``s_m`` is cumulative Euclidean polyline length in waypoint order.  A
    terminal duplicate point is retained and gets the adjacent finite
    curvature value.  ``curvature_1_m`` is smoothed per direction segment by
    a centered, Voronoi-cell-weighted 0.40 m arc-length window by default.
    """

    xy: np.ndarray
    yaw_unwrapped: np.ndarray
    s_m: np.ndarray
    curvature_1_m: np.ndarray
    segment_id: np.ndarray
    drive_direction: np.ndarray
    main_command_rad_s: np.ndarray

    def __post_init__(self) -> None:
        _validate_numeric_source(self.xy, name="xy")
        _validate_numeric_source(self.yaw_unwrapped, name="yaw_unwrapped")
        _validate_numeric_source(self.s_m, name="s_m")
        _validate_numeric_source(self.curvature_1_m, name="curvature_1_m")
        _validate_numeric_source(
            self.segment_id, name="segment_id", require_integer=True
        )
        _validate_numeric_source(
            self.main_command_rad_s, name="main_command_rad_s"
        )
        try:
            direction_source = np.asarray(self.drive_direction)
        except (TypeError, ValueError) as error:
            raise ValueError("drive_direction must be a string array") from error
        if direction_source.dtype.kind not in ("O", "S", "U"):
            raise ValueError("drive_direction must be a string array")
        if any(
            not isinstance(value, str) or value not in _DIRECTIONS
            for value in direction_source.reshape(-1)
        ):
            raise ValueError("drive_direction must contain only forward or reverse")

        xy = _immutable_copy(self.xy, dtype=np.dtype(np.float64), name="xy")
        yaw = _immutable_copy(
            self.yaw_unwrapped, dtype=np.dtype(np.float64), name="yaw_unwrapped"
        )
        s_m = _immutable_copy(self.s_m, dtype=np.dtype(np.float64), name="s_m")
        curvature = _immutable_copy(
            self.curvature_1_m,
            dtype=np.dtype(np.float64),
            name="curvature_1_m",
        )
        segment_id = _immutable_copy(
            self.segment_id, dtype=np.dtype(np.int64), name="segment_id"
        )
        drive_direction = _immutable_copy(
            self.drive_direction,
            dtype=np.dtype("<U7"),
            name="drive_direction",
        )
        main_command = _immutable_copy(
            self.main_command_rad_s,
            dtype=np.dtype(np.float64),
            name="main_command_rad_s",
        )

        count = int(xy.shape[0]) if xy.ndim >= 1 else 0
        if xy.ndim != 2 or xy.shape[1] != 2 or count < 2:
            raise ValueError("xy must have shape (N, 2) with at least two points")
        for array, name in (
            (yaw, "yaw_unwrapped"),
            (s_m, "s_m"),
            (curvature, "curvature_1_m"),
            (segment_id, "segment_id"),
            (drive_direction, "drive_direction"),
            (main_command, "main_command_rad_s"),
        ):
            if array.ndim != 1 or array.shape[0] != count:
                raise ValueError(f"{name} must have one value per xy point")
        if not np.all(np.isfinite(xy)):
            raise ValueError("xy must contain finite values")
        if not np.all(np.isfinite(yaw)):
            raise ValueError("yaw_unwrapped must contain finite values")
        if not np.all(np.isfinite(s_m)) or np.any(np.diff(s_m) < 0.0):
            raise ValueError("s_m must be finite and monotonic")
        if not np.all(np.isfinite(curvature)):
            raise ValueError("curvature_1_m must contain finite values")
        if not np.all(np.isfinite(main_command)):
            raise ValueError("main_command_rad_s must contain finite values")
        if np.any(segment_id < 0) or int(segment_id[0]) != 0:
            raise ValueError("segment_id must start at zero and be non-negative")
        if np.any(np.diff(segment_id) < 0) or np.any(np.diff(segment_id) > 1):
            raise ValueError("segment_id must be contiguous and non-decreasing")
        if any(str(value) not in _DIRECTIONS for value in drive_direction):
            raise ValueError("drive_direction must contain only forward or reverse")
        expected_segment_id = _segment_ids(
            tuple(str(value) for value in drive_direction)
        )
        if not np.array_equal(segment_id, expected_segment_id):
            raise ValueError("segment_id must match drive_direction runs")
        if np.any(
            ((main_command > 0.0) & (drive_direction != "forward"))
            | ((main_command < 0.0) & (drive_direction != "reverse"))
        ):
            raise ValueError("main_command_rad_s sign must match drive_direction")
        if float(s_m[-1]) <= float(s_m[0]):
            raise ValueError("profile needs meaningful positive arc-length support")

        object.__setattr__(self, "xy", xy)
        object.__setattr__(self, "yaw_unwrapped", yaw)
        object.__setattr__(self, "s_m", s_m)
        object.__setattr__(self, "curvature_1_m", curvature)
        object.__setattr__(self, "segment_id", segment_id)
        object.__setattr__(self, "drive_direction", drive_direction)
        object.__setattr__(self, "main_command_rad_s", main_command)

    @classmethod
    def from_waypoints(
        cls,
        waypoints: Iterable[KinodynamicWaypoint],
        smoothing_window_m: float = _DEFAULT_SMOOTHING_WINDOW_M,
    ) -> "SpatialReferenceProfile":
        """Build a profile from planner-style waypoint records.

        The input records are read as scalar values and are never modified.
        Direction changes start a new independent geometry segment.  A zero
        smoothing window is allowed for inspecting raw finite differences.
        """

        smoothing = _finite_float(
            smoothing_window_m, name="smoothing_window_m"
        )
        if smoothing < 0.0:
            raise ValueError("smoothing_window_m must be non-negative")
        try:
            waypoint_tuple = tuple(waypoints)
        except TypeError as error:
            raise ValueError("waypoints must be an iterable of KinodynamicWaypoint") from error
        if len(waypoint_tuple) < 2:
            raise ValueError("at least two waypoints are required")
        if not all(isinstance(item, KinodynamicWaypoint) for item in waypoint_tuple):
            raise ValueError("waypoints must contain KinodynamicWaypoint values")

        xy_values: list[tuple[float, float]] = []
        yaw_values: list[float] = []
        directions: list[str] = []
        commands: list[float] = []
        for index, waypoint in enumerate(waypoint_tuple):
            x = _finite_float(waypoint.x_m, name=f"waypoints[{index}].x_m")
            y = _finite_float(waypoint.y_m, name=f"waypoints[{index}].y_m")
            yaw = _finite_float(waypoint.yaw_rad, name=f"waypoints[{index}].yaw_rad")
            command = _finite_float(
                waypoint.main_command_rad_s,
                name=f"waypoints[{index}].main_command_rad_s",
            )
            _finite_float(
                waypoint.auxiliary_angle_rad,
                name=f"waypoints[{index}].auxiliary_angle_rad",
            )
            direction = waypoint.drive_direction
            if not isinstance(direction, str) or direction not in _DIRECTIONS:
                raise ValueError(
                    f"waypoints[{index}].drive_direction must be forward or reverse"
                )
            if (command > 0.0 and direction != "forward") or (
                command < 0.0 and direction != "reverse"
            ):
                raise ValueError(
                    f"waypoints[{index}] command sign does not match drive_direction"
                )
            xy_values.append((x, y))
            yaw_values.append(yaw)
            directions.append(str(direction))
            commands.append(command)

        xy = np.asarray(xy_values, dtype=np.float64)
        yaw_raw = np.asarray(yaw_values, dtype=np.float64)
        direction_array = np.asarray(directions, dtype="<U7")
        command_array = np.asarray(commands, dtype=np.float64)
        deltas = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        if not np.all(np.isfinite(deltas)):
            raise ValueError("waypoint arc lengths must be finite")
        s_m = np.concatenate((np.asarray([0.0]), np.cumsum(deltas)))
        if not np.all(np.isfinite(s_m)) or np.any(np.diff(s_m) < 0.0):
            raise ValueError("waypoint arc length must be finite and monotonic")
        segment_id = _segment_ids(tuple(directions))

        yaw_unwrapped = np.empty_like(yaw_raw)
        curvature_raw = np.empty_like(yaw_raw)
        for start, end in _segment_ranges(segment_id):
            segment_s = s_m[start : end + 1]
            segment_xy = xy[start : end + 1]
            segment_yaw = np.unwrap(yaw_raw[start : end + 1])
            segment_intervals = np.diff(segment_s)
            if start == end:
                # A planner can make a direction switch after one moving
                # sample.  It carries metadata but has no local derivative;
                # keep it isolated at exactly zero curvature.
                if not np.all(np.isfinite(segment_xy)):
                    raise ValueError("segment coordinates must be finite")
                yaw_unwrapped[start] = segment_yaw[0]
                curvature_raw[start] = 0.0
                continue
            if not np.any(segment_intervals > 0.0):
                raise ValueError(
                    f"direction segment {int(segment_id[start])} needs positive arc-length support"
                )
            last_positive = int(np.flatnonzero(segment_intervals > 0.0)[-1])
            if np.any(segment_intervals[:last_positive] <= 0.0):
                raise ValueError(
                    "arc length must increase within a segment except terminal duplicates"
                )
            # ``segment_xy`` is intentionally touched here to keep the
            # geometry/arc-length relationship explicit at the segment gate.
            if not np.all(np.isfinite(segment_xy)):
                raise ValueError("segment coordinates must be finite")
            yaw_unwrapped[start : end + 1] = segment_yaw
            curvature_raw[start : end + 1] = _raw_segment_curvature(
                segment_s, segment_yaw
            )

        curvature = np.empty_like(curvature_raw)
        for start, end in _segment_ranges(segment_id):
            curvature[start : end + 1] = _smooth_segment_curvature(
                s_m[start : end + 1],
                curvature_raw[start : end + 1],
                smoothing_window_m=smoothing,
            )

        return cls(
            xy=xy,
            yaw_unwrapped=yaw_unwrapped,
            s_m=s_m,
            curvature_1_m=curvature,
            segment_id=segment_id,
            drive_direction=direction_array,
            main_command_rad_s=command_array,
        )

    def preview_max_abs_curvature(self, index: int, distance_m: float) -> float:
        """Return max ``|curvature|`` ahead, bounded by the current segment."""

        if isinstance(index, (bool, np.bool_)) or not isinstance(
            index, (int, np.integer)
        ):
            raise TypeError("index must be an integer")
        point_index = int(index)
        if point_index < 0 or point_index >= self.s_m.shape[0]:
            raise ValueError("index is outside the profile")
        distance = _finite_float(distance_m, name="distance_m")
        if distance < 0.0:
            raise ValueError("distance_m must be non-negative")
        current_segment = int(self.segment_id[point_index])
        segment_indices = np.flatnonzero(self.segment_id == current_segment)
        segment_end = int(segment_indices[-1])
        upper_s = min(float(self.s_m[point_index]) + distance, float(self.s_m[segment_end]))
        candidates = np.arange(point_index, segment_end + 1, dtype=np.int64)
        candidates = candidates[self.s_m[candidates] <= upper_s]
        if candidates.size == 0:
            raise ValueError("preview query has no current-segment samples")
        value = float(np.max(np.abs(self.curvature_1_m[candidates])))
        if not math.isfinite(value):
            raise ValueError("preview curvature is non-finite")
        return value


@dataclass(frozen=True)
class DelayAwareReferenceQuery:
    """A bounded local reference query for delay-aware tracking."""

    index: int
    projection_s_m: float
    delay_distance_m: float
    query_s_m: float
    curvature_1_m: float


def query_delay_aware_reference(
    profile: SpatialReferenceProfile,
    *,
    current_index: int,
    projection_s_m: float | None = None,
    speed_m_s: float,
    auxiliary_latency_s: float,
    profile_window_m: float,
    maximum_delay_distance_m: float,
) -> DelayAwareReferenceQuery:
    """Query local curvature after a bounded, same-segment phase lead.

    ``projection_s_m`` is an optional continuous projection along the current
    direction segment.  When omitted, the historical waypoint arc length at
    ``current_index`` is used.  An explicit projection is validated against
    the complete current segment before any lead is applied, so a sparse
    segment or a direction cusp cannot leak the query into the next run.
    """

    if isinstance(current_index, (bool, np.bool_)) or not isinstance(
        current_index, (int, np.integer)
    ):
        raise TypeError("current_index must be an integer")
    point_index = int(current_index)
    if point_index < 0 or point_index >= profile.s_m.shape[0]:
        raise ValueError("current_index is outside the profile")

    try:
        speed = _finite_float(speed_m_s, name="speed_m_s")
        latency = _finite_float(auxiliary_latency_s, name="auxiliary_latency_s")
        window = _finite_float(profile_window_m, name="profile_window_m")
        maximum_delay = _finite_float(
            maximum_delay_distance_m, name="maximum_delay_distance_m"
        )
    except ValueError as error:
        raise ValueError("delay-query inputs must be finite and nonnegative") from error
    if latency < 0.0 or window < 0.0 or maximum_delay < 0.0:
        raise ValueError("delay-query inputs must be finite and nonnegative")
    if maximum_delay > _MAX_DELAY_DISTANCE_M:
        raise ValueError("maximum_delay_distance_m must not exceed 0.30 m")

    segment = int(profile.segment_id[point_index])
    same_segment = np.flatnonzero(profile.segment_id == segment)
    segment_start = int(same_segment[0])
    segment_end = int(same_segment[-1])
    if projection_s_m is None:
        projection_s = float(profile.s_m[point_index])
    else:
        if isinstance(projection_s_m, (bool, np.bool_)):
            raise ValueError("projection_s_m must be finite and within the current segment")
        try:
            projection_s = _finite_float(projection_s_m, name="projection_s_m")
        except ValueError as error:
            raise ValueError(
                "projection_s_m must be finite and within the current segment"
            ) from error
        segment_start_s = float(profile.s_m[segment_start])
        segment_end_s = float(profile.s_m[segment_end])
        if projection_s < segment_start_s or projection_s > segment_end_s:
            raise ValueError(
                "projection_s_m must be finite and within the current segment"
            )
    delay_distance = min(abs(speed) * latency, maximum_delay)
    query_s = min(
        projection_s + delay_distance,
        float(profile.s_m[segment_end]),
    )

    # An explicit continuous projection can lie between sparse waypoints.
    # Aggregate over the complete same-direction segment so the centered
    # window is still spatially local without ever crossing its cusp.  Keep
    # the historical current-index slice when no continuous projection was
    # supplied, preserving Task 1's original API behavior.
    local_start = segment_start if projection_s_m is not None else point_index
    local_s = profile.s_m[local_start : segment_end + 1]
    local_curvature = profile.curvature_1_m[local_start : segment_end + 1]
    half_window = 0.5 * window
    lower = max(float(local_s[0]), query_s - half_window)
    upper = min(float(local_s[-1]), query_s + half_window)
    edges = _sample_support_edges(local_s)
    overlap_lower = np.maximum(edges[:-1], lower)
    overlap_upper = np.minimum(edges[1:], upper)
    weights = np.maximum(0.0, overlap_upper - overlap_lower)
    with np.errstate(over="ignore", invalid="ignore"):
        total_weight = float(np.sum(weights))
    if not math.isfinite(total_weight):
        raise ValueError("delay-query curvature aggregation is non-finite")
    if total_weight > 0.0:
        with np.errstate(over="ignore", invalid="ignore"):
            curvature = float(np.dot(local_curvature, weights) / total_weight)
    else:
        curvature = float(np.interp(query_s, local_s, local_curvature))
    if not math.isfinite(curvature):
        raise ValueError("delay-query curvature aggregation is non-finite")
    index = local_start + int(np.searchsorted(local_s, query_s, side="left"))
    index = min(index, segment_end)
    return DelayAwareReferenceQuery(
        index=index,
        projection_s_m=projection_s,
        delay_distance_m=float(delay_distance),
        query_s_m=float(query_s),
        curvature_1_m=curvature,
    )


def curvature_blend_weight(
    curvature_1_m: float,
    *,
    blend_start_1_m: float,
    blend_full_1_m: float,
) -> float:
    """Return a symmetric curvature magnitude blend weight in ``[0, 1]``."""

    try:
        curvature = _finite_float(curvature_1_m, name="curvature_1_m")
        blend_start = _finite_float(blend_start_1_m, name="blend_start_1_m")
        blend_full = _finite_float(blend_full_1_m, name="blend_full_1_m")
    except ValueError as error:
        raise ValueError("curvature blend thresholds must be finite") from error
    if blend_start < 0.0 or blend_full <= blend_start:
        raise ValueError("blend_full_1_m must exceed blend_start_1_m")
    return float(
        np.clip(
            (abs(curvature) - blend_start) / (blend_full - blend_start),
            0.0,
            1.0,
        )
    )


def blend_tracking_curvature(
    pure_pursuit: float,
    profile: float,
    feedback: float,
    *,
    weight: float,
) -> float:
    """Blend Pure Pursuit with local profile curvature and feedback."""

    try:
        pure = _finite_float(pure_pursuit, name="pure_pursuit")
        profile_curvature = _finite_float(profile, name="profile")
        feedback_curvature = _finite_float(feedback, name="feedback")
        blend_weight = _finite_float(weight, name="weight")
    except ValueError as error:
        raise ValueError("curvature blend inputs must be finite and bounded") from error
    if not 0.0 <= blend_weight <= 1.0:
        raise ValueError("curvature blend inputs must be finite and bounded")
    result = (1.0 - blend_weight) * pure + blend_weight * (
        profile_curvature + feedback_curvature
    )
    if not math.isfinite(result):
        raise ValueError("curvature blend inputs must be finite and bounded")
    return float(result)
