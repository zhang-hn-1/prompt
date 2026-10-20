"""Versioned, simulator-independent drive model for the ball robot."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np


PROFILE_SCHEMA = "drive_dynamics_v1"
VELOCITY_LIMITED_PROFILE_SCHEMA = "drive_dynamics_v2"
SUPPORTED_PROFILE_SCHEMAS = frozenset((PROFILE_SCHEMA, VELOCITY_LIMITED_PROFILE_SCHEMA))
ACTION_PROTOCOL_ID = "ball_action_v2_aux30"
MAIN_COMMANDS_RAD_S = (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5)
AUXILIARY_ANGLES_RAD = tuple(
    math.radians(value) for value in (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0)
)
EXPECTED_CONTROLLER = {
    "main_acceleration_limit_rad_s2": 0.5,
    "main_derivative_gain": 80.0,
    "auxiliary_kp": 300.0,
    "auxiliary_kd": 60.0,
    "auxiliary_target_rate_limit_rad_s": math.radians(30.0),
}
_DRIVE_RESPONSE_CACHE_CAPACITY = 16_384


def _finite_float(value: object, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _direction_for_main(main_command_rad_s: float) -> str:
    if main_command_rad_s > 0.0:
        return "forward"
    if main_command_rad_s < 0.0:
        return "reverse"
    raise ValueError("main_command_rad_s must be non-zero")


def _validate_direction(value: object) -> str:
    if value not in {"forward", "reverse"}:
        raise ValueError("drive_direction must be 'forward' or 'reverse'")
    return str(value)


@dataclass(frozen=True)
class SteeringSample:
    main_command_rad_s: float
    drive_direction: str
    auxiliary_angle_rad: float
    measured_main_joint_velocity_rad_s: float
    speed_m_s: float
    curvature_1_m: float


@dataclass(frozen=True)
class BrakingSample:
    main_command_rad_s: float
    drive_direction: str
    speed_m_s: float
    stopping_distance_m: float
    stop_time_s: float


@dataclass(frozen=True)
class DriveModel:
    """Measured steering and braking response separated by drive direction."""

    measurement_timestamp_utc: str
    physics_hz: int
    controller: tuple[tuple[str, float], ...]
    steering_samples: tuple[SteeringSample, ...]
    braking_samples: tuple[BrakingSample, ...]
    auxiliary_velocity_limit_rad_s: float | None = None
    _steering_groups_by_direction: tuple[
        tuple[str, tuple[tuple[SteeringSample, ...], ...]], ...
    ] = field(init=False, repr=False, compare=False)
    _response_cache: OrderedDict[tuple[float, float], tuple[float, float]] = field(
        init=False, default_factory=OrderedDict, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        magnitudes = (0.5, 1.0, 1.5)
        groups_by_direction = tuple(
            (
                direction,
                tuple(
                    tuple(
                        sorted(
                            (
                                sample
                                for sample in self.steering_samples
                                if sample.drive_direction == direction
                                and math.isclose(
                                    abs(sample.main_command_rad_s), magnitude
                                )
                            ),
                            key=lambda sample: sample.auxiliary_angle_rad,
                        )
                    )
                    for magnitude in magnitudes
                ),
            )
            for direction in ("forward", "reverse")
        )
        object.__setattr__(self, "_steering_groups_by_direction", groups_by_direction)

    @classmethod
    def from_json(cls, path: str | Path) -> "DriveModel":
        source = Path(path)
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"could not load drive profile: {source}") from error
        return cls.from_mapping(payload)

    @classmethod
    def from_mapping(cls, payload: object) -> "DriveModel":
        if not isinstance(payload, Mapping):
            raise ValueError("drive profile must be an object")
        schema_version = payload.get("schema_version")
        if schema_version not in SUPPORTED_PROFILE_SCHEMAS:
            raise ValueError(
                "drive profile must use schema drive_dynamics_v1 or drive_dynamics_v2"
            )
        if payload.get("action_protocol_id") != ACTION_PROTOCOL_ID:
            raise ValueError(f"drive profile must use {ACTION_PROTOCOL_ID}")
        timestamp = payload.get("measurement_timestamp_utc")
        if not isinstance(timestamp, str) or not timestamp:
            raise ValueError("measurement_timestamp_utc must be non-empty")
        physics_hz = payload.get("physics_hz")
        if isinstance(physics_hz, bool) or not isinstance(physics_hz, int) or physics_hz <= 0:
            raise ValueError("physics_hz must be a positive integer")

        controller_payload = payload.get("controller")
        if not isinstance(controller_payload, Mapping):
            raise ValueError("controller metadata is required")
        controller: list[tuple[str, float]] = []
        for name, expected in EXPECTED_CONTROLLER.items():
            actual = _finite_float(controller_payload.get(name), name=f"controller.{name}")
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError("controller metadata does not match accepted settings")
            controller.append((name, actual))

        auxiliary_velocity_limit = None
        if schema_version == VELOCITY_LIMITED_PROFILE_SCHEMA:
            auxiliary_velocity_limit = _finite_float(
                controller_payload.get("auxiliary_velocity_limit_rad_s"),
                name="controller.auxiliary_velocity_limit_rad_s",
            )
            if auxiliary_velocity_limit <= 0.0:
                raise ValueError(
                    "controller.auxiliary_velocity_limit_rad_s must be positive"
                )

        steering = cls._parse_steering(payload.get("steering_samples"))
        braking = cls._parse_braking(payload.get("braking_samples"))
        return cls(
            measurement_timestamp_utc=timestamp,
            physics_hz=physics_hz,
            controller=tuple(controller),
            steering_samples=steering,
            braking_samples=braking,
            auxiliary_velocity_limit_rad_s=auxiliary_velocity_limit,
        )

    @staticmethod
    def _parse_steering(value: object) -> tuple[SteeringSample, ...]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError("steering_samples must be a sequence")
        samples: dict[tuple[float, float], SteeringSample] = {}
        for row in value:
            if not isinstance(row, Mapping):
                raise ValueError("each steering sample must be an object")
            main = _finite_float(row.get("main_command_rad_s"), name="main_command_rad_s")
            auxiliary = _finite_float(row.get("auxiliary_angle_rad"), name="auxiliary_angle_rad")
            direction = _validate_direction(row.get("drive_direction"))
            if direction != _direction_for_main(main):
                raise ValueError("drive_direction does not match main command sign")
            key = (main, auxiliary)
            if key in samples:
                raise ValueError("duplicate steering operating point")
            speed = _finite_float(row.get("speed_m_s"), name="speed_m_s")
            if speed < 0.0:
                raise ValueError("speed_m_s must be non-negative")
            samples[key] = SteeringSample(
                main_command_rad_s=main,
                drive_direction=direction,
                auxiliary_angle_rad=auxiliary,
                measured_main_joint_velocity_rad_s=_finite_float(
                    row.get("measured_main_joint_velocity_rad_s"),
                    name="measured_main_joint_velocity_rad_s",
                ),
                speed_m_s=speed,
                curvature_1_m=_finite_float(
                    row.get("curvature_1_m"), name="curvature_1_m"
                ),
            )
        expected = {
            (main, auxiliary)
            for main in MAIN_COMMANDS_RAD_S
            for auxiliary in AUXILIARY_ANGLES_RAD
        }
        if set(samples) != expected:
            raise ValueError("steering profile must contain the complete signed matrix")
        for main in MAIN_COMMANDS_RAD_S:
            curvatures = np.asarray(
                [samples[(main, auxiliary)].curvature_1_m for auxiliary in AUXILIARY_ANGLES_RAD]
            )
            if not np.all(np.diff(curvatures) < 0.0):
                raise ValueError("steering curvature must be strictly monotonic in auxiliary angle")
        return tuple(samples[key] for key in sorted(samples))

    @staticmethod
    def _parse_braking(value: object) -> tuple[BrakingSample, ...]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError("braking_samples must be a sequence")
        samples: dict[float, BrakingSample] = {}
        for row in value:
            if not isinstance(row, Mapping):
                raise ValueError("each braking sample must be an object")
            main = _finite_float(row.get("main_command_rad_s"), name="main_command_rad_s")
            if main in samples:
                raise ValueError("duplicate braking operating point")
            direction = _validate_direction(row.get("drive_direction"))
            if direction != _direction_for_main(main):
                raise ValueError("drive_direction does not match main command sign")
            speed = _finite_float(row.get("speed_m_s"), name="speed_m_s")
            distance = _finite_float(
                row.get("stopping_distance_m"), name="stopping_distance_m"
            )
            stop_time = _finite_float(row.get("stop_time_s"), name="stop_time_s")
            if min(speed, distance, stop_time) < 0.0:
                raise ValueError("braking measurements must be non-negative")
            samples[main] = BrakingSample(
                main_command_rad_s=main,
                drive_direction=direction,
                speed_m_s=speed,
                stopping_distance_m=distance,
                stop_time_s=stop_time,
            )
        if set(samples) != set(MAIN_COMMANDS_RAD_S):
            raise ValueError("braking profile must contain both drive directions")
        for direction in ("forward", "reverse"):
            rows = sorted(
                (sample for sample in samples.values() if sample.drive_direction == direction),
                key=lambda sample: sample.speed_m_s,
            )
            if any(
                right.stopping_distance_m <= left.stopping_distance_m
                for left, right in zip(rows, rows[1:])
            ):
                raise ValueError("stopping distance must increase with speed")
        return tuple(samples[key] for key in sorted(samples))

    def _steering_groups(
        self, main_command_rad_s: float
    ) -> tuple[tuple[float, ...], tuple[tuple[SteeringSample, ...], ...]]:
        main = _finite_float(main_command_rad_s, name="main_command_rad_s")
        direction = _direction_for_main(main)
        for cached_direction, groups in self._steering_groups_by_direction:
            if cached_direction == direction:
                return (0.5, 1.0, 1.5), groups
        raise AssertionError("missing cached steering groups")

    def curvature(self, main_command_rad_s: float, auxiliary_angle_rad: float) -> float:
        main = _finite_float(main_command_rad_s, name="main_command_rad_s")
        auxiliary = _finite_float(auxiliary_angle_rad, name="auxiliary_angle_rad")
        return self._response(main, auxiliary)[1]

    def _response(
        self, main_command_rad_s: float, auxiliary_angle_rad: float
    ) -> tuple[float, float]:
        key = (main_command_rad_s, auxiliary_angle_rad)
        try:
            response = self._response_cache.pop(key)
        except KeyError:
            magnitudes, groups = self._steering_groups(main_command_rad_s)
            per_speed = [
                float(
                    np.interp(
                        auxiliary_angle_rad,
                        [sample.auxiliary_angle_rad for sample in group],
                        [sample.curvature_1_m for sample in group],
                    )
                )
                for group in groups
            ]
            curvature = float(
                np.interp(abs(main_command_rad_s), magnitudes, per_speed)
            )
            per_command = [
                float(
                    np.interp(
                        auxiliary_angle_rad,
                        [sample.auxiliary_angle_rad for sample in group],
                        [sample.speed_m_s for sample in group],
                    )
                )
                for group in groups
            ]
            speed = float(
                np.interp(
                    abs(main_command_rad_s),
                    np.asarray([0.0, *magnitudes]),
                    np.asarray([0.0, *per_command]),
                )
            )
            response = (speed, curvature)
        self._response_cache[key] = response
        if len(self._response_cache) > _DRIVE_RESPONSE_CACHE_CAPACITY:
            self._response_cache.popitem(last=False)
        return response

    def speed(self, main_command_rad_s: float, auxiliary_angle_rad: float) -> float:
        main = _finite_float(main_command_rad_s, name="main_command_rad_s")
        auxiliary = _finite_float(auxiliary_angle_rad, name="auxiliary_angle_rad")
        if main == 0.0:
            return 0.0
        return self._response(main, auxiliary)[0]

    def auxiliary_for_curvature(
        self, main_command_rad_s: float, desired_curvature_1_m: float
    ) -> float:
        main = _finite_float(main_command_rad_s, name="main_command_rad_s")
        desired = _finite_float(desired_curvature_1_m, name="desired_curvature_1_m")
        magnitudes, groups = self._steering_groups(main)
        per_speed: list[float] = []
        for group in groups:
            by_curvature = sorted(group, key=lambda sample: sample.curvature_1_m)
            per_speed.append(
                float(
                    np.interp(
                        desired,
                        [sample.curvature_1_m for sample in by_curvature],
                        [sample.auxiliary_angle_rad for sample in by_curvature],
                    )
                )
            )
        return float(np.interp(abs(main), magnitudes, per_speed))

    def stopping_distance(self, speed_m_s: float, drive_direction: str) -> float:
        speed = _finite_float(speed_m_s, name="speed_m_s")
        if speed < 0.0:
            raise ValueError("speed_m_s must be non-negative")
        direction = _validate_direction(drive_direction)
        samples = sorted(
            (
                sample
                for sample in self.braking_samples
                if sample.drive_direction == direction
            ),
            key=lambda sample: sample.speed_m_s,
        )
        speeds = np.asarray([0.0, *(sample.speed_m_s for sample in samples)])
        distances = np.asarray(
            [0.0, *(sample.stopping_distance_m for sample in samples)]
        )
        if speed <= speeds[-1]:
            return float(np.interp(speed, speeds, distances))
        slope = float((distances[-1] - distances[-2]) / (speeds[-1] - speeds[-2]))
        return float(distances[-1] + slope * (speed - speeds[-1]))
