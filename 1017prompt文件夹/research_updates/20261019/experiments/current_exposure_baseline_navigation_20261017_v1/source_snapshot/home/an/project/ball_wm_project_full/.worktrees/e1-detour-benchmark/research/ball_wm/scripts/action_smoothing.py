"""Shared execution policy for the ball robot's two-field action protocol.

The learned policy still emits bounded target actions.  This module applies the
same physical command shaping in collection and closed-loop evaluation: main
joint velocity is acceleration-limited and auxiliary steering position is
rate-limited.  Keeping it Isaac-free makes the behavior directly testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

import numpy as np


DEFAULT_MAIN_ACCELERATION_RAD_S2 = 0.5
DEFAULT_MAIN_DERIVATIVE_GAIN = 80.0
DEFAULT_AUXILIARY_KP = 300.0
DEFAULT_AUXILIARY_KD = 60.0
DEFAULT_AUXILIARY_RATE_RAD_S = math.radians(30.0)


@dataclass(frozen=True)
class ActionSmootherSnapshot:
    """Complete cloneable state used by the production smoother update."""

    current_action: tuple[float, float]
    last_smoothed_target: tuple[float, float]


def _finite_vector(value: object, *, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (2,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite vector with shape (2,)")
    return vector


def slew_limited_target(
    current: float,
    target: float,
    maximum_rate: float,
    dt_s: float,
) -> float:
    """Move a scalar command toward its target without overshoot."""

    values = np.asarray([current, target, maximum_rate, dt_s], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("slew-limit inputs must be finite")
    if maximum_rate <= 0.0 or dt_s <= 0.0:
        raise ValueError("maximum_rate and dt_s must be positive")
    maximum_delta = maximum_rate * dt_s
    return float(current + np.clip(target - current, -maximum_delta, maximum_delta))


def velocity_limited_position_target(
    *,
    requested_target_rad: float,
    actual_position_rad: float,
    actual_velocity_rad_s: float,
    maximum_velocity_rad_s: float,
    dt_s: float,
) -> float:
    """Hold a target only while the measured joint is outside its speed envelope.

    The action protocol remains position-target based and its established
    target-rate limiter remains active during normal motion.  This feedback
    step is deliberately a supervisor: if Isaac reports an actual auxiliary
    speed above the configured limit, hold the current position for this tick
    so the position drive's damping can reduce the speed before tracking
    resumes.  The measured velocity is still logged for post-run validation.
    """

    values = np.asarray(
        [
            requested_target_rad,
            actual_position_rad,
            actual_velocity_rad_s,
            maximum_velocity_rad_s,
            dt_s,
        ],
        dtype=np.float64,
    )
    if not np.isfinite(values).all():
        raise ValueError("velocity-limit inputs must be finite")
    if maximum_velocity_rad_s <= 0.0:
        raise ValueError("maximum_velocity_rad_s must be positive")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")

    # If Isaac has already exceeded the envelope, stop asking the position
    # drive to move farther in the current direction.  The derivative gain
    # then damps the measured motion before the target resumes moving.
    if abs(actual_velocity_rad_s) > maximum_velocity_rad_s:
        return float(actual_position_rad)

    return float(requested_target_rad)


def default_articulation_gains() -> tuple[np.ndarray, np.ndarray]:
    """Return the gains validated by the auxiliary steering probe."""

    return (
        np.asarray([0.0, DEFAULT_AUXILIARY_KP, 0.0], dtype=np.float32),
        np.asarray(
            [DEFAULT_MAIN_DERIVATIVE_GAIN, DEFAULT_AUXILIARY_KD, 0.0],
            dtype=np.float32,
        ),
    )


@dataclass
class ActionSmoother:
    """Apply the accepted physical command limits to one action per physics step."""

    dt_s: float
    max_main_acceleration_rad_s2: float = DEFAULT_MAIN_ACCELERATION_RAD_S2
    max_auxiliary_rate_rad_s: float = DEFAULT_AUXILIARY_RATE_RAD_S
    max_auxiliary_velocity_rad_s: float | None = None
    _current_action: np.ndarray = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64), init=False, repr=False
    )
    _last_smoothed_target: np.ndarray = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64), init=False, repr=False
    )

    def __post_init__(self) -> None:
        values = np.asarray(
            [self.dt_s, self.max_main_acceleration_rad_s2, self.max_auxiliary_rate_rad_s],
            dtype=np.float64,
        )
        if not np.isfinite(values).all() or np.any(values <= 0.0):
            raise ValueError("action smoothing configuration must be positive and finite")
        if self.max_auxiliary_velocity_rad_s is not None:
            velocity = float(self.max_auxiliary_velocity_rad_s)
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise ValueError(
                    "max_auxiliary_velocity_rad_s must be positive and finite or None"
                )

    @property
    def current_action(self) -> np.ndarray:
        """Return the last applied action without exposing mutable internal state."""

        return self._current_action.copy()

    @property
    def last_smoothed_target(self) -> np.ndarray:
        """Return the slew-limited target before the velocity supervisor."""

        return self._last_smoothed_target.copy()

    def reset(self, applied_action: Sequence[float] | None = None) -> None:
        """Reset the filter, optionally preserving a restored state's applied action."""

        if applied_action is None:
            self._current_action.fill(0.0)
            self._last_smoothed_target.fill(0.0)
            return
        self._current_action = _finite_vector(applied_action, name="applied_action")
        self._last_smoothed_target = self._current_action.copy()

    def snapshot(self) -> ActionSmootherSnapshot:
        return ActionSmootherSnapshot(
            current_action=tuple(map(float, self._current_action)),
            last_smoothed_target=tuple(map(float, self._last_smoothed_target)),
        )

    def restore(self, snapshot: ActionSmootherSnapshot) -> None:
        if not isinstance(snapshot, ActionSmootherSnapshot):
            raise TypeError("snapshot must be ActionSmootherSnapshot")
        self._current_action = _finite_vector(
            snapshot.current_action, name="snapshot.current_action"
        )
        self._last_smoothed_target = _finite_vector(
            snapshot.last_smoothed_target, name="snapshot.last_smoothed_target"
        )

    def clone(self) -> "ActionSmoother":
        cloned = ActionSmoother(
            dt_s=self.dt_s,
            max_main_acceleration_rad_s2=self.max_main_acceleration_rad_s2,
            max_auxiliary_rate_rad_s=self.max_auxiliary_rate_rad_s,
            max_auxiliary_velocity_rad_s=self.max_auxiliary_velocity_rad_s,
        )
        cloned.restore(self.snapshot())
        return cloned

    def step(
        self,
        target_action: Sequence[float],
        action_minimum: Sequence[float],
        action_maximum: Sequence[float],
        *,
        auxiliary_position_rad: float | None = None,
        auxiliary_velocity_rad_s: float | None = None,
    ) -> np.ndarray:
        """Clip a target and return its physically limited action for this step."""

        target = _finite_vector(target_action, name="target_action")
        minimum = _finite_vector(action_minimum, name="action_minimum")
        maximum = _finite_vector(action_maximum, name="action_maximum")
        if np.any(minimum > maximum):
            raise ValueError("action_minimum must not exceed action_maximum")
        target = np.clip(target, minimum, maximum)
        next_action = np.asarray(
            [
                slew_limited_target(
                    self._current_action[0],
                    target[0],
                    self.max_main_acceleration_rad_s2,
                    self.dt_s,
                ),
                slew_limited_target(
                    self._current_action[1],
                    target[1],
                    self.max_auxiliary_rate_rad_s,
                    self.dt_s,
                ),
            ],
            dtype=np.float64,
        )
        self._last_smoothed_target = np.clip(next_action, minimum, maximum)
        if self.max_auxiliary_velocity_rad_s is not None:
            if auxiliary_position_rad is None or auxiliary_velocity_rad_s is None:
                raise ValueError(
                    "measured auxiliary position and velocity are required with a velocity limit"
                )
            next_action[1] = velocity_limited_position_target(
                requested_target_rad=float(next_action[1]),
                actual_position_rad=float(auxiliary_position_rad),
                actual_velocity_rad_s=float(auxiliary_velocity_rad_s),
                maximum_velocity_rad_s=float(self.max_auxiliary_velocity_rad_s),
                dt_s=self.dt_s,
            )
        self._current_action = np.clip(next_action, minimum, maximum)
        return self._current_action.astype(np.float32, copy=True)
