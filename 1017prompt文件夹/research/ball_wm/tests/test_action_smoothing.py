"""Isaac-free tests for the shared ball action execution policy."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from action_smoothing import (  # noqa: E402
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    DEFAULT_MAIN_DERIVATIVE_GAIN,
    ActionSmoother,
    default_articulation_gains,
)


def _bounds() -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([-2.0, -math.radians(30.0)], dtype=np.float32),
        np.asarray([2.0, math.radians(30.0)], dtype=np.float32),
    )


def test_default_control_profile_matches_accepted_probe_parameters():
    assert DEFAULT_MAIN_ACCELERATION_RAD_S2 == pytest.approx(0.5)
    assert DEFAULT_AUXILIARY_KP == pytest.approx(300.0)
    assert DEFAULT_AUXILIARY_KD == pytest.approx(60.0)
    assert DEFAULT_AUXILIARY_RATE_RAD_S == pytest.approx(math.radians(30.0))
    assert DEFAULT_MAIN_DERIVATIVE_GAIN == pytest.approx(80.0)

    kps, kds = default_articulation_gains()

    np.testing.assert_allclose(kps, [0.0, 300.0, 0.0])
    np.testing.assert_allclose(kds, [80.0, 60.0, 0.0])


def test_action_smoother_limits_main_acceleration_and_auxiliary_rate():
    minimum, maximum = _bounds()
    smoother = ActionSmoother(
        dt_s=0.02,
        max_main_acceleration_rad_s2=0.5,
        max_auxiliary_rate_rad_s=math.radians(30.0),
    )

    first = smoother.step([0.5, math.radians(30.0)], minimum, maximum)
    second = smoother.step([0.5, math.radians(30.0)], minimum, maximum)

    np.testing.assert_allclose(first, [0.01, math.radians(0.6)])
    np.testing.assert_allclose(second, [0.02, math.radians(1.2)])


def test_action_smoother_clips_targets_and_never_overshoots_bounds():
    minimum, maximum = _bounds()
    smoother = ActionSmoother(dt_s=0.02)

    for _ in range(1000):
        applied = smoother.step([99.0, -99.0], minimum, maximum)

    np.testing.assert_allclose(applied, maximum * np.asarray([1.0, -1.0]))
    assert np.all(applied >= minimum)
    assert np.all(applied <= maximum)


def test_action_smoother_reset_starts_from_saved_applied_action():
    minimum, maximum = _bounds()
    smoother = ActionSmoother(dt_s=0.02)
    saved = np.asarray([0.2, math.radians(10.0)], dtype=np.float32)

    smoother.reset(saved)
    applied = smoother.step([0.5, math.radians(30.0)], minimum, maximum)

    np.testing.assert_allclose(applied, [0.21, math.radians(10.6)])


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dt_s": 0.0},
        {"dt_s": 0.02, "max_main_acceleration_rad_s2": 0.0},
        {"dt_s": 0.02, "max_auxiliary_rate_rad_s": -1.0},
    ],
)
def test_action_smoother_rejects_nonpositive_configuration(kwargs):
    with pytest.raises(ValueError, match="positive"):
        ActionSmoother(**kwargs)
