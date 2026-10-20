"""Isolated camera-placement probe; does not change the frozen benchmark rig."""

import numpy as np


def front_narrow_positions() -> tuple[np.ndarray, np.ndarray]:
    return (
        np.array([0.35, 0.10, 0.10], dtype=np.float64),
        np.array([0.35, -0.10, 0.10], dtype=np.float64),
    )


def front_external_positions() -> tuple[np.ndarray, np.ndarray]:
    return (
        np.array([0.48, 0.10, 0.10], dtype=np.float64),
        np.array([0.48, -0.10, 0.10], dtype=np.float64),
    )
