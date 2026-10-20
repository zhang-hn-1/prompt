"""Frozen request primitives and physical short-horizon evidence rules."""

from __future__ import annotations

import numpy as np


PRIMITIVES = {
    "forward_straight": (2.0, 0.0),
    "forward_negative": (2.0, -np.pi / 6),
    "forward_positive": (2.0, np.pi / 6),
    "reverse_straight": (-2.0, 0.0),
    "reverse_negative": (-2.0, -np.pi / 6),
    "reverse_positive": (-2.0, np.pi / 6),
    "stop": (0.0, 0.0),
}
HORIZON_S = 6.0


def primitive_block(name: str, *, controls_per_block: int) -> np.ndarray:
    if name not in PRIMITIVES or controls_per_block != 5:
        raise ValueError("expected frozen primitive and formal five-control block")
    return np.tile(np.asarray(PRIMITIVES[name], dtype=np.float32), controls_per_block)


def local_outcome(duration_s: float, displacement_m: float, *,
                  any_contact: bool, measurement_available: bool) -> str:
    if not measurement_available:
        return "insufficient"
    if any_contact:
        return "contact"
    if duration_s < HORIZON_S - 1e-5:
        return "insufficient"
    return "survived_moving" if displacement_m >= 0.25 else "survived_no_motion"


def same_start(first: dict, second: dict, *, atol: float = 1e-6) -> bool:
    if first.keys() != second.keys():
        return False
    return all(np.asarray(first[key]).shape == np.asarray(second[key]).shape
               and np.allclose(first[key], second[key], atol=atol, rtol=0,
                               equal_nan=False)
               for key in first)
