"""Explicit policy-intent and protocol-request semantics for Round 6."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from training.action_protocol import ActionProtocol


DirectionLabel = str


@dataclass(frozen=True)
class ActionRequestLayers:
    """One policy request before and after the formal action protocol."""

    raw_component: tuple[float, float]
    protocol_component: tuple[float, float]
    raw_block: np.ndarray
    protocol_block: np.ndarray
    source: str
    clipped: bool
    clip_reason: str | None


def classify_signed_direction(value: object, *, deadband: float) -> DirectionLabel:
    """Classify a signed scalar without conflating stopped and unavailable."""

    try:
        scalar = float(value)
        threshold = float(deadband)
    except (TypeError, ValueError, OverflowError):
        return "unknown"
    if not math.isfinite(scalar) or not math.isfinite(threshold) or threshold < 0.0:
        return "unknown"
    if abs(scalar) <= threshold:
        return "near_zero"
    return "forward" if scalar > 0.0 else "reverse"


def normalize_action_request(
    protocol: ActionProtocol,
    *,
    main_velocity: float,
    auxiliary_angle: float,
    source: str,
) -> ActionRequestLayers:
    """Normalize raw intent through the same ``ActionProtocol.clip_block`` as runtime."""

    if not isinstance(source, str) or not source:
        raise ValueError("source must be a non-empty string")
    raw_component = np.asarray((main_velocity, auxiliary_angle), dtype=np.float32)
    if raw_component.shape != (2,) or not np.all(np.isfinite(raw_component)):
        raise ValueError("raw action component must contain two finite values")
    raw_block = np.tile(raw_component, protocol.controls_per_block).astype(np.float32)
    protocol_block = protocol.clip_block(raw_block)
    clipped_axes = np.flatnonzero(raw_component != protocol_block[:2])
    reasons: list[str] = []
    for axis in clipped_axes:
        name = "main" if int(axis) == 0 else "auxiliary"
        bound = "below_minimum" if raw_component[axis] < protocol_block[axis] else "above_maximum"
        reasons.append(f"{name}_{bound}")
    raw_block.setflags(write=False)
    protocol_block.setflags(write=False)
    return ActionRequestLayers(
        raw_component=(float(raw_component[0]), float(raw_component[1])),
        protocol_component=(float(protocol_block[0]), float(protocol_block[1])),
        raw_block=raw_block,
        protocol_block=protocol_block,
        source=source,
        clipped=bool(reasons),
        clip_reason="+".join(reasons) if reasons else None,
    )

