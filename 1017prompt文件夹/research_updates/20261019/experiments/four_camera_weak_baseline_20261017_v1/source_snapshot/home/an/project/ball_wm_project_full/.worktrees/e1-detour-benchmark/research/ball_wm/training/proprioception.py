"""Versioned velocity coordinates; raw sensor records remain world-frame."""
from __future__ import annotations

from typing import Mapping
import numpy as np


def validate_velocity_frame(frame: str) -> str:
    if frame not in ('world', 'base'):
        raise ValueError('proprio_velocity_frame must be world or base')
    return frame


def checkpoint_velocity_frame(checkpoint: Mapping) -> str:
    """Unlabelled historical weights retain their original world inputs."""
    config = checkpoint.get('config', {})
    if not isinstance(config, Mapping):
        raise ValueError('checkpoint config must declare a valid proprio_velocity_frame')
    frame = validate_velocity_frame(config.get('proprio_velocity_frame', 'world'))
    declared = validate_velocity_frame(checkpoint.get('proprio_velocity_frame', frame))
    if declared != frame:
        raise ValueError('checkpoint proprio_velocity_frame conflicts with its config')
    return frame


def transform_proprio_velocities(proprio, quaternion_wxyz, *, frame: str = 'world') -> np.ndarray:
    """Rotate only linear/angular velocity using each row's measured attitude.

    Quaternions are world-from-base. Their transpose maps both three-vectors
    to base; the remaining eleven entries already have their legacy semantics.
    Nothing is modified in place and no future or shared terminal attitude is
    substituted for an individual historical sample.
    """
    validate_velocity_frame(frame)
    result = np.array(proprio, dtype=np.float32, copy=True)
    if result.ndim < 1 or result.shape[-1] != 17:
        raise ValueError('proprio must have shape (..., 17)')
    if frame == 'world':
        return result
    quat = np.asarray(quaternion_wxyz, dtype=np.float64)
    if quat.shape != result.shape[:-1] + (4,) or not np.isfinite(quat).all():
        raise ValueError('quaternion history must be finite and match proprio rows (..., 4)')
    norms = np.linalg.norm(quat, axis=-1, keepdims=True)
    if (norms <= np.finfo(np.float64).eps).any():
        raise ValueError('quaternion must have non-zero norm')
    if not np.isfinite(result).all():
        raise ValueError('proprio must contain finite measurements')
    w, x, y, z = np.moveaxis(quat / norms, -1, 0)
    rotation = np.stack([
        np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], axis=-1),
        np.stack([2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], axis=-1),
        np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], axis=-1),
    ], axis=-2)
    for start in (0, 3):
        result[..., start:start+3] = np.einsum('...ji,...j->...i', rotation, result[..., start:start+3])
    return result
