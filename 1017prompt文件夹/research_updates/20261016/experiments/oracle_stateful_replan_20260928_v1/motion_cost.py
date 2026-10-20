"""Frozen per-plan opportunity cost; not a dynamic braking predictor."""
import math
import numpy as np

def reversal_cost(speed, direction, model):
    if not math.isfinite(speed) or speed < 0 or direction not in ('forward','reverse'):
        raise ValueError('finite nonnegative speed and drive direction required')
    samples=sorted((s for s in model.braking_samples if s.drive_direction==direction),key=lambda s:s.speed_m_s)
    stop_time=float(np.interp(speed,[0.,*[s.speed_m_s for s in samples]],[0.,*[s.stop_time_s for s in samples]]))
    return .5+speed*(stop_time+.5)
