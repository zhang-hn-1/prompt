"""Exact-state repeatability guard, deliberately not a general dynamics model."""
import numpy as np

def replay_prediction(expected_start,current_start,template_xy):
    if expected_start != current_start:
        raise ValueError('Full start state mismatch: template cannot be used')
    return np.asarray(template_xy).copy()
