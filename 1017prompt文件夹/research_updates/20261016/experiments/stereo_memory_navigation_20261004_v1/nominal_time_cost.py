"""Static V1 speed-weighted distance cost; no physical deadline prediction."""
import math

def nominal_edge_cost(length, speed, maximum_speed):
    length, speed, maximum_speed = map(float, (length, speed, maximum_speed))
    if not all(map(math.isfinite, (length, speed, maximum_speed))):
        raise ValueError("nominal cost inputs must be finite")
    if length < 0 or speed < 0 or maximum_speed <= 0 or speed > maximum_speed + 1e-12:
        raise ValueError("invalid nominal cost length or speed range")
    return length * maximum_speed / max(.05, speed)
