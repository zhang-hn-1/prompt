"""Independent measured arrival hold; count endpoints, not requested angles."""
import math


def stop_hold(distances, speeds, times, *, entry_index):
    if not (len(distances) == len(speeds) == len(times)):
        raise ValueError('unaligned measurements')
    if entry_index is None:
        return None
    run = 0
    for i in range(entry_index + 1, len(times)):
        if not math.isclose(times[i] - times[i-1], .1, abs_tol=1e-5):
            run = 0
        if (math.isfinite(distances[i]) and math.isfinite(speeds[i])
                and distances[i] <= .2 and speeds[i] < .05 and times[i] <= 35.00001):
            run += 1
        else:
            run = 0
        if run >= 5:
            return float(times[i])
    return None
