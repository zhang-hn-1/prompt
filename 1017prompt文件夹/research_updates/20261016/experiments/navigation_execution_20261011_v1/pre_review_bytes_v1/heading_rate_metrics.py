"""Frozen navigation gates for this one independent feedback candidate."""
import math

def navigation_gates(b0,candidate,*,changed,integrity):
    keys=['net_goal_axis_progress_m','cross_rms_m','cross_peak_m','wall_median_s','wall_p90_s']
    if not all(math.isfinite(float(arm[k])) for arm in (b0,candidate) for k in keys):
        raise ValueError('finite measured navigation metrics required')
    return dict(integrity=bool(integrity),actual_request_change=bool(changed),
        no_contact=bool(candidate['raw_contacts_zero']),arrival=bool(candidate['arrival']),
        net_progress=candidate['net_goal_axis_progress_m']>=b0['net_goal_axis_progress_m'],
        cross_rms=candidate['cross_rms_m']<=max(.2,1.2*b0['cross_rms_m']),
        cross_peak=candidate['cross_peak_m']<=max(.4,1.2*b0['cross_peak_m']),
        latency=candidate['wall_median_s']<=1.5*b0['wall_median_s'] and candidate['wall_p90_s']<=1.5*b0['wall_p90_s'])
