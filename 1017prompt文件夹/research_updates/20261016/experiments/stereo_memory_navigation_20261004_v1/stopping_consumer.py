"""One post-policy observed-hazard veto; no label writes or new motion model.

The radius adds a measured finite12s stopping-tail maximum to the original
.4m body+.1m runtime margin and cell half-diagonal. This is a heuristic,
NOT a proved stopping bound: higher speeds/bank/history were not covered.
Unknown floor is counted but remains explicitly permitted exploration risk.
"""
import numpy as np
from primitive_planner import assess_support
EMPIRICAL_TAIL_M=0.14088407559444638
EMPIRICAL_SPEED_PEAK_M_S=0.1637726750910607
RUNTIME_BODY_PLUS_MARGIN_M=.5


def stop_disc_filter(action,xy,labels,origin,resolution):
    proposed=np.asarray(action,dtype=np.float32)
    # A radial envelope includes lateral drift up to the measured path length;
    # it is not a forecast of the next actual trajectory.
    support=assess_support([xy],labels,origin,resolution,
        RUNTIME_BODY_PLUS_MARGIN_M+EMPIRICAL_TAIL_M)
    veto=bool(proposed.any() and support['occupied_cells']>0)
    final=np.zeros_like(proposed) if veto else proposed.copy()
    info=dict(support,stop_veto=veto,empirical_tail_m=EMPIRICAL_TAIL_M,
        certified_stopping_bound=False,
        support_status='observed_hazard' if support['occupied_cells'] else 'unknown_risk' if support['unknown_cells'] else 'observed_floor_only',
        raw_zero_is_actual_stop=False)
    return final,info


class StoppingConsumer:
    def __init__(self):self.low_count=0
    def observe_stop(self,speed_xy):
        if not np.isfinite(speed_xy) or speed_xy<0:raise ValueError('current speed required')
        self.low_count=self.low_count+1 if speed_xy<.05 else 0
        return self.low_count>=5
    def apply(self,action,xy,speed_xy,labels,origin,resolution):
        qualified=self.observe_stop(speed_xy)
        final,info=stop_disc_filter(action,xy,labels,origin,resolution)
        info.update(actual_stop_qualified=qualified,low_speed_count=self.low_count,
            beyond_empirical_speed=bool(speed_xy>EMPIRICAL_SPEED_PEAK_M_S))
        return final,info
