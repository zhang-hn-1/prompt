"""Measured positive-height partial surfaces under an explicit flat-floor prior."""
import numpy as np
from partial_surface_memory import PartialSurfaceMemory

class GenericHeightSurfaceMemory(PartialSurfaceMemory):
    def update(self, packet, left_class, right_class, now, input_hashes):
        # Reject causal violations before accessing any physical payload.
        t=float(packet['time_s'])
        if not np.isfinite(now) or not np.isfinite(t) or t>now or t<self._last_time:
            raise ValueError('current/past monotonic source clock required before payload')
        w=np.asarray(packet['world'])
        if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():
            raise ValueError('finite measured world points')
        keep=(w[:,2]>.05)&(w[:,2]<=1.5)
        filtered=dict(packet)
        for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):
            a=np.asarray(packet[k])
            if len(a)!=len(w):raise ValueError('aligned physical source records')
            filtered[k]=a[keep]
        # RGB colour masks are diagnostic only; actual pixel-domain identity remains checked.
        all_pixels=np.ones((480,640),dtype=bool)
        record=super().update(filtered,all_pixels,all_pixels,now,input_hashes)
        record.update(flat_floor_height_is_prior=True,no_colour_selector=True)
        self._records[-1].update(flat_floor_height_is_prior=True,no_colour_selector=True)
        return record
