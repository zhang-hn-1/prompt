"""Positive-height source qualification using existing per-point range sensitivity.

The one-pixel sigma is an approximation, not a calibrated confidence interval.
Ambiguous new points supply no occupied/free assertion; existing risks persist.
"""
import copy
import numpy as np
from paired_texture_surface import PairedTextureSurfaceMemory

class HeightIntervalSurfaceMemory(PairedTextureSurfaceMemory):
    def update(self,packet,left_rgb,right_rgb,now,input_hashes,disparity_native):
        t=float(packet['time_s'])
        if not np.isfinite(now) or not np.isfinite(t) or t>now or t<self._last_time:
            raise ValueError('current/past monotonic source clock required before payload')
        world=np.asarray(packet['world']);sigma=np.asarray(packet['sigma_m'])
        if world.ndim!=2 or world.shape[1]!=3 or sigma.shape!=(len(world),) or not np.isfinite(sigma).all() or (sigma<0).any():
            raise ValueError('finite nonnegative aligned range uncertainty required')
        record=super().update(packet,left_rgb,right_rgb,now,input_hashes,disparity_native)
        # Existing scalar Euclidean range sensitivity bounds its vertical component.
        # Keep the same flat-floor prior .05m; no new fitted threshold or multiplier.
        keep=record['world'][:,2]-record['sigma_m']>.05
        ambiguous=int((~keep).sum())
        for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):
            record[k]=record[k][keep].copy()
        record.update(height_interval_ambiguous_count=ambiguous,
                      height_interval_qualification=True,
                      uncertainty_not_calibrated=True,
                      ambiguous_is_unknown_not_free=True)
        self._records[-1]=copy.deepcopy(record)
        return record
