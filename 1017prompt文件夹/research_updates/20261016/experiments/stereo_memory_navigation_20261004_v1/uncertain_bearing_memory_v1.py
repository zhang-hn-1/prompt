"""Past image evidence that cannot populate measured occupied/free geometry."""
import copy
import numpy as np
from bearing_observability_v1 import build_context
from feature_tracks import rotation

class UncertainBearingMemory:
    def __init__(self):
        self._time=-np.inf;self._records={}

    def update(self,context,now,pose_sigma_m):
        now=float(now);sigma=float(pose_sigma_m)
        if not np.isfinite(now) or now<self._time:raise ValueError('monotonic finite consumer clock')
        if context.get('observed_depth') is not False or context.get('role')!='current_2D_bearings':raise ValueError('uncertain current image context only')
        p=build_context(context['xy'],context['pose'],context['K'],context['time_s'],context['frame'])
        if p['time_s']>now:raise ValueError('future capture forbidden')
        if not np.isfinite(sigma) or sigma<0:raise ValueError('finite nonnegative current pose uncertainty')
        # All validation precedes mutation. Replaying a packet cannot refresh its capture age.
        self._time=now;self._records={k:v for k,v in self._records.items() if now-v['capture_time_s']<=10.}
        key=(p['frame'],p['time_s'])
        if now-p['time_s']>10. or key in self._records:return
        rays=np.c_[p['xy'],np.ones(len(p['xy']))]@np.linalg.inv(p['K']).T
        rays=rays@rotation(p['pose']).T;norm=np.linalg.norm(rays,axis=1);rays=rays/norm[:,None]
        eligible=sigma<=.05
        self._records[key]=dict(frame=p['frame'],eye='left',capture_time_s=p['time_s'],query_xy=p['xy'].tolist(),pose_sigma_m=sigma,world_ray_available=bool(eligible),world_ray_origin=p['pose'][:3,3].tolist() if eligible else None,world_ray_directions=rays.tolist() if eligible else None,observed_depth=False,certified_free=False,role='uncertain_2D_bearing_history',range_prior_not_measurement=True)

    def snapshot(self,now):
        now=float(now)
        if not np.isfinite(now) or now<self._time:raise ValueError('query precedes accepted consumer update')
        return copy.deepcopy([v for v in self._records.values() if now-v['capture_time_s']<=10.])
