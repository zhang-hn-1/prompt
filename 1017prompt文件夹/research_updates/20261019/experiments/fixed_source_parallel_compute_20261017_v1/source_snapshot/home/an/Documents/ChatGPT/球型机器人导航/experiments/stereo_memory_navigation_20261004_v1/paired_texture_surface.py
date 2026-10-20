"""Necessary current-pair texture support, no new depth/matching/shape prediction."""
import copy
import numpy as np
from generic_height_surface import GenericHeightSurfaceMemory
from audit_generic_height_binocular_support_v1 import measure

class PairedTextureSurfaceMemory(GenericHeightSurfaceMemory):
    def update(self, packet, left_rgb, right_rgb, now, input_hashes, disparity_native):
        t=float(packet['time_s'])
        if not np.isfinite(now) or not np.isfinite(t) or t>now or t<self._last_time:
            raise ValueError('current/past monotonic source clock required before payload')
        if any(np.asarray(a).shape!=(480,640,3) or np.asarray(a).dtype!=np.uint8 for a in [left_rgb,right_rgb]):
            raise ValueError('current paired RGB required')
        d=np.asarray(disparity_native)
        if d.shape!=(480,640):raise ValueError('native disparity geometry')
        ids=np.asarray(packet['query_pixel_id'])
        if not np.issubdtype(ids.dtype,np.integer) or (ids<0).any() or (ids>=640*480).any():raise ValueError('paired query pixels')
        if not np.isfinite(d.ravel()[ids]).all():raise ValueError('finite original query disparity')
        record=super().update(packet,None,None,now,input_hashes)
        q=record['query_pixel_id'];x=q%640;y=q//640
        if not np.isfinite(d[y,x]).all():raise ValueError('finite selected disparity')
        support=measure(left_rgb,right_rgb,np.column_stack([x,y]),d[y,x])
        keep=(support['left_std']>=2)&(support['right_std']>=2)&support['patch_inside'].astype(bool)
        for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:record[k]=record[k][keep].copy()
        record.update(paired_texture_supported=True,texture_support_not_matching_certificate=True)
        self._records[-1]=copy.deepcopy(record)
        return record
