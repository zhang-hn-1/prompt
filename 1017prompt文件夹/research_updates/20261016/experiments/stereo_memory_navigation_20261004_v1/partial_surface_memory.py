"""Immutable causal partial positive surfaces; absent space stays unknown."""
import copy
import numpy as np

class PartialSurfaceMemory:
    def __init__(self):
        self._records = []
        self._last_time = -np.inf

    def update(self, packet, left_class, right_class, now, input_hashes):
        t = float(packet['time_s'])
        if not np.isfinite(now) or not np.isfinite(t) or t > now or t < self._last_time:
            raise ValueError('current/past monotonic source clock required before payload')
        world = np.asarray(packet['world'])
        q, r = np.asarray(packet['query_pixel_id']), np.asarray(packet['partner_pixel_id'])
        if world.ndim != 2 or world.shape[1] != 3 or not np.isfinite(world).all():
            raise ValueError('finite measured world points')
        masks = [np.asarray(a).reshape(-1) for a in (left_class, right_class)]
        if any(a.dtype != bool for a in masks):
            raise ValueError('boolean current RGB masks')
        if any(ids.shape != (len(world),) or not np.issubdtype(ids.dtype, np.integer) for ids in (q, r)):
            raise ValueError('paired pixel identities')
        if any(np.any(ids < 0) or np.any(ids >= len(mask)) for ids, mask in zip((q, r), masks)):
            raise ValueError('pixel outside exposure')
        selected = masks[0][q] & masks[1][r]
        record = {k: np.asarray(packet[k])[selected].copy() for k in ('world', 'sigma_m', 'query_pixel_id', 'partner_pixel_id')}
        record.update(frame=int(packet['frame']), acquisition_time_s=t, input_hashes=copy.deepcopy(input_hashes), provisional_occupied_surface=True, certified_free=False, unseen_volume_observed=False)
        self._last_time = t
        self._records.append(copy.deepcopy(record))
        return record

    def at(self, now):
        if not np.isfinite(now):
            raise ValueError('finite query time')
        return copy.deepcopy([r for r in self._records if r['acquisition_time_s'] <= now])
