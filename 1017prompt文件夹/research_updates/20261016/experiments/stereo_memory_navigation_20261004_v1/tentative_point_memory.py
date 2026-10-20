"""No action API. Immutable expiring rejected-source diagnostic surfaces.
Not admitted occupancy, free space, calibrated uncertainty or hidden volume.
"""
import copy,math,re
import numpy as np

class TentativePointMemory:
    def __init__(self):
        self._records=[];self._last_time=-np.inf;self._frames=set()
    def update(self,packet,now):
        t=float(packet['time_s']);frame=int(packet['frame'])
        if not math.isfinite(now) or not math.isfinite(t) or t>now or t<self._last_time or frame in self._frames:
            raise ValueError('causal finite unique acquisition before payload')
        world=np.asarray(packet['world']);q=np.asarray(packet['query_pixel_id']);r=np.asarray(packet['partner_pixel_id']);hashes=packet['input_sha256']
        if world.ndim!=2 or world.shape[1]!=3 or not np.isfinite(world).all():raise ValueError('finite point surfaces')
        if any(a.shape!=(len(world),) or not np.issubdtype(a.dtype,np.integer) or np.any(a<0) for a in (q,r)):raise ValueError('paired pixel provenance')
        if not isinstance(hashes,dict) or not hashes or any(not isinstance(v,str) or re.fullmatch('[0-9a-f]{64}',v) is None for v in hashes.values()):raise ValueError('source SHA256 provenance')
        row=dict(time_s=t,frame=frame,world=world.copy(),query_pixel_id=q.copy(),partner_pixel_id=r.copy(),input_sha256=copy.deepcopy(hashes),layer='TENTATIVE_RGB_DEPTH_SURFACE_SOURCE_REJECTED_DIAGNOSTIC')
        self._records.append(row);self._last_time=t;self._frames.add(frame)
    def at(self,now):
        if not math.isfinite(now) or now<self._last_time:raise ValueError('finite present query, no rewind')
        records=copy.deepcopy([r for r in self._records if 0<=now-r['time_s']<=10.])
        current=sum(len(r['world']) for r in records if r['time_s']==now);past=sum(len(r['world']) for r in records if r['time_s']<now)
        points=np.concatenate([r['world'] for r in records],axis=0) if records else np.empty((0,3))
        cells=len(np.unique(np.floor(points[:,:2]/.05).astype(np.int64),axis=0)) if len(points) else 0
        return dict(records=records,current_count=current,historical_count=past,distinct_XY_cells=cells,certified_free=False,source_admitted=False,hidden_volume_observed=False,uncertainty_calibrated=False,no_action_API=True)
