"""Causal 24-channel source-feature frontend, independent of risk admission.

All actual source observations may inform features, including observations not
admitted to risk memory. That never reinserts their rejected risk records.
No labels, static maps, scene IDs, globalXY model channels, model, or files.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt
NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
PARTNER = (1, 0, 3, 2)

class FeatureTracker:
    def __init__(self):
        self.frame = -1; self.source_time = -np.inf; self.now = -np.inf
        self.origin = None; self.history = {}; self.lifetime = {}

    def update(self, frame, source_time, now, payload):
        if isinstance(frame, bool) or not isinstance(frame, (int, np.integer)) or frame <= self.frame or not np.isfinite([source_time, now]).all() or source_time > now or source_time <= self.source_time or now < self.now:
            raise ValueError('strictly new causal source before payload access')
        if set(payload) != {'rgb', 'eyes', 'body_estimated'} or set(payload['rgb']) != set(NAMES) or set(payload['eyes']) != set(NAMES):
            raise ValueError('only actual fourRGB/source/estimated-body fields accepted')
        body = np.asarray(payload['body_estimated'], float)
        if body.shape != (4, 4) or not np.isfinite(body).all(): raise ValueError('finite estimated source body')
        rgb = [np.asarray(payload['rgb'][n]) for n in NAMES]
        if not all(x.shape == (480, 640, 3) and x.dtype == np.uint8 for x in rgb): raise ValueError('fixed native fourRGB')
        checked = []
        for n in NAMES:
            a = payload['eyes'][n]
            if set(a) != {'world', 'query_pixel', 'partner_pixel', 'depth', 'disparity', 'CAD'}: raise ValueError('source-only eye schema')
            w = np.asarray(a['world']); q = np.asarray(a['query_pixel']); p = np.asarray(a['partner_pixel'])
            z = np.asarray(a['depth']); d = np.asarray(a['disparity']); cad = np.asarray(a['CAD'])
            if w.shape != (len(q), 3) or q.ndim != 1 or p.shape != q.shape or not np.issubdtype(q.dtype, np.integer) or not np.issubdtype(p.dtype, np.integer) or not np.isfinite(w).all() or (q < 0).any() or (q >= 480*640).any() or (p < 0).any() or (p >= 480*640).any(): raise ValueError('finite source point/UID fields')
            if z.shape != (480, 640) or d.shape != z.shape or cad.shape != z.shape or cad.dtype != bool: raise ValueError('source image shapes')
            if not np.isfinite(z.ravel()[q]).all() or not np.isfinite(d.ravel()[q]).all() or not ((z.ravel()[q] >= .5) & (z.ravel()[q] <= 3)).all(): raise ValueError('original selected source range')
            checked.append((w, q, p, z, d, cad))
        origin = body[:2, 3] - 20 if self.origin is None else self.origin.copy()
        vv=[]; ww=[]; ee=[]; qq=[]; pp=[]; cc=[]; total=0; outside=0
        for e, (w, q, partner, depth, disparity, cad_mask) in enumerate(checked):
            z = depth.ravel()[q]; disp = abs(disparity.ravel()[q]); total += len(w)
            gray=rgb[e].astype(float).mean(2); gy,gx=np.gradient(gray)
            grad=np.hypot(gx,gy).ravel()[q]
            mad=abs(rgb[e].reshape(-1,3)[q].astype(float)-rgb[PARTNER[e]].reshape(-1,3)[partner].astype(float)).mean(1)
            cad=distance_transform_edt(~cad_mask).ravel()[q]
            cells=np.rint((w[:,:2]-origin)/.1).astype(np.int64)
            inside=(cells>=0).all(1)&(cells<400).all(1); outside+=int((~inside).sum())
            v=np.c_[z,w[:,2],np.linalg.norm(w-body[:3,3],axis=1),disp,gray.ravel()[q],grad,mad,cad]
            vv.append(v[inside]); ww.append(w[inside]); ee.append(np.full(inside.sum(),e,int))
            qq.append(q[inside]); pp.append(partner[inside]); cc.append(cells[inside])
        vals=np.concatenate(vv); world=np.concatenate(ww); eye=np.concatenate(ee)
        uids=np.concatenate(qq); partner=np.concatenate(pp); cells=np.concatenate(cc)
        keys,inverse=np.unique(cells,axis=0,return_inverse=True)
        X=[]; staged_history={}; staged_lifetime={}; t=source_time
        for j, keyarr in enumerate(keys):
            key=tuple(keyarr); ix=inverse==j; v=vals[ix]; xyz=world[ix]; eyes=set(map(int,eye[ix]))
            lifetime=self.lifetime.get(key,0)+1; staged_lifetime[key]=lifetime
            h=[x for x in self.history.get(key,[]) if t-x['time']<=10.]; old=h.copy()
            cur=dict(time=t,z=float(v[:,0].mean()),height=float(v[:,1].mean()),gray=float(v[:,4].mean()),eyes=eyes,body=body[:3,3].copy())
            h.append(cur); staged_history[key]=h
            recenteyes=set().union(*(x['eyes'] for x in h)); span=t-h[0]['time']
            motion=max(np.linalg.norm(x['body']-body[:3,3]) for x in h)
            age=min(10.,t-old[-1]['time'])/10 if old else 1.
            f=[np.log1p(len(v)),len(eyes)/4,np.log1p(lifetime),span/10,v[:,0].mean()/3,v[:,0].std()/3,v[:,1].mean()/1.5,v[:,1].std()/1.5,v[:,2].mean()/3,v[:,3].mean()/448,v[:,3].std()/448,v[:,4].mean()/255,v[:,4].std()/255,v[:,5].mean()/255,v[:,6].mean()/255,v[:,7].mean()/640,np.linalg.norm(xyz[:,:2].std(0))/.1,np.std([x['height'] for x in h])/1.5,motion/3,np.std([x['z'] for x in h])/3,len(recenteyes)/4,np.std([x['gray'] for x in h])/255,age,len({e//2 for e in eyes})/2]
            X.append(f)
        X=np.array(X,float).reshape(-1,24)
        if not np.isfinite(X).all(): raise ValueError('nonfinite causal features; no state committed')
        self.origin=origin.copy(); self.history.update(staged_history); self.lifetime.update(staged_lifetime)
        self.frame=int(frame); self.source_time=float(source_time); self.now=float(now)
        return dict(features=X,cells=keys,origin=origin,source_eye=eye,source_query_uid=uids,
            source_partner_uid=partner,source_cell_row=inverse,original_points=total,outside_points=outside,
            time_s=source_time,model_channels_are_local_not_globalXY=True)

def interface_checks():
    x=FeatureTracker()
    class Poison:
        def __iter__(self): raise AssertionError('future payload inspected')
    try: x.update(0, 2., 1., Poison())
    except ValueError: pass
    else: raise AssertionError('future accepted')
    assert x.frame == -1 and not x.history and x.origin is None
    try: x.update(0, 0., 0., {'GT_label':1})
    except ValueError: pass
    else: raise AssertionError('label accepted')
    assert x.frame == -1 and not x.history and x.origin is None
    return dict(future_before_payload=True, teacher_schema_rejected=True, invalid_no_state_mutation=True)
