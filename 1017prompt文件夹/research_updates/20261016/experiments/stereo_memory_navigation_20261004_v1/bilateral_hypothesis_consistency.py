"""One readonly agreement cue. It does not certify either predicted point."""
import numpy as np

def bilateral_hypothesis_pairs(left, right, left_usd, right_usd, now):
    for packet in (left, right):
        t = float(packet['current_time_s'])
        if not np.isfinite(t) or not np.isfinite(now) or abs(t-now)>1e-6:
            raise ValueError('same current exposure required before payload')
    if int(left['current_frame']) != int(right['current_frame']):
        raise ValueError('same exposure frame required before payload')
    world = [np.asarray(p['world'],float) for p in (left,right)]
    uv = [np.asarray(p['current_uv'],int) for p in (left,right)]
    for w,p in zip(world,uv):
        if w.shape!=(len(p),3) or p.shape!=(len(w),2) or not np.isfinite(w).all():
            raise ValueError('finite world/query arrays required')
        if (p%2).any() or (p[:,0]<0).any() or (p[:,0]>=640).any() or (p[:,1]<0).any() or (p[:,1]>=480).any():
            raise ValueError('native even lattice required')
        if len(np.unique(p[:,1]*640+p[:,0]))!=len(p):
            raise ValueError('unique retained query required')
    def project(w,pose):
        T=np.asarray(pose,float)
        if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('finite USD pose required')
        optical=(w-T[:3,3])@(T[:3,:3]@np.diag([1.,-1.,-1.]))
        z=optical[:,2];out=np.full((len(w),2),np.nan)
        positive=z>0
        out[positive]=optical[positive,:2]/z[positive,None]*584.8888888888889+[319.5,239.5]
        return out,positive
    ab,pos_ab=project(world[0],right_usd);ba,pos_ba=project(world[1],left_usd)
    lookup={tuple(p):i for i,p in enumerate(uv[1])}
    pairs=[];gaps=[]
    for i in np.flatnonzero(pos_ab):
        nearest=tuple((np.rint(ab[i]/2)*2).astype(int))
        j=lookup.get(nearest)
        if j is None or not pos_ba[j]:continue
        if not np.array_equal(np.rint(ba[j]/2)*2,uv[0][i]):continue
        gap=float(np.linalg.norm(world[0][i]-world[1][j]))
        if gap<=.1:pairs.append((i,j));gaps.append(gap)
    return np.asarray(pairs,int).reshape(-1,2),np.asarray(gaps,float)
