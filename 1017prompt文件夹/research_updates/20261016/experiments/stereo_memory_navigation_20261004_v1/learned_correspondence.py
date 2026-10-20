"""Geometry of frozen learned pairs, separate from network and offline labels."""
import numpy as np
from feature_tracks import causal_sources,epipolar_distances,triangulate

def supported_features(features,mask):
    mask=np.asarray(mask,bool);xy=np.asarray(features['xy'],float)
    desc=np.asarray(features['desc']);scores=np.asarray(features['scores'])
    if xy.ndim!=2 or xy.shape[1]!=2 or desc.shape!=(len(xy),256) or scores.shape!=(len(xy),):raise ValueError('feature shape')
    if not all(np.isfinite(a).all() for a in [xy,desc,scores]):raise ValueError('nonfinite features')
    kept=[]
    for i,(px,py) in enumerate(xy):
        x,y=np.rint([px,py]).astype(int);radius=72
        if x-radius>=0 and y-radius>=0 and x+radius<mask.shape[1] and y+radius<mask.shape[0] and not mask[y-radius:y+radius+1,x-radius:x+radius+1].any():kept.append(i)
    ix=np.array(kept,int)
    return dict(xy=xy[ix].copy(),desc=desc[ix].copy(),scores=scores[ix].copy(),original_indices=ix)

def predict_correspondences(reference,sources,K,pair_provider):
    if not np.isfinite(float(reference['time_s'])):raise ValueError('nonfinite reference time')
    sources=causal_sources(reference,sources);q=np.asarray(reference['xy'],float);pools=[[] for _ in q];stats=[]
    for s in sources:
        pairs=np.asarray(pair_provider(reference,s))
        if pairs.size==0:pairs=np.empty((0,2),int)
        if pairs.ndim!=2 or pairs.shape[1]!=2 or not np.isfinite(pairs).all() or not np.equal(pairs,np.rint(pairs)).all():raise ValueError('invalid pair array')
        pairs=pairs.astype(int)
        if len(pairs) and (pairs.min()<0 or pairs[:,0].max()>=len(q) or pairs[:,1].max()>=len(s['xy']) or len(np.unique(pairs[:,0]))!=len(pairs) or len(np.unique(pairs[:,1]))!=len(pairs)):raise ValueError('invalid pair indices')
        distances=np.diag(epipolar_distances(q[pairs[:,0]],np.asarray(s['xy'])[pairs[:,1]],reference['pose'],s['pose'],K)) if len(pairs) else np.empty(0)
        geometrically_valid=0
        for (i,j),distance in zip(pairs,distances):
            if distance>1.5:continue
            r=triangulate(q[i],s['xy'][j],reference['pose'],s['pose'],K)
            if r is None or not np.isfinite(r['sigma_m']) or r['sigma_m']>.1:continue
            r.update(query=[reference['frame'],reference['eye'],int(i),reference['time_s']],source=[s['frame'],s['eye'],int(j),s['time_s']]);pools[i].append(r);geometrically_valid+=1
        stats.append(dict(source=[s['frame'],s['eye'],s['time_s']],learned_pairs=pairs.tolist(),epipolar_px=distances.tolist(),geometrically_valid=geometrically_valid))
    accepted=[];conflicts=0
    for pool in pools:
        if not pool:continue
        if any(np.linalg.norm(np.array(a['world'])-b['world'])>.1 for a in pool for b in pool):conflicts+=1;continue
        accepted.append(max(pool,key=lambda r:r['parallax_deg']))
    return dict(query_xy=q.tolist(),query_count=len(q),points=accepted,source_views=[[s['frame'],s['eye'],s['time_s']] for s in sources],conflicting_queries=conflicts,pair_diagnostics=stats,certified_free=False)
