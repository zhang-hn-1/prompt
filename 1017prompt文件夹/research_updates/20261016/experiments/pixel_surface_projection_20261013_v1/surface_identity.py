"""OFFLINE supervision only; authored-surface compatibility, not online semantics."""
import numpy as np
from scipy.spatial import cKDTree

def categories(world,scene):
    p=np.asarray(world,float).reshape(-1,3);obs=np.zeros(len(p),bool)
    finite=np.isfinite(p).all(1)
    if not finite.all():raise ValueError('finite labelled points only')
    tree=cKDTree(p)
    for s in scene['shapes']:
        radius=float(s['radius']);bound=radius if s['type']=='Sphere' else np.hypot(radius,s['height']/2)
        ids=np.asarray(tree.query_ball_point(s['centre'],bound+.03),int)
        if not len(ids):continue
        q=(p[ids]-s['centre'])@np.asarray(s['rotation'])
        if s['type']=='Sphere':dist=np.abs(np.linalg.norm(q,axis=1)-radius)
        elif s['type']=='Cylinder':
            delta=np.stack([np.linalg.norm(q[:,:2],axis=1)-radius,np.abs(q[:,2])-s['height']/2],1)
            sd=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0)
            dist=np.abs(sd)
        else:raise ValueError('unqualified scene type')
        obs[ids[dist<=.03]]=True
    floor=(np.abs(p[:,2]-scene['ground_z_m'])<=.03)
    # Scene ground extent is authored where present; otherwise runtime adds known floor.
    lo=np.asarray(scene['ground_xy_min_m']);hi=np.asarray(scene['ground_xy_max_m'])
    floor&=(p[:,:2]>=lo).all(1)&(p[:,:2]<=hi).all(1)
    out=np.zeros(len(p),np.uint8);out[floor&~obs]=1;out[obs&~floor]=2
    return out,dict(ground=int((floor&~obs).sum()),obstacle=int((obs&~floor).sum()),ambiguous=int((floor&obs).sum()),unsupported=int((~floor&~obs).sum()))
