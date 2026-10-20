"""Current reference geometry, retaining equal-distance projection ambiguity."""
import numpy as np

def project(path,point):
 p=np.asarray(path,float);x=np.asarray(point,float)
 if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or x.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(x).all():raise ValueError('finite complete polyline and current XY')
 edges=np.diff(p,axis=0);length=np.linalg.norm(edges,axis=1);cumulative=np.r_[0,np.cumsum(length)];nonzero=length>0
 if not nonzero.any():raise ValueError('nonzero reference required')
 alpha=np.zeros(len(edges));alpha[nonzero]=np.clip(np.sum((x-p[:-1])[nonzero]*edges[nonzero],axis=1)/length[nonzero]**2,0,1)
 closest=p[:-1]+alpha[:,None]*edges;dist=np.linalg.norm(closest-x,axis=1);best=float(dist[nonzero].min());ids=np.flatnonzero(nonzero & (np.abs(dist-best)<=1e-10));arcs=cumulative[ids]+alpha[ids]*length[ids]
 low=float(arcs.min());high=float(arcs.max())
 return dict(distance_m=best,arc_min_m=low,arc_max_m=high,ambiguous=bool(high-low>1e-10),total_arc_m=float(cumulative[-1]),endpoint_distance_m=float(np.linalg.norm(x-p[-1])),equal_distance_segment_indices=ids.tolist())
