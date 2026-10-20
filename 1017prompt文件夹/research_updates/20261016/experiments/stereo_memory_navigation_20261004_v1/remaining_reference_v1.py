"""Read-only XY geometry; nearest projection is not certified temporal progress."""
import numpy as np
def project_remaining(reference_xy,root_xy):
 p=np.asarray(reference_xy,float);q=np.asarray(root_xy,float)
 if p.ndim!=2 or p.shape[1]!=2 or not len(p) or q.shape!=(2,) or not np.isfinite(p).all() or not np.isfinite(q).all():raise ValueError('finite nonempty XY reference and root')
 if len(p)==1:return dict(progress_m=0.,distance_m=float(np.linalg.norm(q-p[0])),remaining_xy=p.tolist(),ambiguous_progress=False,tied_progress_m=[0.],segment_index=0)
 delta=np.diff(p,axis=0);length=np.linalg.norm(delta,axis=1);s=np.r_[0.,np.cumsum(length)];n=length**2
 t=np.divide(np.sum((q-p[:-1])*delta,axis=1),n,out=np.zeros_like(n),where=n>1e-20);t=np.clip(t,0,1);projection=p[:-1]+t[:,None]*delta;distance=np.linalg.norm(projection-q,axis=1)
 tied=np.flatnonzero(distance<=distance.min()+1e-9);progress=s[:-1]+t*length;ordered=sorted(tied,key=lambda i:(progress[i],int(i)));i=int(ordered[0]);unique=[]
 for value in sorted(progress[tied]):
  if not unique or value-unique[-1]>1e-8:unique.append(float(value))
 remaining=np.vstack([projection[i],p[i+1:]])
 return dict(progress_m=float(progress[i]),distance_m=float(distance[i]),remaining_xy=remaining.tolist(),ambiguous_progress=len(unique)>1,tied_progress_m=unique,segment_index=i)
