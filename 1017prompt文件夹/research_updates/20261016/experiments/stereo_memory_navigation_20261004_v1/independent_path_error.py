"""Independent post-close geometry metric; never a control projection."""
import numpy as np
def distance_to_complete_reference(xy,reference):
 xy=np.asarray(xy,float);r=np.asarray(reference,float)
 if xy.shape!=(2,) or r.ndim!=2 or r.shape[1]!=2 or len(r)<2 or not np.isfinite(xy).all() or not np.isfinite(r).all():raise ValueError('finite complete XY polyline required')
 a=r[:-1];d=r[1:]-a;den=np.sum(d*d,axis=1);t=np.divide(np.sum((xy-a)*d,axis=1),den,out=np.zeros_like(den),where=den>0);closest=a+np.clip(t,0,1)[:,None]*d
 return float(np.linalg.norm(closest-xy,axis=1).min())
