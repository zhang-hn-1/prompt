"""Linear quantile with positive infinite error mass retained, never filtered."""
import numpy as np
def quantile(values,q):
 a=np.sort(np.asarray(values,float));q=float(q)
 if a.ndim!=1 or not len(a) or np.isnan(a).any() or (a<0).any() or not np.isfinite(q) or not 0<=q<=1:raise ValueError('nonnegative error array and finite quantile')
 index=(len(a)-1)*q;lo=int(np.floor(index));hi=int(np.ceil(index));weight=index-lo
 if weight==0 or a[lo]==a[hi]:return float(a[lo])
 if np.isposinf(a[hi]):return float('inf')
 return float((1-weight)*a[lo]+weight*a[hi])
