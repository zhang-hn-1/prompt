"""Controlled validity repair: local data, not regularizer alone, must select depth."""
import numpy as np
from reference import decode

def evidence_decode(total,cost,support,disparities):
    c=np.asarray(cost,dtype=np.float64);m=np.asarray(support);a=np.asarray(total);ds=np.asarray(disparities)
    if c.shape!=a.shape or c.ndim!=3 or m.shape!=c.shape or m.dtype!=bool or len(ds)!=c.shape[-1]:raise ValueError('same HWD costs/support/aggregate and D disparities required')
    if np.any(~np.isfinite(c[m])) or np.any(c[m]<0):raise ValueError('supported unary cost finite nonnegative required')
    original=decode(a,ds);observed=np.where(m,c,np.inf);lo=observed.min(axis=-1);idx=observed.argmin(axis=-1)
    unique=(np.isfinite(observed)&(observed==lo[...,None])).sum(axis=-1)==1
    alternatives=m.sum(axis=-1)>=2
    keep=np.isfinite(original)&unique&alternatives&(original==ds[idx])
    return np.where(keep,original,np.nan).astype(np.float32)
