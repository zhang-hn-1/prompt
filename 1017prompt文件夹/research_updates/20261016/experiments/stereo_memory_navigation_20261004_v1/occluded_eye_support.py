"""Unobservable is distinct from usable RGB class-negative; MODEL support only."""
import numpy as np

def combine_support(positive,unobservable):
 p=np.asarray(positive,bool);u=np.asarray(unobservable,bool)
 if p.ndim!=3 or p.shape[-1]!=2 or p.shape!=u.shape:raise ValueError('voxel,view,eye arrays required')
 old=p.any(axis=1).all(axis=1)&(p.any(axis=2).sum(axis=1)>=2)
 left=(p[:,:,0].sum(axis=1)>=2)&u[:,:,1].all(axis=1)
 right=(p[:,:,1].sum(axis=1)>=2)&u[:,:,0].all(axis=1)
 return old|left|right
