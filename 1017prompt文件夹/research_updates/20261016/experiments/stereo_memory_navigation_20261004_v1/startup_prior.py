"""Explicit immutable task assumption, never an observed-floor map or 3D certificate."""
import numpy as np
class ImmutableStartupPrior:
 def __init__(self,cells,active_from_s,*,source,trusted):
  c=np.asarray(cells);t=float(active_from_s)
  if source!='task_assumption' or trusted is not True or not np.isfinite(t) or c.ndim!=2 or c.shape[1]!=2 or not np.isfinite(c).all() or not np.equal(c,np.floor(c)).all():raise ValueError('explicit finite trusted task assumption required')
  self._cells=frozenset(map(tuple,c.astype(int)));self._time=t
 def evaluate(self,labels,cells,time_s):
  t=float(time_s);a=np.asarray(labels);c=np.asarray(cells)
  if not np.isfinite(t) or a.ndim!=2 or c.ndim!=2 or c.shape[1]!=2 or not np.isfinite(c).all() or not np.equal(c,np.floor(c)).all():raise ValueError('invalid support query')
  counts=dict(observed_floor_cells=0,prior_support_cells=0,unknown_cells=0,occupied_cells=0)
  for i,j in c.astype(int):
   inside=0<=i<a.shape[0] and 0<=j<a.shape[1]
   if not inside:counts['unknown_cells']+=1
   elif a[i,j]>0:counts['occupied_cells']+=1
   elif a[i,j]==0:counts['observed_floor_cells']+=1
   elif t>=self._time and (i,j) in self._cells:counts['prior_support_cells']+=1
   else:counts['unknown_cells']+=1
  return dict(**counts,checked_cells=len(c),necessary_support_passed=counts['unknown_cells']==0 and counts['occupied_cells']==0,prior_is_observation=False,full_3d_certified=False,certified_free=False,action_authorized=False)
