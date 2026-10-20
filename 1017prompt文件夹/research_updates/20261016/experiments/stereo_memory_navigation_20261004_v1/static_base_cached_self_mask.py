"""Cache exact joint-independent CAD base, never dynamic joints or source points."""
import numpy as np
from dynamic_self_mask import RobotSelfMask
class StaticBaseCachedSelfMask(RobotSelfMask):
 def __init__(self):
  super().__init__()
  self._static_base=super().project(np.zeros(3),parts=('base_link',),include_frozen=False)
  for a in self._static_base.values():a.setflags(write=False)
 def project(self,joints,parts=('base_link','link1','link2','link3'),include_frozen=True):
  q=np.asarray(joints,float)
  if q.shape!=(3,) or not np.isfinite(q).all():raise ValueError('finite three current joints')
  if tuple(parts)!=('base_link','link1','link2','link3'):
   return super().project(q,parts=parts,include_frozen=include_frozen)
  current=super().project(q,parts=('link1','link2','link3'),include_frozen=False)
  out={eye:current[eye]|self._static_base[eye] for eye in ('left','right')}
  if include_frozen:out={eye:out[eye]|self.frozen[eye] for eye in out}
  return out
