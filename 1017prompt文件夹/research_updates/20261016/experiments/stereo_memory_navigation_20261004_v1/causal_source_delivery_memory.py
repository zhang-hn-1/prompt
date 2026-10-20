"""Distinct immutable acquisition and availability clocks, no future payload reads."""
import numpy as np
from tentative_point_memory import TentativePointMemory
class CausalSourceDeliveryMemory:
 def __init__(self):
  self._memory=TentativePointMemory();self._available={};self._last_available=-np.inf;self._last_now=-np.inf;self._frame=-1;self._birth=-np.inf
 def deliver(self,packet,*,available_time_s,now):
  a=float(available_time_s);n=float(now)
  if not np.isfinite(a) or not np.isfinite(n) or a>n or a<self._last_available or n<self._last_now:
   raise ValueError('past finite monotonic availability before packet read')
  b=float(packet['time_s'])
  if not np.isfinite(b) or b>a:raise ValueError('acquisition cannot follow availability')
  self._memory.update(packet,n)
  frame=int(packet['frame']);self._available[frame]=a;self._last_available=a;self._last_now=n;self._frame=frame;self._birth=b
 def snapshot(self,now):
  n=float(now)
  if not np.isfinite(n) or n<self._last_now:raise ValueError('monotonic consumer clock')
  out=self._memory.at(n);self._last_now=n
  for r in out['records']:r['available_time_s']=self._available[r['frame']]
  out.update(source_frame=self._frame,source_time_s=self._birth,available_time_s=None if self._frame<0 else self._last_available,availability_not_acquisition=True)
  return out
