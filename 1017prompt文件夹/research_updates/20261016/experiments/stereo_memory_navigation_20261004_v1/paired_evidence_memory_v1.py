"""Read-only dual provenance memory; no occupancy, free-space or action interface."""
import copy,math
from tentative_point_memory import TentativePointMemory
class PairedEvidenceMemory:
 def __init__(self):
  self._provisional=TentativePointMemory();self._unknown=TentativePointMemory();self._now=-float('inf')
 def update(self,provisional,unknown,now):
  if not math.isfinite(now) or now<self._now:raise ValueError('finite monotonic consumer clock')
  clocks=[]
  for p,role in [(provisional,'image_supported_provisional'),(unknown,'UNKNOWN_unsupported')]:
   t=float(p['time_s']);f=p['frame']
   if not math.isfinite(t) or t>now:raise ValueError('future acquisition before payload')
   if p['evidence_role']!=role or any(bool(p[k]) for k in ['certified_free','source_model_admitted','measured_geometry_admitted']):raise ValueError('provisional roles cannot claim measured/free admission')
   clocks.append((t,f))
  if clocks[0]!=clocks[1]:raise ValueError('paired same epoch')
  a=copy.deepcopy(self._provisional);b=copy.deepcopy(self._unknown)
  a.update(provisional,now);b.update(unknown,now)
  self._provisional=a;self._unknown=b;self._now=now
 def at(self,now):
  if not math.isfinite(now) or now<self._now:raise ValueError('finite monotonic query')
  self._now=now;a=self._provisional.at(now);b=self._unknown.at(now)
  for layer,role in [(a,'image_supported_provisional'),(b,'UNKNOWN_unsupported')]:
   for row in layer['records']:row['evidence_role']=role
  return dict(provisional=a,unknown=b,source_admitted=False,measured_geometry_admitted=False,certified_free=False,no_action_API=True)
