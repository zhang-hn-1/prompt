"""Annotation transport only. Rejected raw points and evidence roles remain unchanged."""
import copy,math,numpy as np
from paired_evidence_memory_v1 import PairedEvidenceMemory
from ground_prior_consistency_v1 import annotate_ground_prior
class PairedPriorMemory:
 def __init__(self,plane_world=(0,0,1,0)):
  plane=np.asarray(plane_world,dtype=float)
  annotate_ground_prior(dict(time_s=0.,world=np.empty((0,3))),0.,plane)
  self._plane=plane.copy();self._base=PairedEvidenceMemory();self._annotations={};self._now=-float('inf')
 def update(self,provisional,unknown,now):
  if not math.isfinite(now) or now<self._now:raise ValueError('finite monotonic consumer clock')
  for p in [provisional,unknown]:
   t=float(p['time_s'])
   if not math.isfinite(t) or t>now:raise ValueError('past acquisition before geometry')
  anns=[annotate_ground_prior(p,now,self._plane) for p in [provisional,unknown]]
  base=copy.deepcopy(self._base);base.update(provisional,unknown,now)
  frame=int(provisional['frame']);self._base=base;self._annotations[frame]=anns;self._now=now
 def at(self,now):
  s=self._base.at(now);self._now=now
  for k,layer in enumerate(['provisional','unknown']):
   for row in s[layer]['records']:
    a=self._annotations[row['frame']][k]
    if not np.array_equal(row['world'],a['world']) or row['time_s']!=a['acquisition_time_s']:raise AssertionError('annotation source identity')
    for key in ['signed_plane_distance_m','prior_inconsistent','ground_plane_world','ground_is_prior','pose_uncertainty_not_certified']:row[key]=copy.deepcopy(a[key])
  s.update(prior_annotation_is_observation=False,all_points_retained=True,plane_sign_not_source_qualification=True)
  return s
