"""Adapt existing exact static-base CAD memoization to the fixed accepted four rig.
Only joint-independent base is cached; dynamic bodies use every measured joint.
"""
from pathlib import Path
import sys,time
O=Path(__file__).resolve().parents[1]/'fixed_source_parallel_compute_20261017_v1';sys.path.insert(0,str(O))
from parallel_source import ParallelFourSGBM,legacy
np=legacy.np
class CachedBaseParallelSGBM(ParallelFourSGBM):
 def __init__(self):
  start=time.monotonic();super().__init__();v=self.cad.triangles['base_link'];base=v@np.eye(3).T+np.zeros(3)
  self.static={n:legacy.mask_triangles((base-T[:3,3])@(T[:3,:3]@legacy.AX),640,480,legacy.F,319.5,239.5) for n,T in zip(legacy.NAMES,legacy.LOCAL)}
  for a in self.static.values():a.setflags(write=False)
  self.initialization_wall_s=time.monotonic()-start
 def masks(self,joints):
  bodies=self.cad.body_transforms(joints)
  tri=np.concatenate([v@bodies[n][:3,:3].T+bodies[n][:3,3] for n,v in self.cad.triangles.items() if n!='base_link'])
  return {n:self.static[n]|legacy.mask_triangles((tri-T[:3,3])@(T[:3,:3]@legacy.AX),640,480,legacy.F,319.5,239.5) for n,T in zip(legacy.NAMES,legacy.LOCAL)}
