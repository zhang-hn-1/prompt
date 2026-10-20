"""One candidate: immutable measured seed -> explicitly static possible-occupancy belief.

No cylinder geometry, RGB completion, refreshed observation or certified free space.
"""
import copy
import numpy as np
from prior_rgb_navigation import PriorRGBGrid
from qualified_hazard_consumer import subset

class StaticSeedBeliefGrid(PriorRGBGrid):
 def __init__(self,*a,**k):
  super().__init__(*a,memory_mode='current',**k)
  self.belief_seed=None
  self.belief_observation_time=np.full(self.labels.shape,-np.inf)
 def seed(self,packet,now):
  t=float(now);qt=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(qt) or qt>t or t<self.time:raise ValueError('past seed clock before payload')
  frame=float(packet['frame']);width=float(packet['frame_width'])
  if not np.isfinite(frame) or frame<0 or frame!=int(frame) or abs(frame*.1-qt)>1e-6 or not np.isfinite(width) or width!=640:raise ValueError('frozen source frame/clock/width identity')
  xyz=np.asarray(packet['world'],float)
  if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():raise ValueError('finite source endpoints')
  keep=(xyz[:,2]>.05)&(xyz[:,2]<=1.5)
  candidate=subset(packet,keep)
  for key in ['sigma_m','query_pixel_id','partner_pixel_id']:
   if np.asarray(candidate[key]).shape!=(int(keep.sum()),):raise ValueError('seed IDs/sigma shape')
  if self.belief_seed is not None:
   for key in ['world','sigma_m','query_pixel_id','partner_pixel_id','frame','time_s','frame_width']:
    if not np.array_equal(self.belief_seed[key],candidate[key]):raise ValueError('immutable seed changed')
   return
  if not keep.any():raise ValueError('nonempty qualified hazard seed required')
  self.belief_seed=copy.deepcopy(candidate)
 def update_points(self,points,root,time_s,ground_meshes=()):
  super().update_points(points,root,time_s,ground_meshes)
  self.belief_observation_time.fill(-np.inf)
  p=self.belief_seed;t=float(time_s)
  if p is None or t>=60.:return
  qt=float(p['time_s']);cells=np.rint((np.asarray(p['world'])[:,:2]-self.origin)/self.resolution).astype(int);cells=cells[self.inside(cells)]
  for i,j in cells:
   # Current measured hazards win; old seed is never reinserted as a fresh source.
   if self.codes[i,j] in (3,4):continue
   self.labels[i,j]=1
   if t-qt<=10.:
    self.codes[i,j]=4;self.memory.hazard_time[i,j]=qt;self.last_seen[i,j]=max(self.last_seen[i,j],qt)
   else:
    self.codes[i,j]=5;self.belief_observation_time[i,j]=qt
 def diagnostics(self):
  d=super().diagnostics();d.update(static_belief_cells=int((self.codes==5).sum()),static_belief_is_observation=False,seed_geometry_time_s=None if self.belief_seed is None else float(self.belief_seed['time_s']))
  return d

class SafeStartBootstrap:
 def __init__(self):self.time=-np.inf;self.handed_off=False;self.handoff_time_s=None;self.low_count=0;self.blocked=False
 def choose(self,time_s,measured_XY_speed,packet,nominal_request,event,current_frame):
  t=float(time_s);v=float(measured_XY_speed);a=np.asarray(nominal_request).copy();frame=int(current_frame)
  if not np.isfinite(t) or t<self.time or not np.isfinite(v) or v<0 or a.shape!=(10,) or not np.isfinite(a).all():raise ValueError('finite current measurements')
  if event is not None:
   et=float(event['time_s']);ef=float(event['frame'])
   if not np.isfinite(et) or et>t or not np.isfinite(ef) or ef!=int(ef) or ef>frame:raise ValueError('past event clock before payload')
  self.time=t
  if self.handed_off:return a
  self.low_count=self.low_count+1 if v<.05 else 0
  if t>=8.:self.blocked=True
  if not self.blocked and self.low_count>=5 and event is not None and event['confirmed'] is True:
   qt=float(packet['time_s']);pf=float(packet['frame'])
   if not np.isfinite(qt) or qt>t or not np.isfinite(pf) or pf!=int(pf) or pf>frame:raise ValueError('current packet clock before payload')
   if t-qt<=.5 and pf>=ef:
    xyz=np.asarray(packet['world'],float)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():raise ValueError('finite provisional source')
    if ((xyz[:,2]>.05)&(xyz[:,2]<=1.5)).sum()>=100:
     self.handed_off=True;self.handoff_time_s=t;return a
  return np.zeros(10,np.float32)
