"""Frozen metric/RGB consistency of MODEL cores; not source-reliability admission."""
import numpy as np
from scipy.spatial import cKDTree

class MetricCoreSupport:
 def __init__(self):self.last_time=-np.inf;self.sources={}
 def add(self,packet,class_masks,now):
  t=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or abs(t-now)>1e-9 or t<=self.last_time:raise ValueError('current clock before payload')
  f=float(packet['frame'])
  if not np.isfinite(f) or f!=round(f) or abs(f*.1-t)>1e-5 or int(packet['frame_width'])!=640:raise ValueError('source identity')
  w=np.asarray(packet['world'],float);q=np.asarray(packet['query_pixel_id']);s=np.asarray(packet['partner_pixel_id'])
  if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all() or q.shape!=(len(w),) or s.shape!=q.shape:raise ValueError('finite metric points and UID shape')
  for ids in [q,s]:
   if not np.issubdtype(ids.dtype,np.integer) or (ids<0).any() or (ids>=480*640).any():raise ValueError('native pixel UID')
  for cl in class_masks.values():
   if cl.shape!=(480,640) or cl.dtype!=bool:raise ValueError('native class mask')
  use=class_masks['left'].ravel()[q]&class_masks['right'].ravel()[s]&(w[:,2]>.05)&(w[:,2]<=1.5)
  self.sources[int(f)]=dict(time_s=t,world=w[use].copy(),query_pixel_id=q[use].copy(),partner_pixel_id=s[use].copy());self.last_time=t
 def screen(self,cores,support_frames,now):
  if not np.isfinite(now):raise ValueError('current finite time before payload')
  fs=np.asarray(support_frames)
  if fs.ndim!=1 or not np.isfinite(fs).all() or not np.equal(fs,np.rint(fs)).all() or (fs*.1>now+1e-5).any():raise ValueError('past support identity')
  cores=np.asarray(cores,float)
  if cores.ndim!=2 or cores.shape[1]!=3 or not np.isfinite(cores).all():raise ValueError('finite cores')
  points=[];ids=[]
  for f in fs.astype(int):
   if f not in self.sources:continue
   r=self.sources[f]
   if r['time_s']>now+1e-9:raise ValueError('future buffered metric source')
   for p,q,s in zip(r['world'],r['query_pixel_id'],r['partner_pixel_id']):points.append(p);ids.append(dict(source_frame=int(f),source_time_s=r['time_s'],query_pixel_id=int(q),partner_pixel_id=int(s)))
  keep=np.zeros(len(cores),bool);proof=[None]*len(cores)
  if points and len(cores):
   d,j=cKDTree(points).query(cores);keep=d<=.2+1e-9
   for k in np.flatnonzero(keep):proof[k]=dict(**ids[j[k]],distance_m=float(d[k]),consistency_tolerance_m=.2,not_measured_geometry=True)
  return keep,proof
