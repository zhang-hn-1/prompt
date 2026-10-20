"""Read-only provisional RGB endpoint-support map; never a safety certificate.

No floor interpolation, ray carving, body-footprint prior, or truth input.
Scalar sigma is perturbation sensitivity, not a measured error bound. The
floor/hazard grid is a 2.5D diagnostic of an already-rejected source model.
"""
import copy
import numpy as np

def finite_scalar(value):
 value=float(value)
 if not np.isfinite(value):raise ValueError('finite value required')
 return value

class PointSupportMap:
 def __init__(self,origin,shape,resolution,max_age_s=10.):
  self.origin=np.asarray(origin,float);self.shape=tuple(shape);self.resolution=finite_scalar(resolution);self.age=finite_scalar(max_age_s)
  if self.origin.shape!=(2,) or not np.isfinite(self.origin).all() or len(self.shape)!=2 or min(self.shape)<=0 or self.resolution<=0 or self.age<0:raise ValueError('invalid map geometry')
  self.records={};self.time=-np.inf
 def update(self,points,time_s,pose_sigma_m):
  t=finite_scalar(time_s);pose_sigma=finite_scalar(pose_sigma_m)
  if t<self.time or pose_sigma<0:raise ValueError('noncausal time or invalid pose uncertainty')
  staged={}
  for p in points:
   q=p['query'];qt=finite_scalar(q[3])
   if qt>t:continue # Do not read a future record's payload or partner.
   source=p['source'];st=finite_scalar(source[3])
   if st>qt:raise ValueError('source later than its query')
   if t-qt>self.age:continue # Replaying old data cannot refresh observation age.
   xyz=np.asarray(p['world'],float);sigma=finite_scalar(p['sigma_m'])
   if xyz.shape!=(3,) or not np.isfinite(xyz).all() or sigma<0:raise ValueError('invalid measured point')
   key=tuple(q);r=dict(world=xyz.tolist(),sigma_m=sigma,pose_sigma_m=pose_sigma,query=list(q),source=list(source))
   previous=staged.get(key,self.records.get(key))
   if previous is not None and previous!=r:raise ValueError('same query identity has changed')
   staged[key]=r
  # All inputs checked before any state mutation.
  self.records={k:r for k,r in self.records.items() if t-r['query'][3]<=self.age}
  self.records.update(staged);self.time=t
 def snapshot(self,time_s):
  t=finite_scalar(time_s)
  if t<self.time:raise ValueError('past query would read future evidence')
  floor=np.full(self.shape,-np.inf);hazard=np.full(self.shape,-np.inf);records=[];ignored=0
  for r in self.records.values():
   qt=r['query'][3]
   if t-qt>self.age:continue
   r=copy.deepcopy(r);records.append(r);xyz=np.asarray(r['world']);cell=np.rint((xyz[:2]-self.origin)/self.resolution).astype(int)
   if min(cell)<0 or cell[0]>=self.shape[0] or cell[1]>=self.shape[1]:ignored+=1;continue
   sigma=r['sigma_m']+r['pose_sigma_m'];z=xyz[2];ij=tuple(cell)
   if abs(z)+sigma<=.05 and sigma<=self.resolution*.5:floor[ij]=max(floor[ij],qt)
   elif .05<z<=1.5:hazard[ij]=max(hazard[ij],qt)
   else:ignored+=1
  f=np.isfinite(floor);h=np.isfinite(hazard);labels=np.full(self.shape,-1,np.int8);labels[f]=0;labels[h]=1
  codes=np.zeros(self.shape,np.uint8);codes[f]=np.where(np.abs(floor[f]-t)<1e-6,1,2);codes[h]=np.where(np.abs(hazard[h]-t)<1e-6,3,4)
  return dict(time_s=t,labels=labels,provenance=codes,floor_time=floor,hazard_time=hazard,records=records,ignored_records=ignored,conflict_cells=int((f&h).sum()),certified_free=False,source_model_admitted=False,full_3d_certified=False,model_imputed_regions=0)

def swept_indices(path,origin,resolution,radius_m):
 path=np.asarray(path,float).reshape(-1,2);origin=np.asarray(origin,float);resolution=finite_scalar(resolution);radius=finite_scalar(radius_m)+np.sqrt(2)*resolution/2
 if not len(path) or not np.isfinite(path).all() or resolution<=0 or radius_m<0:raise ValueError('finite whole trajectory required')
 lo=np.floor((path.min(0)-radius-origin)/resolution).astype(int);hi=np.ceil((path.max(0)+radius-origin)/resolution).astype(int)
 ii,jj=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1),indexing='ij');cells=np.c_[ii.ravel(),jj.ravel()];xy=origin+cells*resolution
 distance=np.full(len(cells),np.inf)
 for a,b in zip(path[:-1],path[1:]):
  v=b-a;vv=np.dot(v,v);u=np.zeros(len(xy)) if vv==0 else np.clip((xy-a)@v/vv,0,1)
  distance=np.minimum(distance,np.linalg.norm(xy-(a+u[:,None]*v),axis=1))
 if len(path)==1:distance=np.linalg.norm(xy-path[0],axis=1)
 return cells[distance<=radius+1e-12]

def support_result(labels,cells):
 labels=np.asarray(labels);cells=np.asarray(cells,int).reshape(-1,2);inside=(cells>=0).all(1)&(cells[:,0]<labels.shape[0])&(cells[:,1]<labels.shape[1]);values=np.full(len(cells),-1,np.int8)
 values[inside]=labels[cells[inside,0],cells[inside,1]];unknown=cells[values<0];occupied=cells[values>0]
 return dict(checked_cells=len(cells),unknown_cells=len(unknown),occupied_cells=len(occupied),floor_cells=int((values==0).sum()),support_passed=not len(unknown) and not len(occupied),unknown_indices=unknown.tolist(),occupied_indices=occupied.tolist(),full_3d_certified=False,action_authorized=False)

def replay_prefix(packets,time_s,origin,shape,resolution):
 t=finite_scalar(time_s);m=PointSupportMap(origin,shape,resolution)
 for p in packets:
  pt=finite_scalar(p['time_s'])
  if pt>t:continue # Future packet payload is not read.
  m.update(p['points'],pt,p['pose_sigma_m'])
 return m
