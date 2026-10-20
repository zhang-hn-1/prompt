"""Independent one-eye-positive possible geometry; no measured occupancy admission."""
import copy,cv2,numpy as np
from partial_cue_source_v2 import PartialCueSetSource,inflate_xy
from occluded_eye_support import combine_support
class OccludedEyePartialSource(PartialCueSetSource):
 def __init__(self,radius):
  super().__init__(radius);self.candidate_birth=None;self.original_packet=None
 def update(self,*args,**kwargs):
  original=super().update(*args,**kwargs);self.original_packet=copy.deepcopy(original)
  if self.model is None:return original
  root_xy=kwargs.get('root_xy',args[7] if len(args)>7 else None)
  w=self.vox;p=np.zeros((len(w),len(self.views),2),bool);u=np.ones_like(p)
  for j,v in enumerate(self.views):
   for e,eye in enumerate(['left','right']):
    T=v['poses'][eye];local=(w-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]))
    with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
    valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=3)&(uv[:,0]<636)&(uv[:,1]>=3)&(uv[:,1]<476)&(np.linalg.norm(w[:,:2]-root_xy,axis=1)>.55)
    idx=np.flatnonzero(valid);x,y=np.rint(uv[idx]).astype(int).T;blocked=cv2.dilate(v['masks'][eye].astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool);idx=idx[~blocked[y,x]];x,y=np.rint(uv[idx]).astype(int).T
    u[idx,j,e]=False;p[idx,j,e]=self.class_masks[(v['frame'],eye)][y,x]
  keep=combine_support(p,u)&(np.linalg.norm(w[:,:2]-self.start,axis=1)>.7)
  result=dict(original,raw_world_voxel_centres=w[keep],possible_xy=inflate_xy(w[keep,:2],self.radius),single_eye_occluded_possible_only=True,measured_geometry=False,certified_free=False,source_model_admitted=False)
  if self.candidate_birth is None:self.candidate_birth=copy.deepcopy(result)
  self.birth_packet=copy.deepcopy(self.candidate_birth)
  return result
