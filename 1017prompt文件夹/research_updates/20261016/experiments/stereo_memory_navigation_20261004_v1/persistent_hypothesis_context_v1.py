"""Readonly hypothesis provenance/current projection. No surface/visibility certificate."""
import math,re,numpy as np
class PersistentHypothesisContext:
 def __init__(self,birth,hypothesis_sha256,now):
  t=float(birth['time_s'])
  if not np.isfinite([t,now]).all() or t>now:raise ValueError('causal birth before payload')
  if not isinstance(hypothesis_sha256,str) or re.fullmatch('[0-9a-f]{64}',hypothesis_sha256) is None:raise ValueError('immutable source SHA')
  if not birth['model_hypothesis'] or birth['measured_geometry'] or birth['certified_free']:raise ValueError('hypothesis-only birth')
  w=np.asarray(birth['raw_world_voxel_centres'],dtype=float)
  if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():raise ValueError('finite hypothetical voxels')
  self._points=w.copy();self._birth=t;self._sha=hypothesis_sha256;self._now=float(now)
 def inspect(self,current,now):
  t=float(current['time_s']);n=float(now)
  if not np.isfinite([t,n]).all() or t!=n or n<self._now:raise ValueError('same current exposure, before payload')
  active=n-self._birth<=60.+1e-9
  base=dict(model_active=active,state='retained_static_hypothesis' if active else 'expired_hypothesis',birth_time_s=self._birth,hypothesis_sha256=self._sha,retention_s=60.,hypothesis_identity_is_object_identity=False,measured_geometry=False,observed_free=False,source_admitted=False,projection_not_physical_visibility=True,current_depth_absence_does_not_clear=True,eye_reason={},eye_pixel={})
  if not active:self._now=n;return base
  for eye in ['left','right']:
   im=np.asarray(current['images'][eye]);mask=np.asarray(current['masks'][eye]);T=np.asarray(current['poses'][eye],dtype=float)
   if im.shape!=(480,640,3) or im.dtype!=np.uint8 or mask.shape!=(480,640) or mask.dtype!=bool:raise ValueError('native RGB/boolean CAD')
   if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-8) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6):raise ValueError('rigid current USD world camera')
   local=(self._points-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]))
   with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
   valid=(local[:,2]>0)&np.isfinite(uv).all(1)&(uv[:,0]>=-.5)&(uv[:,0]<639.5)&(uv[:,1]>=-.5)&(uv[:,1]<479.5)
   ids=np.flatnonzero(valid);pix=np.full((len(local),2),-1,dtype=int);pix[ids]=np.rint(uv[ids]).astype(int);x,y=pix[ids].T;reason=np.full(len(local),'outside_projectable_frustum',dtype='<U40');black=np.all(im[y,x]==0,axis=1);reason[ids]=np.where(mask[y,x],'CAD_unknown',np.where(black,'black_RGB_unknown','nonblack_projection_only'));base['eye_reason'][eye]=reason;base['eye_pixel'][eye]=pix
  self._now=n;return base
