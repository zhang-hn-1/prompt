"""14 causal hypothesis features; voxel guesses NEVER measured endpoints."""
import numpy as np
from scipy.spatial import cKDTree
from partial_cue_source_v2 import inflate_xy

def build_features(voxels,positive,unobservable,optical_depths,camera_centres,seed_world,seed_sigma,root_xy,now,view_times):
 times=np.asarray(view_times,float)
 if times.ndim!=1 or not len(times) or not np.isfinite(times).all() or not np.isfinite(now) or np.any(times>now):raise ValueError('future/invalid view clock before payload')
 w=np.asarray(voxels,float);p=np.asarray(positive);u=np.asarray(unobservable);d=np.asarray(optical_depths,float);c=np.asarray(camera_centres,float);seed=np.asarray(seed_world,float);sigma=np.asarray(seed_sigma,float);root=np.asarray(root_xy,float)
 if w.ndim!=2 or w.shape[1]!=3 or p.dtype!=bool or u.dtype!=bool or p.shape!=(len(w),len(times),2) or u.shape!=p.shape or d.shape!=p.shape or c.shape!=(len(times),2,3):raise ValueError('hypothesis support shape')
 if seed.ndim!=2 or seed.shape[1]!=3 or not len(seed) or sigma.shape!=(len(seed),) or not np.isfinite(seed).all() or not np.isfinite(sigma).all() or np.any(sigma<0):raise ValueError('measured seed shape/finite')
 if root.shape!=(2,) or not all(np.isfinite(a).all() for a in [w,d,c,root]) or np.any(p&u) or np.any(d[p]<=0):raise ValueError('finite causal support')
 posfrac=p.mean(1);ufrac=u.mean(1);epoch=p.any(2).mean(1);neg=(~p&~u).mean((1,2));baseline=np.linalg.norm(c.reshape(-1,3)[:,None]-c.reshape(-1,3)[None,:],axis=-1).max()/.6
 med=np.median(seed[:,:2],axis=0);distance=np.linalg.norm(w[:,:2]-med,axis=1)/.5;nearest=cKDTree(seed[:,:2]).query(w[:,:2])[0]/.5
 depth=np.array([np.median(row[pr]) if pr.any() else 0. for row,pr in zip(d,p)])/3.
 rays=w[:,None,None,:]-c[None];rays=rays.reshape(len(w),-1,3);norm=np.linalg.norm(rays,axis=-1)
 if np.any(norm==0):raise ValueError('zero projection ray')
 rays/=norm[:,:,None];mask=p.reshape(len(w),-1);parallax=np.zeros(len(w))
 for j in range(rays.shape[1]):
  for k in range(j):
   angle=np.arccos(np.clip((rays[:,j]*rays[:,k]).sum(1),-1,1));parallax=np.maximum(parallax,np.where(mask[:,j]&mask[:,k],angle,0.))
 x=np.column_stack((posfrac,ufrac,epoch,neg,np.full(len(w),baseline),distance,nearest,np.full(len(w),np.median(sigma)/.1),w[:,2]/1.5,depth,parallax/np.deg2rad(2),np.linalg.norm(w[:,:2]-root,axis=1)/3.))
 assert x.shape==(len(w),14) and np.isfinite(x).all();return x

def scored_hypotheses(voxels,probabilities,now):
 w=np.asarray(voxels,float);p=np.asarray(probabilities,float)
 if w.ndim!=2 or w.shape[1]!=3 or p.shape!=(len(w),) or not np.isfinite(w).all() or not np.isfinite(p).all() or np.any((p<0)|(p>1)) or not np.isfinite(now):raise ValueError('finite hypothesis scores')
 kept=w[p>=.5]
 return dict(raw_world_voxel_centres=kept,possible_xy=inflate_xy(kept[:,:2],.2),model_hypothesis=True,measured_geometry=False,certified_free=False,source_model_admitted=False,time_s=float(now),static_persistence_prior=True)
