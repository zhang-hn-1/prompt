"""One frozen local current-RGB correspondence check; no truth or memory."""
import numpy as np
import cv2

def refine(left,right,left_mask,right_mask,right_positive,query_xy,initial_disparity):
 shape=left.shape[:2]
 if left.shape!=right.shape or left.ndim!=3 or left.shape[2]!=3 or any(np.asarray(m).shape!=shape for m in [left_mask,right_mask,right_positive]):raise ValueError('paired current RGB and masks')
 q=np.asarray(query_xy,float);d=np.asarray(initial_disparity,float)
 if q.shape!=(len(d),2) or not np.isfinite(q).all() or not np.isfinite(d).all():raise ValueError('finite current pixel/disparity')
 n=len(d); offsets=np.arange(-4,4.001,.5);dx,dy=np.meshgrid(np.arange(-3,4),np.arange(-3,4));patch=np.column_stack([dx.ravel(),dy.ravel()]);coords=q[:,None,:]+patch[None,:,:];h,w=shape
 def sample(a,c):
  return cv2.remap(np.asarray(a,np.float32),c[:,:,0].astype(np.float32),c[:,:,1].astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
 def inside(c):return ((c[:,:,0]>=0)&(c[:,:,0]<=w-1)&(c[:,:,1]>=0)&(c[:,:,1]<=h-1)).all(1)
 if n==0:return dict(keep=np.zeros(0,bool),disparity_px=np.zeros(0),score=np.zeros(0),uniqueness_gap=np.zeros(0))
 l=sample(cv2.cvtColor(left,cv2.COLOR_RGB2GRAY),coords);lm=l.mean(1,keepdims=True);lc=l-lm;ls=l.std(1);ln=np.linalg.norm(lc,axis=1)
 lv=inside(coords)&(sample(~np.asarray(left_mask,bool),coords)>.999).all(1)&(ls>=2)
 scores=np.full((n,len(offsets)),-np.inf)
 for j,offset in enumerate(offsets):
  disparity=d+offset;c=coords.copy();c[:,:,1]-=disparity[:,None]
  rr=sample(cv2.cvtColor(right,cv2.COLOR_RGB2GRAY),c);rc=rr-rr.mean(1,keepdims=True);rn=np.linalg.norm(rc,axis=1)
  rv=inside(c)&(sample(~np.asarray(right_mask,bool),c)>.999).all(1)&(rr.std(1)>=2)&(disparity>1)
  centre=c[:,24:25,:];positive=sample(np.asarray(right_positive,bool),centre)[:,0]>.999
  valid=lv&rv&positive
  correlation=np.sum(lc*rc,axis=1)/np.maximum(ln*rn,1e-12)
  scores[valid,j]=correlation[valid]
 best=np.argmax(scores,axis=1);bestscore=scores[np.arange(n),best];bestd=d+offsets[best]
 competitors=np.abs(offsets[None,:]-offsets[best,None])>=1.5
 competitor=np.max(np.where(competitors,scores,-np.inf),axis=1)
 with np.errstate(invalid='ignore'):gap=bestscore-competitor
 keep=np.isfinite(bestscore)&np.isfinite(competitor)&(bestscore>=.9)&(gap>=.05)
 return dict(keep=keep,disparity_px=bestd,score=bestscore,uniqueness_gap=gap)
