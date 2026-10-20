"""Frozen postseal single-eye prediction metrics; never called by inference."""
import numpy as np
from scipy.ndimage import maximum_filter

def evaluate_geometry(pred,truth,camera,mask,prompt,root):
 assert pred.shape==truth.shape==mask.shape==prompt.shape==(480,640)
 yy,xx=np.mgrid[:480,:640];ray=np.stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(xx.shape)],axis=-1);rot=camera[:3,:3]@np.diag([1,-1,-1]);pz=pred[yy,xx];gz=truth[yy,xx]
 with np.errstate(invalid='ignore',over='ignore'):
  pw=camera[:3,3]+(ray*pz[...,None])@rot.T;gw=camera[:3,3]+(ray*gz[...,None])@rot.T;error=np.linalg.norm(pw-gw,axis=-1)
 def eligible(z,w):
  return (~mask[yy,xx])&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(w).all(-1)&(w[...,2]>.05)&(w[...,2]<=1.5)&(np.linalg.norm(w[...,:2]-root[:2],axis=-1)>.55)
 pe=eligible(pz,pw);ge=eligible(gz,gw);finite=np.isfinite(error);correct=finite&(error<=.1);halo=maximum_filter(prompt>0,size=7,mode='constant')[yy,xx];new=pe&correct&~halo
 bins=len(np.unique(np.floor(pw[new,:2]/.1).astype(np.int64),axis=0)) if new.any() else 0
 n=int(pe.sum());ng=int(ge.sum());prec=float((pe&correct).sum()/n) if n else None;cover=float((ge&correct).sum()/ng) if ng else None
 errs=np.where(finite,error,np.inf)[ge];p90=float(np.quantile(errs,.9,method='nearest')) if ng else None
 return dict(predicted_is_observation=False,predicted_eligible=n,correct=int((pe&correct).sum()),insufficient_truth=int((pe&~finite).sum()),finite_wrong=int((pe&finite&~correct).sum()),precision=prec,visible_domain=ng,coverage=cover,visible_p90_m=p90,inside_halo_predicted=int((pe&halo).sum()),inside_halo_correct=int((pe&halo&correct).sum()),outside_halo_predicted=int((pe&~halo).sum()),outside_halo_correct=int(new.sum()),outside_halo_XY_bins=bins,gate=bool(n>=100 and prec>=.95 and ng>=100 and cover>=.90 and p90<=.10 and new.sum()>=20 and bins>=2))
