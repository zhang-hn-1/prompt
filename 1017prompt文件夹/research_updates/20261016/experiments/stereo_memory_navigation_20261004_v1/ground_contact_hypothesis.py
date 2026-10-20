"""Conditional floor-ray hypothesis. Negative colour never certifies free ground."""
import numpy as np

def predict(positive,self_mask,optical_world):
 positive=np.asarray(positive,bool);self_mask=np.asarray(self_mask,bool);T=np.asarray(optical_world,float)
 if positive.shape!=(480,640) or self_mask.shape!=(480,640) or T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('calibrated images and finite current pose')
 uv=[];world=[];reason=dict(no_positive=0,bottom_cut_or_self=0,ray_inapplicable=0)
 for x in range(640):
  ys=np.flatnonzero(positive[:,x])
  if not len(ys):reason['no_positive']+=1;continue
  y=int(ys[-1])
  if y>475 or self_mask[y:y+4,x].any() or positive[y+1:y+4,x].any():reason['bottom_cut_or_self']+=1;continue
  ray=np.array([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,1.]);direction=T[:3,:3]@ray
  if abs(direction[2])<1e-12:reason['ray_inapplicable']+=1;continue
  depth=-T[2,3]/direction[2]
  if not .5<=depth<=3.:reason['ray_inapplicable']+=1;continue
  uv.append([x,y]);world.append(T[:3,3]+direction*depth)
 return dict(uv=np.asarray(uv,int).reshape(-1,2),world=np.asarray(world,float).reshape(-1,3),reason=reason,label='GROUNDED_CONTACT_MODEL_HYPOTHESIS',certified_free=False,measured_stereo=False,full_volume=False)
