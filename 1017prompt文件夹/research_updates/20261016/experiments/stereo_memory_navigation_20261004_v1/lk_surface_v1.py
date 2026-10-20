"""Frozen current-to-past LK; 2D correspondences, no measured depth or free space."""
import cv2
import numpy as np
def track_pairs(query_rgb,source_rgb,query_xy,source_mask):
 a=np.asarray(query_rgb);b=np.asarray(source_rgb);xy=np.asarray(query_xy,np.float32);mask=np.asarray(source_mask,bool)
 if a.ndim!=3 or a.shape[2]!=3 or b.shape!=a.shape or mask.shape!=a.shape[:2] or xy.ndim!=2 or xy.shape[1]!=2 or not np.isfinite(xy).all():raise ValueError('native RGB, finite XY and same-sized CAD mask')
 if not len(xy):return []
 gray0=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY);gray1=cv2.cvtColor(b,cv2.COLOR_RGB2GRAY);p=xy.reshape(-1,1,2)
 settings=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01),minEigThreshold=.0001)
 forward,status,error=cv2.calcOpticalFlowPyrLK(gray0,gray1,p,None,**settings)
 if forward is None:return []
 back,backstatus,_=cv2.calcOpticalFlowPyrLK(gray1,gray0,forward,None,**settings)
 if back is None:return []
 rows=[];height,width=mask.shape
 for i,(f,backp) in enumerate(zip(forward[:,0],back[:,0])):
  fb=float(np.linalg.norm(backp-xy[i]));err=float(error[i,0])
  if not status[i,0] or not backstatus[i,0] or not np.isfinite(np.r_[f,fb,err]).all() or fb>1. or err>20.:continue
  x,y=np.rint(f).astype(int)
  if x-10<0 or y-10<0 or x+10>=width or y+10>=height or mask[y-10:y+11,x-10:x+11].any():continue
  rows.append(dict(query_index=i,source_xy=f.tolist(),forward_backward_px=fb,photometric_mean_absolute=err))
 return rows
