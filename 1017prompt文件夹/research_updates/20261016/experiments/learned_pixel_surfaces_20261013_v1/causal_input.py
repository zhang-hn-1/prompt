"""Only current dual RGB and calibrated native orientation: no GT/scene labels."""
import numpy as np
from projection import F,FLIP
Y,X=np.mgrid[0:480:2,0:640:2]
RAYS=np.stack([(X-319.5)/F,(Y-239.5)/F,np.ones_like(X)],-1)
def features(left,right,left_T,right_T):
 if left.shape!=(480,640,3) or right.shape!=left.shape:raise ValueError('native paired RGB')
 rgb=np.concatenate([left[::2,::2],right[::2,::2]],-1).astype(np.float32)/255
 gravity=np.stack([(RAYS@(np.asarray(T)[:3,:3]@FLIP).T)[...,2] for T in [left_T,right_T]],-1)
 xy=RAYS[...,:2]
 return np.concatenate([rgb,gravity,xy],-1).transpose(2,0,1).astype(np.float32)
