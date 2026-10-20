"""Source-only helper; rejected ray/graph outputs stay frozen."""
import numpy as np,cv2
def unmasked_linear_support(mask,mx,my):
 mask=np.asarray(mask,bool);mx=np.asarray(mx,np.float32);my=np.asarray(my,np.float32)
 inside=np.isfinite(mx)&np.isfinite(my)&(mx>=0)&(my>=0)&(mx<=mask.shape[1]-1)&(my<=mask.shape[0]-1)
 return inside&(cv2.remap((~mask).astype(np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)==1.)
