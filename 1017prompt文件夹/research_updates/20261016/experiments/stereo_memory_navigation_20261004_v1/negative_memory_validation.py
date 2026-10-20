"""Independent post-close truth/pair/evidence checks; never actor imports."""
from pathlib import Path
import numpy as np

def archives_equal(a,b,prefix_samples=None):
 with np.load(a) as x,np.load(b) as y:
  if set(x.files)!=set(y.files):return False
  for key in x.files:
   xx=x[key];yy=y[key]
   if prefix_samples is not None:
    if len(xx)<prefix_samples or len(yy)<prefix_samples:return False
    xx=xx[:prefix_samples];yy=yy[:prefix_samples]
   if not np.array_equal(xx,yy,equal_nan=np.issubdtype(xx.dtype,np.inexact)):return False
 return True

def true_cylinder_clearance(xy):
 xy=np.asarray(xy,float)
 if xy.ndim!=2 or xy.shape[1]!=2 or not len(xy) or not np.isfinite(xy).all():raise ValueError('finite nominal XY')
 if len(xy)==1:return float(np.linalg.norm(xy[0])-.35)
 a=xy[:-1];v=np.diff(xy,axis=0);u=np.clip((-a*v).sum(1)/np.maximum((v*v).sum(1),1e-15),0,1)
 return float(np.linalg.norm(a+u[:,None]*v,axis=1).min()-.35)

def save_arm_errors_once(directory,arm,arrays):
 if arm not in ['R0','R1']:raise ValueError('frozen arm')
 p=Path(directory)/('negative_ray_memory_'+arm+'_errors_posthoc_v1.npz')
 with p.open('xb') as f:np.savez_compressed(f,**arrays)
 return p
