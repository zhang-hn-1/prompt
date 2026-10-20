"""One copied-RGB CAD-invalid zero-fill ablation; no geometry or truth input."""
import numpy as np

def masked_inputs(left_rgb,right_rgb,left_mask,right_mask):
 l=np.asarray(left_rgb);r=np.asarray(right_rgb);lm=np.asarray(left_mask);rm=np.asarray(right_mask)
 if l.dtype!=np.uint8 or r.dtype!=np.uint8 or l.ndim!=3 or l.shape[-1]!=3 or l.shape!=r.shape:raise ValueError('same uint8 HxWx3 RGB required')
 if lm.dtype!=bool or rm.dtype!=bool or lm.shape!=l.shape[:2] or rm.shape!=l.shape[:2]:raise ValueError('current binary masks matching RGB required')
 a=l.copy();b=r.copy();a[lm]=0;b[rm]=0;return a,b
