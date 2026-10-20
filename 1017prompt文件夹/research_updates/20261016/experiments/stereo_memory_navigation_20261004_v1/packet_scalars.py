"""Restore saved scalar NPZ metadata to its original runtime scalar type."""
import numpy as np

def normalize_npz_packet(packet):
 out=dict(packet)
 for k in ('frame','time_s','frame_width','pose_sigma_m','source_model_admitted','certified_free'):
  if k in out:
   v=out[k]
   if isinstance(v,np.ndarray):
    if v.shape!=():raise ValueError('metadata scalar required')
    out[k]=v.item()
   elif isinstance(v,np.generic):out[k]=v.item()
 return out
