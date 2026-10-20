"""Frozen RGB/native-pose source pipeline, loaded only in spawn child."""
from pathlib import Path
import time,numpy as np
class Backend:
 def __init__(self):
  import torch
  from fast_stereo import FastStereo
  from static_base_cached_self_mask_v2 import StaticBaseCachedSelfMask
  torch.set_num_threads(1);self.model=FastStereo();self.cad=StaticBaseCachedSelfMask()
  p=Path(__file__).resolve().parents[1]/'visible_surface_connectivity_20261004_v1/base_link_masks.npz'
  with np.load(p) as z:self.masks={k:z[k].copy() for k in z.files}
 def __call__(self,job):
  from masked_source_pair import paired_sources
  from paired_texture_surface import PairedTextureSurfaceMemory
  from stereo_geometry import restore_native
  import torch
  t=float(job['time_s']);i=int(job['frame']);begin=time.monotonic()
  pred=self.model.predict(job['left_rgb'],job['right_rgb'],self.masks['left'],self.masks['right']);torch.cuda.synchronize();nn=time.monotonic()-begin
  begin=time.monotonic();_,raw,current_masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**job['pose'],time_s=t),job['history'],self.cad,t);cad=time.monotonic()-begin
  begin=time.monotonic();memory=PairedTextureSurfaceMemory();points=memory.update(raw,job['left_rgb'],job['right_rgb'],t,job['input_sha256'],restore_native(pred['disparity_px']));select=time.monotonic()-begin
  return dict(frame=i,time_s=t,prediction=pred,raw=raw,current_masks=current_masks,surface=points,input_sha256=job['input_sha256'],timing=dict(NN_wall_s=nn,CAD_pair_wall_s=cad,texture_wall_s=select))
