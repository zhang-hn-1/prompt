"""Independent camera exposure clock guard before deferred NPZ payload reads."""
from pathlib import Path
import numpy as np
from mvs_causal_packet_bridge import build_request

class DeferredCapture(dict):
 def __init__(self,time,eye,rgb,pose,mask):super().__init__(time_s=time);self.eye=eye;self.rgb=rgb;self.pose=pose;self.mask=mask
 def __getitem__(self,key):
  if key=='time_s':return super().__getitem__(key)
  if key=='rgb':return self.rgb[self.eye+'_rgb']
  if key=='pose':return self.pose[self.eye+'_camera_usd_world']
  if key=='mask':return self.mask[self.eye]
  raise KeyError(key)

def captured_request(root,frame,eye,clocks,loader):
 if eye not in ['left','right']:raise ValueError('unknown eye')
 root=Path(root);packets=[]
 for j in [frame,frame-10,frame-20]:
  expected=float(clocks[j])
  im=loader(root/'online_inputs'/f'frame_{j:03d}.npz');pose=loader(root/'known_poses'/f'frame_{j:03d}.npz');mask=loader(root/'masks'/f'frame_{j:03d}.npz')
  times=[float(f['time_s']) for f in [im,pose,mask]]
  if not np.isfinite(expected) or any(not np.isfinite(t) or abs(t-expected)>1e-9 for t in times) or int(mask['frame'])!=j:raise ValueError('capture independent clock/frame mismatch before payload')
  packets.append(DeferredCapture(expected,eye,im,pose,mask))
 return build_request(packets[0],packets[1:],float(clocks[frame]))
