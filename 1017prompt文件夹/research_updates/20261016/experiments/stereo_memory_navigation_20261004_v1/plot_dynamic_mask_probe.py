from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
fig,axes=plt.subplots(4,2,figsize=(12,12))
for r,i in enumerate((0,30,60,120)):
 data=np.load(H/'dynamic_mask_probe_run_v2'/'online_inputs'/f'frame_{i:03d}.npz')
 for c,eye in enumerate(('left','right')):
  z=np.load(H/'dynamic_mask_probe_assessment'/f'{i:03d}_{eye}.npz');a=axes[r,c];a.imshow(data[eye+'_rgb']);overlay=np.zeros((480,640,4));overlay[z['renderer_self_mask']]=[0,1,0,.25];overlay[z['extra_mask']]=[1,.2,0,.75];a.imshow(overlay);a.set_title(f'{i/10:g}s {eye}: green=renderer self, orange=added CAD mask');a.axis('off')
fig.tight_layout();fig.savefig(H/'dynamic_mask_probe_actual_views.png',dpi=140)
