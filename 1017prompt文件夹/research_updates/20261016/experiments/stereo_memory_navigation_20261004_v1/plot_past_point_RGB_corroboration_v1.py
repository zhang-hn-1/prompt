from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'anchor_validation_fresh_run_v1';d=np.load(H/'past_point_RGB_corroboration_predictions_v1/frame_190.npz');rgb=np.load(O/'online_inputs/frame_190.npz');sel=d['appearance_corroborated'];f,ax=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
for a,eye in zip(ax,['left','right']):
 a.imshow(rgb[eye+'_rgb']);a.scatter(d[eye+'_uv'][sel,0],d[eye+'_uv'][sel,1],s=16,facecolors='none',edgecolors='#f1f144',linewidths=.8);a.set_title(f'{eye} RGB at19s: 30 past points corroborated');a.axis('off')
f.suptitle('Past geometry at18s + current RGB appearance at19s; privileged known pose\nAppearance support only; no new depth, geometry-age refresh or navigation admission',fontsize=11)
f.savefig(H/'past_point_RGB_corroboration_frame190_v1.png',dpi=150)
