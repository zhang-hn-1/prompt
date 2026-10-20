from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
D=Path(__file__).resolve().parent;W=D.parents[1];L=W/'experiments/learned_visible_obstacles_20261013_v1';C=W/'experiments/source_confidence_calibration_20261012_v1';cm=ListedColormap(['white','#83c5e8','#fdae61','#d7191c','#762a83']);norm=BoundaryNorm(np.arange(-.5,5.5),5)
for name in ['E0','E1']:
 R=C/f'data_{name.lower()}_run';f=np.load(R/'frames/frame_450.npz');src=np.load(R/'source_predictions/frame_450.npz');a=np.load(D/f'{name}_oracle_maps.npz');sgbm=np.load(L/'sgbm_predictions'/f'{name}.npz')['labels'][90]
 fig,axs=plt.subplots(2,3,figsize=(13,8));axs=axs.ravel();axs[0].imshow(np.concatenate([f['left_rgb'],f['right_rgb']],1));axs[0].set_title('Current left/right RGB');axs[0].axis('off')
 for ax,k,title in [(axs[1],'all','All currently visible true surfaces'),(axs[2],'potential','Opposite FOV/CAD/range potential (GT oracle)'),(axs[3],'verified','Both eyes depth-verified (GT oracle)'),(axs[4],'native','Perfect depth + native pose (GT oracle)')]:
  ax.imshow(a[k][90],origin='lower',extent=(-3,3,0,4),cmap=cm,norm=norm,interpolation='nearest');ax.set(title=title,xlabel='Lateral (m)',ylabel='Forward (m)')
 axs[5].imshow(sgbm,origin='lower',extent=(-3,3,0,4),cmap=cm,norm=norm,interpolation='nearest');axs[5].set(title='Existing SGBM current prediction',xlabel='Lateral (m)',ylabel='Forward (m)')
 fig.suptitle(name+' fixed midpoint 45s: geometry qualification, NOT new model predictions\nWhite unknown; blue ground NOT free; orange/red/purple height bands');fig.tight_layout();fig.savefig(D/f'{name}_geometry.png',dpi=150);plt.close(fig)
 z=src['depth_m'];gt=f['left_depth_m'];valid=np.isfinite(z)&(z>=.5)&(z<=3);score=valid&np.isfinite(gt)&(gt>=.5)&(gt<=3)
 fig,axs=plt.subplots(1,4,figsize=(15,4));axs[0].imshow(f['left_rgb']);axs[0].set_title(name+' left RGB 45s')
 for ax,data,title,clim,cmap in [(axs[1],np.where(valid,z,np.nan),'Existing raw SGBM depth',(.5,3),'viridis'),(axs[2],np.where(np.isfinite(gt)&(gt>=.5)&(gt<=3),gt,np.nan),'Rendered true depth',(.5,3),'viridis'),(axs[3],np.where(score,np.abs(z-gt),np.nan),'Raw finite-depth absolute error',(0,.6),'magma')]:
  im=ax.imshow(data,vmin=clim[0],vmax=clim[1],cmap=cmap);ax.set_title(title);fig.colorbar(im,ax=ax,shrink=.6,label='m')
 for ax in axs:ax.axis('off')
 fig.suptitle('Raw full-image SGBM, before CAD/source selection; white unavailable / outside .5–3m. Not map-point scoring.');fig.tight_layout();fig.savefig(D/f'{name}_depth.png',dpi=140);plt.close(fig)
