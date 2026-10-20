from common import *
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R,cp=GROUPS['C65'];cfg=json.loads(cp.read_text());i=450;p=f'frame_{i:03d}.npz';f=np.load(R/'frames'/p);rgb=np.load(R/'source_inputs'/p);gt=f['left_depth_m'];preds=[np.load(R/'source_predictions'/p),np.load(D/'predictions/C65'/p)]
fig,axs=plt.subplots(2,3,figsize=(15,8))
axs[0,0].imshow(rgb['left_rgb']);axs[0,0].set_title('Current left RGB');axs[1,0].imshow(rgb['right_rgb']);axs[1,0].set_title('Current right RGB')
for j,(a,title) in enumerate(zip(preds,['Original SGBM','FoundationStereo'])):
 z=a['depth_m'];valid=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3);score=valid&np.isfinite(gt)&(gt>0);im=axs[j,1].imshow(np.where(valid,z,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[j,1].set_title(title+' accepted depth (m)');fig.colorbar(im,ax=axs[j,1],shrink=.65)
 if j==0:im=axs[j,2].imshow(np.where(np.isfinite(gt)&(gt>=.5)&(gt<=3),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[j,2].set_title('Rendered GT depth (m)')
 else:im=axs[j,2].imshow(np.where(score,np.abs(z-gt),np.nan),vmin=0,vmax=.15,cmap='magma');axs[j,2].set_title('FoundationStereo absolute error (m)')
 fig.colorbar(im,ax=axs[j,2],shrink=.65)
for ax in axs.flat:ax.axis('off')
fig.suptitle('C65 E1 45.0s — fixed mid-sequence illustration, not random/generalization; white=unknown/outside display/no score')
fig.tight_layout();fig.savefig(D/'c65_45s_depth.png',dpi=130);plt.close(fig)
occ=np.load(cfg['occupancy']);true=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(true);fig,axs=plt.subplots(1,3,figsize=(15,5));axs[0].scatter(true[:,0],true[:,1],s=7,c='gray',marker='s');axs[0].set_title('GT body-band reference (score only)')
for j,title in enumerate(['baseline','candidate']):
 m=np.load(D/'predictions/C65'/f'map_{title}_{i:03d}.npz');c=np.argwhere(m['labels']==1);xy=m['origin']+c*float(m['resolution']);dist=tree.query(xy)[0];old=float(m['time_s'])-m['clock'][tuple(c.T)]>10;ax=axs[j+1];ax.scatter(true[:,0],true[:,1],s=6,c='lightgray');ax.scatter(xy[:,0],xy[:,1],s=15,c=np.where(dist>.15,'red','green'));ax.scatter(xy[old,0],xy[old,1],s=35,facecolors='none',edgecolors='purple');ax.set_title(('Original SGBM' if j==0 else 'FoundationStereo')+' risk + long memory')
for ax in axs:ax.set(aspect='equal',xlim=(-4,7),ylim=(-4,4),xlabel='World X (m)',ylabel='World Y (m)')
fig.suptitle('Same C65 E1 45.0s — green within .15m GT; red farther; purple age>10s; blank prediction=unknown')
fig.tight_layout();fig.savefig(D/'c65_45s_bev.png',dpi=130);plt.close(fig)
print('images_saved')
