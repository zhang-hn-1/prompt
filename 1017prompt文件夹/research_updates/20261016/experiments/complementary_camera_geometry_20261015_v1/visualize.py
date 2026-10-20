from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;a=np.load(D/'example.npz');fig,ax=plt.subplots(2,3,figsize=(15,8));
for j,e in enumerate(['left','right']):
 ax[j,0].imshow(np.rot90(a[e+'_rgb']));ax[j,0].set_title('B65 47s original '+e+' RGB (prior selected frame)');ax[j,0].axis('off')
 z=a[e+'_gt'];ax[j,1].imshow(np.rot90(np.where(np.isfinite(z),z,np.nan)),vmin=.5,vmax=3,cmap='viridis');ax[j,1].set_title('Original renderer GT .5-3m display; white=no hit');ax[j,1].axis('off')
o=a['origin'];t=a['target_cells']*.1+o;c=a['retained_cells']*.1+o;ax[0,2].scatter(t[:,0],t[:,1],c='red',s=10,label='Original visible obstacle target');ax[0,2].scatter(c[:,0],c[:,1],c='green',s=10,label='Candidate frustum upper bound');ax[0,2].scatter(*a['root'][:2],c='black');ax[0,2].axis('equal');ax[0,2].legend(fontsize=7);ax[0,2].set_title('GT-only BEV; no new scene/CAD occlusion\nNo predicted or memory map generated')
r=np.load(D/'ring_2.0.npz');th=r['angle'];ax[1,2].plot(th*180/np.pi,r['baseline'].astype(int),label='Same headings');ax[1,2].plot(th*180/np.pi,r['candidate'].astype(int),label='Yaw +/-20 degrees');ax[1,2].set_xlabel('Base bearing degrees, forward=0');ax[1,2].set_ylabel('Inside calibrated frustum at root radius 2m');ax[1,2].legend();ax[1,2].set_title('3600 equal angular samples; geometry only\n0 = unobserved, not free')
fig.tight_layout();fig.savefig(D/'geometry.png',dpi=160);plt.close(fig)
