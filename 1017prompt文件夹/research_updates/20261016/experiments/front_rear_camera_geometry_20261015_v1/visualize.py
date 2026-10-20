from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;a=np.load(D/'render_recovery_run/frames/frame_000.npz');fig,ax=plt.subplots(2,3,figsize=(15,8))
for j in [0,1]:
 ax[j,0].imshow(np.rot90(a[f'candidate_{j}_rgb']));ax[j,0].axis('off');ax[j,0].set_title(('Front' if j==0 else 'Rear')+' requested view RGB, exposure pose UNQUALIFIED')
 z=a[f'candidate_{j}_gt'];ax[j,1].imshow(np.rot90(np.where(np.isfinite(z),z,np.nan)),cmap='viridis',vmin=.5,vmax=3);ax[j,1].axis('off');ax[j,1].set_title('Saved renderer depth; pose/ray check FAILED')
 ax[j,2].imshow(np.rot90(a[f'baseline_{j}_rgb']));ax[j,2].axis('off');ax[j,2].set_title('Same-pose original '+('left' if j==0 else 'right')+' RGB')
fig.suptitle('First captured pose only; both complete replays failed at pose #1; view identity NOT PASSED\nNo monocular prediction, error/long-term map or full new-view recall generated',fontsize=13);fig.tight_layout();fig.savefig(D/'first_pose_render.png',dpi=150);plt.close(fig)
