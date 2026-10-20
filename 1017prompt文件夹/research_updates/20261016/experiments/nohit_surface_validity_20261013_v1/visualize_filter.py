from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;C=D.parent/'source_confidence_calibration_20261012_v1';R=C/'data_e1_run';ev=np.load(D/'E1_evaluation.npz');best=None
for i in np.unique(ev['frame']):
 ix=np.flatnonzero(ev['frame']==i);q=ev['query'][ix];keep=ev['keep'][ix];f=np.load(R/'frames'/f'frame_{i:03d}.npz');x=q%640;y=q//640;gt=f['left_depth_m'][y,x];finite=np.isfinite(gt)&(gt>0);T=f['left_camera_usd_world'];ray=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);world=T[:3,3]+(ray*np.where(finite,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;obs=finite&(world[:,2]>.05)&(world[:,2]<=1.5)
 if obs.sum()>=100:
  rate=(obs&keep).sum()/obs.sum()
  if best is None or rate<best[0]:best=(rate,int(i))
i=best[1];ix=np.flatnonzero(ev['frame']==i);q=ev['query'][ix];k=ev['keep'][ix];f=np.load(R/'frames'/f'frame_{i:03d}.npz');a=np.load(R/'source_predictions'/f'frame_{i:03d}.npz');z=a['depth_m'];gt=f['left_depth_m'];before=np.full(z.shape,np.nan);before.ravel()[q]=z.ravel()[q];after=before.copy();after.ravel()[q[~k]]=np.nan;finite=np.isfinite(gt)&(gt>0);removed=np.ones((*z.shape,3));wrong=q[~k&finite.ravel()[q]];nohit=q[~k&np.isposinf(gt).ravel()[q]];removed.reshape(-1,3)[wrong]=[.85,.1,.1];removed.reshape(-1,3)[nohit]=[.1,.65,.85]
fig,axs=plt.subplots(2,3,figsize=(13,8));axs[0,0].imshow(f['left_rgb']);axs[0,0].set_title('Current left RGB');axs[0,1].imshow(before,vmin=.5,vmax=3,cmap='viridis');axs[0,1].set_title('Before: source candidate depth');axs[0,2].imshow(after,vmin=.5,vmax=3,cmap='viridis');axs[0,2].set_title('After: rejected model filtering');axs[1,0].imshow(np.where(finite,gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[1,0].set_title('True depth (color capped at3m)');axs[1,1].imshow(removed);axs[1,1].set_title('Removed red=finite hit; cyan=no hit');im=axs[1,2].imshow(np.where(np.isfinite(after)&finite,np.abs(after-gt),np.nan),vmin=0,vmax=.3,cmap='magma');axs[1,2].set_title('After error (capped at0.3m)');fig.colorbar(im,ax=axs[1,2],shrink=.65)
for ax in axs.flat:ax.axis('off')
fig.suptitle(f'E1 worst real-obstacle retention epoch: {float(a["time_s"]):.1f}s, retained {best[0]*100:.1f}% — reporting selection, not average');fig.tight_layout();fig.savefig(D/'e1_filter_truth_comparison.png',dpi=140);plt.close(fig);(D/'visual_selection.json').write_text(json.dumps(dict(frame=i,obstacle_retention=float(best[0]),selection='minimum per-epoch obstacle retention among >=100 source point GTpositiveheight samples; posthoc visualization only'),indent=2)+'\n')
