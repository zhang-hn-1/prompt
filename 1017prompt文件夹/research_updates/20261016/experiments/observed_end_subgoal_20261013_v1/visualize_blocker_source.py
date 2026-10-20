from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
for ep in ['candidate_e1']:
 if not (D/f'{ep}_status.json').exists():continue
 R=D/f'{ep}_run';status=json.loads((D/f'{ep}_status.json').read_text());
 if status['status']!='finished':continue
 c=json.loads((D/f'{ep}_protocol.json').read_text());occ=np.load(c['occupancy']);grid=occ['grid'];origin=occ['global_origin_xy'];res=float(occ['resolution_m']);truecells=np.argwhere(grid>0);truthxy=origin+truecells*res;tree=cKDTree(truthxy)
 states=json.loads((R/'states.json').read_text());track=np.array([s['position'][:2] for s in states]);goal=np.array(json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);predpaths=sorted((R/'source_predictions').glob('*.npz'));picked=R/'source_predictions/frame_495.npz';mp=R/'maps'/picked.name;m=np.load(mp);snapshot_time=float(m['time_s']);track=track[:int(round(snapshot_time*10))+1];cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);dist=tree.query(xy)[0] if len(xy) else np.empty(0);older=(float(m['time_s'])-m['hazard_acquisition_time'][tuple(cells.T)])>10
 fig,axs=plt.subplots(1,3,figsize=(15,5));axs[0].scatter(xy[:,0],xy[:,1],s=15,c='#208b48');axs[0].scatter(xy[older,0],xy[older,1],s=40,facecolors='none',edgecolors='#7028a0');axs[0].set_title('Predicted risk; purple=age >10s');axs[1].scatter(truthxy[:,0],truthxy[:,1],s=8,c='0.6',marker='s');axs[1].set_title('True body-band occupancy reference');axs[2].scatter(truthxy[:,0],truthxy[:,1],s=8,c='0.8',marker='s');axs[2].scatter(xy[:,0],xy[:,1],s=15,c=np.where(dist>.15,'#d73027','#208b48'));axs[2].set_title('Overlay; red >0.15m from reference')
 domain=np.vstack([track,goal[None],xy,truthxy]);lo=domain.min(0)-.5;hi=domain.max(0)+.5
 for ax in axs:
  ax.plot(track[:,0],track[:,1],c='#2166ac',lw=1);ax.plot(*goal,'*',c='black',ms=12);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
 fig.suptitle(f'{ep.upper()} at {snapshot_time:.1f}s — post-hoc blocking-cell source frame; blank = unknown');fig.tight_layout();fig.savefig(D/f'{ep}_blocker_source_map.png',dpi=140,bbox_inches='tight');plt.close(fig)
 predpaths=sorted((R/'source_predictions').glob('*.npz'));p=R/'source_predictions/frame_495.npz';f=np.load(R/'frames'/p.name);a=np.load(p);z=a['depth_m'];gt=f['left_depth_m'];valid=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0)
 fig,axs=plt.subplots(1,4,figsize=(16,4));axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB');im=axs[1].imshow(np.where(a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3),z,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[1].set_title('Original stereo depth (m)');fig.colorbar(im,ax=axs[1],shrink=.6);im=axs[2].imshow(np.where(np.isfinite(gt)&(gt>0)&(gt<=3),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[2].set_title('True depth (m)');fig.colorbar(im,ax=axs[2],shrink=.6);im=axs[3].imshow(np.where(valid,np.abs(z-gt),np.nan),vmin=0,vmax=.3,cmap='magma');axs[3].set_title('Error (m), white=no score');fig.colorbar(im,ax=axs[3],shrink=.6)
 for ax in axs:ax.axis('off')
 fig.suptitle(f'{ep.upper()} at {float(a["time_s"]):.1f}s — post-hoc blocking-cell source frame; white = unavailable/outside display/no score');fig.tight_layout();fig.savefig(D/f'{ep}_blocker_source_depth.png',dpi=140,bbox_inches='tight');plt.close(fig)
