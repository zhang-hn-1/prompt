from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
SOURCE=D.parent/'replanning_reserve_20261013_v1'
for ep in ['candidate_e1']:
 if not (SOURCE/f'{ep}_status.json').exists():continue
 R=SOURCE/f'{ep}_run';status=json.loads((SOURCE/f'{ep}_status.json').read_text());
 if status['status']!='finished':continue
 c=json.loads((SOURCE/f'{ep}_protocol.json').read_text());occ=np.load(c['occupancy']);grid=occ['grid'];origin=occ['global_origin_xy'];res=float(occ['resolution_m']);truecells=np.argwhere(grid>0);truthxy=origin+truecells*res;tree=cKDTree(truthxy)
 states=json.loads((R/'states.json').read_text());track=np.array([s['position'][:2] for s in states]);goal=np.array(json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);predpaths=sorted((R/'source_predictions').glob('*.npz'));picked=R/'source_predictions/frame_220.npz';mp=R/'maps'/picked.name;m=np.load(mp);snapshot_time=float(m['time_s']);track=track[:int(round(snapshot_time*10))+1];cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);dist=tree.query(xy)[0] if len(xy) else np.empty(0);older=(float(m['time_s'])-m['hazard_acquisition_time'][tuple(cells.T)])>10
 fig,axs=plt.subplots(1,3,figsize=(15,5));axs[0].scatter(xy[:,0],xy[:,1],s=15,c='#208b48');axs[0].scatter(xy[older,0],xy[older,1],s=40,facecolors='none',edgecolors='#7028a0');axs[0].set_title('Predicted risk; purple=age >10s');axs[1].scatter(truthxy[:,0],truthxy[:,1],s=8,c='0.6',marker='s');axs[1].set_title('True body-band occupancy reference');axs[2].scatter(truthxy[:,0],truthxy[:,1],s=8,c='0.8',marker='s');axs[2].scatter(xy[:,0],xy[:,1],s=15,c=np.where(dist>.15,'#d73027','#208b48'));axs[2].set_title('Overlay; red >0.15m from reference')
 domain=np.vstack([track,goal[None],xy,truthxy]);lo=domain.min(0)-.5;hi=domain.max(0)+.5
 for ax in axs:
  ax.plot(track[:,0],track[:,1],c='#2166ac',lw=1);ax.plot(*goal,'*',c='black',ms=12);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
 fig.suptitle(f'{ep.upper()} at {snapshot_time:.1f}s — blank prediction is unknown');fig.tight_layout();fig.savefig(D/f'{ep}_map_comparison.png',dpi=140,bbox_inches='tight');plt.close(fig)
 predpaths=sorted((R/'source_predictions').glob('*.npz'));p=R/'source_predictions/frame_220.npz';f=np.load(R/'frames'/p.name);a=np.load(p);z=a['depth_m'];gt=f['left_depth_m'];valid=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0)
 fig,axs=plt.subplots(1,4,figsize=(16,4));axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB');im=axs[1].imshow(np.where(a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3),z,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[1].set_title('Original stereo depth (m)');fig.colorbar(im,ax=axs[1],shrink=.6);im=axs[2].imshow(np.where(np.isfinite(gt)&(gt>0)&(gt<=3),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[2].set_title('True depth (m)');fig.colorbar(im,ax=axs[2],shrink=.6);im=axs[3].imshow(np.where(valid,np.abs(z-gt),np.nan),vmin=0,vmax=.3,cmap='magma');axs[3].set_title('Error (m), white=no score');fig.colorbar(im,ax=axs[3],shrink=.6)
 for ax in axs:ax.axis('off')
 fig.suptitle(f'{ep.upper()} at {float(a["time_s"]):.1f}s — post-selected blocking-cell source frame, not random/worst-frame search; white = unavailable/outside display/no score');fig.tight_layout();fig.savefig(D/f'{ep}_depth_comparison.png',dpi=140,bbox_inches='tight');plt.close(fig)
 totals=0;validn=0;floor=0;errors=[]
 for p in sorted((R/'partial_observations').glob('*.npz')):
  a=np.load(p);f=np.load(R/'frames'/p.name);q=a['query_pixel_id'].astype(int);x=q%640;y=q//640;z=f['left_depth_m'][y,x];ok=np.isfinite(z)&(z>0);rays=np.stack([(x[ok]-319.5)/584.8888888888889,(y[ok]-239.5)/584.8888888888889,np.ones(ok.sum())],1);T=f['left_camera_usd_world'];gtworld=T[:3,3]+(rays*z[ok,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;totals+=len(q);validn+=int(ok.sum());floor+=int((np.abs(gtworld[:,2])<=.03).sum());errors.extend(np.linalg.norm(a['world'][ok]-gtworld,axis=1))
 r=dict(scene=ep,terminal=json.loads((R/'termination.json').read_text()),accepted_point_instances=totals,truth_valid_instances=validn,truth_valid_fraction=validn/max(1,totals),floor_false_instances=floor,floor_false_fraction=floor/max(1,validn),position_error_p50_m=float(np.quantile(errors,.5)) if errors else None,position_error_p90_m=float(np.quantile(errors,.9)) if errors else None,snapshot_time_s=snapshot_time,snapshot_risk_cells=len(xy),snapshot_older10s_cells=int(older.sum()),snapshot_farther015m_reference_cells=int((dist>.15).sum()),snapshot_old_farther015m_cells=int((older&(dist>.15)).sum()),limits='Map reference is existing vertical_max_body_band occupancy (not exact 3D surface). 0.15m cell-centre tolerance only evaluation; no online change. No full-image recall. Accepted-point metrics exclude invalid truth.')
 (D/f'{ep}_map_metrics.json').write_text(json.dumps(r,indent=2)+'\n');print(ep,r,flush=True)
