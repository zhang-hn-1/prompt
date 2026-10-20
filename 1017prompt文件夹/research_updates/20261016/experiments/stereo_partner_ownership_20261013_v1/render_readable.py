"""Presentation fix only: actual E0 coordinate bounds, discrete point markers."""
from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;C=D.parents[1]/'experiments/source_confidence_calibration_20261012_v1';R=C/'data_e0_run'
for name in ['E0_map_comparison.png','E0_depth_comparison.png']:
 p=D/name;p.rename(D/('initial_'+name))
cfg=json.loads((C/'data_e0_protocol.json').read_text());a=np.load(cfg['occupancy']);truth=a['global_origin_xy']+np.argwhere(a['grid']>0)*float(a['resolution_m']);tree=cKDTree(truth);track=np.array([x['position'][:2] for x in json.loads((R/'states.json').read_text())]);goal=np.array(json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);domain=[track,goal[None]];fig,axs=plt.subplots(1,3,figsize=(15,5))
for ax,arm in zip(axs[:2],['before','after']):
 m=np.load(D/f'E0_{arm}_map.npz');cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);old=float(m['time_s'])-m['clock'][tuple(cells.T)]>10;bad=tree.query(xy)[0]>.15;domain.append(xy);ax.scatter(truth[:,0],truth[:,1],s=7,c='0.8',marker='s');ax.scatter(xy[:,0],xy[:,1],s=15,c=np.where(bad,'#d73027','#208b48'));ax.scatter(xy[old,0],xy[old,1],s=38,facecolors='none',edgecolors='#7028a0');ax.set_title(arm+' risk vs reference')
axs[2].scatter(truth[:,0],truth[:,1],s=7,c='0.6',marker='s');axs[2].set_title('True body-band reference');domain=np.vstack(domain);lo=domain.min(0)-2;hi=domain.max(0)+2
for ax in axs:
 ax.plot(track[:,0],track[:,1],c='#2166ac',lw=1);ax.plot(*goal,'*',c='black',ms=12);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
fig.suptitle('E0 final map 89.9s — same-trajectory replay; purple age>10s; red >.15m; blank unknown');fig.tight_layout();fig.savefig(D/'E0_map_comparison.png',dpi=150,bbox_inches='tight');plt.close(fig)
p=R/'source_predictions/frame_450.npz';a=np.load(p);f=np.load(R/'frames'/p.name);v=np.load(D/'predictions'/f'E0_{p.name}');q=v['query'];k=v['keep'];z=a['depth_m'];gt=f['left_depth_m'];fig,axs=plt.subplots(2,3,figsize=(12,8));axs=axs.ravel();axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB, E0 45s')
for ax,ids,title in [(axs[1],q,'Stereo sampled source before'),(axs[2],q[k],'Stereo sampled source after')]:
 im=ax.scatter(ids%640,ids//640,c=z.ravel()[ids],s=1.5,marker='s',linewidths=0,cmap='viridis',vmin=.5,vmax=3);ax.set(xlim=(0,640),ylim=(480,0),aspect='equal',title=title);fig.colorbar(im,ax=ax,shrink=.65,label='m')
im=axs[3].imshow(np.where(np.isfinite(gt)&(gt>0)&(gt<=3),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[3].set_title('Rendered true depth');fig.colorbar(im,ax=axs[3],shrink=.65,label='m');ids=q[k];valid=np.isfinite(gt.ravel()[ids])&(gt.ravel()[ids]>0);ids=ids[valid];im=axs[4].scatter(ids%640,ids//640,c=np.abs(z.ravel()[ids]-gt.ravel()[ids]),s=1.5,marker='s',linewidths=0,vmin=0,vmax=.6,cmap='magma');axs[4].set(xlim=(0,640),ylim=(480,0),aspect='equal',title='After error: white=no score');fig.colorbar(im,ax=axs[4],shrink=.65,label='m')
axs[5].text(.02,.8,'Preselected midpoint, not worst frame.\nOnly actual sampled packet pixels shown.\nPoint markers enlarged for visibility.\nNo interpolation or filled missing depth.\nWhite prediction = unavailable / unknown.\nTruth is used only for evaluation.',va='top',fontsize=11)
for ax in axs:ax.axis('off')
fig.tight_layout();fig.savefig(D/'E0_depth_comparison.png',dpi=150,bbox_inches='tight');plt.close(fig)
