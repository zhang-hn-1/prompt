from common import *
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
def quant(a):return np.quantile(a,[.5,.9,.95]).tolist() if len(a) else [None]*3

summary=json.loads((D/'result.json').read_text());metrics={r['group']:r for r in summary['groups']};out=D/'figures';out.mkdir(exist_ok=True)
for name,i in [('E0',450),('C54',495)]:
 R,cp=GROUPS[name];gm=D/'predictions'/name;f=np.load(R/'frames'/f'frame_{i:03d}.npz');old=np.load(R/'source_predictions'/f'frame_{i:03d}.npz');new=np.load(gm/f'frame_{i:03d}.npz');gt=f['left_depth_m'];finite=np.isfinite(gt)&(gt>0);truth=np.where(finite&(gt>=.5)&(gt<=3),gt,np.nan)
 images=[]
 for a in [old,new]:
  z=a['depth_m'];k=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3);images.append(np.where(k,z,np.nan))
 fig,axs=plt.subplots(2,3,figsize=(15,8));axs[0,0].imshow(f['left_rgb']);axs[0,0].set_title('Current left RGB');axs[1,0].imshow(f['right_rgb']);axs[1,0].set_title('Current right RGB')
 for ax,z,title in [(axs[0,1],images[0],'Original SGBM depth'),(axs[0,2],images[1],'New Census depth'),(axs[1,1],truth,'Rendered truth depth (scoring only)')]:
  im=ax.imshow(z,cmap='viridis',vmin=.5,vmax=3);ax.set_title(title);fig.colorbar(im,ax=ax,fraction=.035,label='m')
 im=axs[1,2].imshow(np.where(finite,np.abs(images[1]-gt),np.nan),cmap='magma',vmin=0,vmax=1);axs[1,2].set_title('Census absolute depth error');fig.colorbar(im,ax=axs[1,2],fraction=.035,label='m')
 for ax in axs.ravel():ax.axis('off')
 note='preselected from prior failure; not random or a new worst-frame search' if name=='C54' else 'fixed 45s display; not selected by candidate score'
 fig.suptitle(f'{name} at {i*.1:.1f}s — {note}\nDepth white = unknown/outside .5–3m; error white = no finite joint score',fontsize=12);fig.tight_layout();fig.savefig(out/f'{name}_depth_{i:03d}.png',dpi=150,bbox_inches='tight');plt.close(fig)
 cfg=json.loads(cp.read_text());occ=np.load(cfg['occupancy']);reference=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(reference);states=json.loads((R/'states.json').read_text());track=np.array([r['position'][:2] for r in states]);goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
 for frame in [i,899]:
  fig,axs=plt.subplots(1,3,figsize=(15,5))
  for ax,arm in zip(axs[:2],['baseline','candidate']):
   m=np.load(gm/f'map_{arm}_{frame:03d}.npz');cell=np.argwhere(m['labels']==1);xy=m['origin']+cell*float(m['resolution']);age=float(m['time_s'])-m['clock'][tuple(cell.T)];bad=tree.query(xy)[0]>.15
   ax.scatter(reference[:,0],reference[:,1],s=9,c='0.82',marker='s');ax.scatter(xy[:,0],xy[:,1],s=16,c=np.where(bad,'#d73027','#208b48'));ax.scatter(xy[age>10,0],xy[age>10,1],s=34,facecolors='none',edgecolors='#7028a0',linewidths=.9)
   ax.set_title(('Original SGBM' if arm=='baseline' else 'Census replay')+f'\nrisk {len(xy)}; >.15m {bad.sum()}; age>10s {(age>10).sum()}')
  axs[2].scatter(reference[:,0],reference[:,1],s=9,c='0.6',marker='s');axs[2].set_title('True body-band occupancy reference')
  for ax in axs:
   ax.plot(track[:frame+1,0],track[:frame+1,1],c='#2166ac',lw=1);ax.plot(*goal,'*',c='black',ms=12);ax.set(xlim=(-5,5.5),ylim=(-5,5.5),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
   if name=='C54':ax.plot(-1.1,-.1,'x',c='black',ms=9)
  fig.suptitle(f'{name} at {frame*.1:.1f}s — same recorded trajectory, not new navigation\nGreen: risk within .15m of reference; red: farther; purple ring: age>10s; blank prediction: unknown',fontsize=11);fig.tight_layout();fig.savefig(out/f'{name}_maps_{frame:03d}.png',dpi=150,bbox_inches='tight');plt.close(fig)
# Prior blocker pixel cohort, chosen before this candidate (not a GT-based online mask).
R,_=GROUPS['C54'];prior=json.loads((O/'blocker_provenance.json').read_text());cohort=[]
for item in prior['source_epochs']:
 i=int(item['frame'].split('_')[1]);q=np.asarray(item['pixel_ids']);old=np.load(R/'source_predictions'/f'frame_{i:03d}.npz');new=np.load(D/'predictions/C54'/f'frame_{i:03d}.npz');f=np.load(R/'frames'/f'frame_{i:03d}.npz');gt=f['left_depth_m'].ravel()[q];z=new['depth_m'].ravel()[q];valid=new['confidence'].ravel()[q]&np.isfinite(z);err=np.abs(z-gt)
 cohort.append(dict(frame=i,queries=len(q),census_measured=int(valid.sum()),census_unknown=int((~valid).sum()),error_le015=int((valid&(err<=.15)).sum()),error_gt030=int((valid&(err>.3)).sum()),old_error_quantiles=quant(np.abs(old['depth_m'].ravel()[q]-gt)),new_error_quantiles=quant(err[valid]),source_pixels=q.tolist()))
cell_states=[]
for arm in ['baseline','candidate']:
 m=np.load(D/'predictions/C54'/f'map_{arm}_899.npz');cell=np.rint((np.array([-1.1,-.1])-m['origin'])/float(m['resolution'])).astype(int);key=tuple(cell);risk=bool(m['labels'][key]==1);cell_states.append(dict(arm=arm,risk=risk,age_s=float(m['time_s']-m['clock'][key]) if risk else None))
save('prior_blocker_cohort.json',dict(rows=cohort,final_original_blocker_cell=cell_states,scope='offline previously selected error provenance; no GT online deletion, unknown not free; full map impacts scored separately'))
