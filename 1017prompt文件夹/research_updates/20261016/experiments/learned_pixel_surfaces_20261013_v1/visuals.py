from common import *
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from scipy.spatial import cKDTree
out=D/'figures';out.mkdir(exist_ok=True);details=[];cmap=ListedColormap(['#dddddd','#4b9bd3','#dc7d27'])
for group,frame in [('E1',450),('C54',495)]:
 R=GROUPS[group];pname=f'frame_{frame:03d}.npz';f=np.load(R/'frames'/pname);a=np.load(D/'predictions'/group/pname);lab=np.load(Q/'labels'/group/pname);old=np.load(R/'source_predictions'/pname)
 gt=f['left_depth_m'];valid=lab['left_eligible']&(lab['left_category']>0);take=valid&(a['category'][0]>0);old_valid=old['confidence']&np.isfinite(old['depth_m'])&(old['depth_m']>=.5)&(old['depth_m']<=3);oe=valid&old_valid;z=np.where(a['category'][0]>0,a['depth_m'][0],np.nan);oz=np.where(old_valid,old['depth_m'],np.nan)
 fig,ax=plt.subplots(2,4,figsize=(18,8));ax[0,0].imshow(f['left_rgb']);ax[0,0].set_title('Current left RGB');ax[1,0].imshow(f['right_rgb']);ax[1,0].set_title('Current right RGB')
 for axis,image,title in [(ax[0,1],oz,'Original SGBM depth'),(ax[0,2],z,'Learned prior range'),(ax[0,3],np.where(np.isfinite(gt)&(gt>=.5)&(gt<=3),gt,np.nan),'Renderer GT range (offline only)')]:
  im=axis.imshow(image,cmap='viridis',vmin=.5,vmax=3);axis.set_title(title);fig.colorbar(im,ax=axis,fraction=.035,label='m')
 for axis,image,title in [(ax[1,1],lab['left_category'],'Authored-compatible category target'),(ax[1,2],a['category'][0],'Learned category, confidence >=.9')]:
  axis.imshow(image,cmap=cmap,vmin=0,vmax=2);axis.set_title(title)
 im=ax[1,3].imshow(np.where(take,np.abs(a['depth_m'][0]-gt),np.nan),cmap='magma',vmin=0,vmax=1);ax[1,3].set_title('Learned range absolute error');fig.colorbar(im,ax=ax[1,3],fraction=.035,label='m')
 for axis in ax.ravel():axis.axis('off')
 note='prior-blocker source selection, not random/new worst search' if group=='C54' else 'fixed45s display, not selected by learned score'
 fig.suptitle(f'{group} {frame/10:.1f}s — {note}\nCategory gray unknown / blue ground (never free) / orange obstacle; depth white unavailable/outside .5–3m; error white unscored; learned ranges are priors',fontsize=11);fig.tight_layout();fig.savefig(out/f'{group}_depth_{frame:03d}.png',dpi=130,bbox_inches='tight');plt.close(fig)
 target=np.load(Q/f'{group}_maps.npz');learn=np.load(D/f'{group}_maps.npz')['prediction'];j=frame//5
 fig,ax=plt.subplots(1,4,figsize=(16,4))
 for axis,grid,title in zip(ax,[target['target'][j],target['native_oracle'][j],target['sgbm_with_oracle_category'][j],learn[j]],['Visible category target','Ideal category + ideal range','Ideal category + SGBM range','Learned category + range']):
  axis.imshow(grid.T,origin='lower',extent=(0,4,-3,3),cmap=cmap,vmin=0,vmax=2,aspect='equal');axis.set_title(title);axis.set_xlabel('Local forward (m)');axis.set_ylabel('Local lateral (m)')
 fig.suptitle(f'{group} {frame/10:.1f}s local grids; seen development; local root/yaw gauge reset\nReferences offline; unknown is not free; not world registration proof');fig.tight_layout();fig.savefig(out/f'{group}_local_{frame:03d}.png',dpi=130,bbox_inches='tight');plt.close(fig)
 cp= W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_protocol.json' if group=='H' else (C/f'data_{group.lower()}_protocol.json' if group in ['E0','E1'] else O/('baseline_e1_protocol.json' if group=='B54' else 'candidate_e1_protocol.json'))
 cfg=json.loads(cp.read_text());occ=np.load(cfg['occupancy']);ref=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(ref);states=json.loads((R/'states.json').read_text());track=np.array([s['position'][:2] for s in states]);goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
 for tframe in [frame,899]:
  fig,axes=plt.subplots(1,4,figsize=(18,5));axes[0].scatter(ref[:,0],ref[:,1],s=8,c='0.6',marker='s');axes[0].set_title('True body-band reference')
  for axis,path,title in zip(axes[1:],[R/f'maps/frame_{tframe:03d}.npz',Q/f'{group}_memory_{tframe:03d}.npz',D/f'{group}_memory_{tframe:03d}.npz'],['Original actual risk','Ideal category/range replay','Learned prior replay']):
   m=np.load(path);cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);dist=tree.query(xy)[0];clock=m['clock'] if 'clock' in m.files else m['hazard_acquisition_time'];age=float(m['time_s'])-clock[tuple(cells.T)];bad=dist>.15
   axis.scatter(ref[:,0],ref[:,1],s=8,c='0.85',marker='s');axis.scatter(xy[:,0],xy[:,1],c=np.where(bad,'#d73027','#208b48'),s=13);axis.scatter(xy[age>10,0],xy[age>10,1],s=30,facecolors='none',edgecolors='#7028a0',linewidths=.7);axis.set_title(f'{title}\nrisk{len(xy)}, far{bad.sum()}, old{(age>10).sum()}')
  for axis in axes:
   axis.plot(track[:tframe+1,0],track[:tframe+1,1],c='#2166ac',lw=1);axis.plot(*goal,'*',c='black',ms=10);axis.set(xlim=(-5,5.5),ylim=(-5,5.5),aspect='equal',xlabel='World X(m)',ylabel='World Y(m)')
  fig.suptitle(f'{group} {tframe/10:.1f}s same recorded trajectory; no new navigation\nGreen<=.15m from reference, red farther, purple ring age>10s; white prediction unknown');fig.tight_layout();fig.savefig(out/f'{group}_memory_{tframe:03d}.png',dpi=130,bbox_inches='tight');plt.close(fig)
 details.append(dict(group=group,frame=frame,selection=note,full_pixels=640*480,eligible_labelled=int(valid.sum()),learned_scored=int(take.sum()),original_scored_same_label=int(oe.sum()),learned_p50_p90_p95=np.quantile(np.abs(a['depth_m'][0][take]-gt[take]),[.5,.9,.95]).tolist() if take.any() else [None]*3,original_p50_p90_p95=np.quantile(np.abs(old['depth_m'][oe]-gt[oe]),[.5,.9,.95]).tolist() if oe.any() else [None]*3))
save('display_metrics.json',details)
