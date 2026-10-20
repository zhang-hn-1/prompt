from common import *
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
out=D/'figures';out.mkdir(exist_ok=True);details=[]
for name,i in [('E1',450),('C54',495)]:
 R,cp=GROUPS[name];f=np.load(R/'frames'/f'frame_{i:03d}.npz');old=np.load(R/'source_predictions'/f'frame_{i:03d}.npz');pkt=np.load(R/'sensor_packets'/f'source_{i:03d}.npz');gt=f['left_depth_m'];ids=pkt['query_pixel_id'].astype(int);z=gt.ravel()[ids];keep=np.isfinite(z)&(z>=.5)&(z<=3);valid=old['confidence']&np.isfinite(old['depth_m'])&(old['depth_m']>=.5)&(old['depth_m']<=3);oracle=np.full((480,640),np.nan);oracle.ravel()[ids[keep]]=z[keep]
 fig,ax=plt.subplots(2,3,figsize=(15,8));ax[0,0].imshow(f['left_rgb']);ax[1,0].imshow(f['right_rgb']);ax[0,0].set_title('Current left RGB');ax[1,0].set_title('Current right RGB')
 for axis,a,title in [(ax[0,1],np.where(valid,old['depth_m'],np.nan),'Original SGBM finite .5–3m'),(ax[0,2],oracle,'OFFLINE perfect range at original query IDs'),(ax[1,1],np.where(np.isfinite(gt)&(gt>=.5)&(gt<=3),gt,np.nan),'Renderer truth (offline only)')]:
  im=axis.imshow(a,cmap='viridis',vmin=.5,vmax=3);axis.set_title(title);fig.colorbar(im,ax=axis,fraction=.035,label='m')
 err=np.full_like(gt,np.nan);err.ravel()[ids[keep]]=np.abs(old['depth_m'].ravel()[ids[keep]]-z[keep]);im=ax[1,2].imshow(err,cmap='magma',vmin=0,vmax=1);ax[1,2].set_title('Original query absolute depth error');fig.colorbar(im,ax=ax[1,2],fraction=.035,label='m')
 for a in ax.ravel():a.axis('off')
 note='previous blocker source selection, not random/new worst search' if name=='C54' else 'fixed45s, not score-selected'
 fig.suptitle(f'{name} {i/10:.1f}s — {note}\nWhite unknown/unscored; perfect range is GT diagnostic, not learned prediction');fig.tight_layout();fig.savefig(out/f'{name}_depth_{i:03d}.png',dpi=130,bbox_inches='tight');plt.close(fig)
 cfg=json.loads(cp.read_text());occ=np.load(cfg['occupancy']);ref=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(ref);states=json.loads((R/'states.json').read_text());track=np.array([s['position'][:2] for s in states]);goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
 for frame in [i,899]:
  fig,axes=plt.subplots(1,4,figsize=(18,5));axes[0].scatter(ref[:,0],ref[:,1],s=8,c='0.6',marker='s');axes[0].set_title('True body-band reference')
  for axis,path,title in zip(axes[1:],[R/f'maps/frame_{frame:03d}.npz',D/f'{name}_range_height_memory_{frame:03d}.npz',D/f'{name}_range_category_memory_{frame:03d}.npz'],['Original actual risk','GT range + height proxy','GT range + GT category']):
   a=np.load(path);cells=np.argwhere(a['labels']==1);xy=a['origin']+cells*float(a['resolution']);bad=tree.query(xy)[0]>.15;clock=a['clock'] if 'clock' in a.files else a['hazard_acquisition_time'];age=float(a['time_s'])-clock[tuple(cells.T)];axis.scatter(ref[:,0],ref[:,1],s=8,c='0.85',marker='s');axis.scatter(xy[:,0],xy[:,1],c=np.where(bad,'#d73027','#208b48'),s=13);axis.scatter(xy[age>10,0],xy[age>10,1],s=30,facecolors='none',edgecolors='#7028a0',linewidths=.7);axis.set_title(f'{title}\nrisk{len(xy)}, far{bad.sum()}, old{(age>10).sum()}')
  for a in axes:a.plot(track[:frame+1,0],track[:frame+1,1],c='#2166ac',lw=1);a.plot(*goal,'*',c='black',ms=10);a.set(xlim=(-5,5.5),ylim=(-5,5.5),aspect='equal',xlabel='World X(m)',ylabel='World Y(m)')
  fig.suptitle(f'{name} {frame/10:.1f}s same recorded trajectory, not new navigation\nGreen<=.15m from reference; red farther; purple age>10s; blank prediction unknown; right two arms OFFLINE GT');fig.tight_layout();fig.savefig(out/f'{name}_memory_{frame:03d}.png',dpi=130,bbox_inches='tight');plt.close(fig)
 details.append(dict(group=name,frame=i,selection=note,queries=len(ids),finite_queries=int(keep.sum()),full_pixels=480*640,original_query_error_p50_p90_p95=np.quantile(err[np.isfinite(err)],[.5,.9,.95]).tolist(),oracle_range_error_m=0.))
save('display_metrics.json',details)
