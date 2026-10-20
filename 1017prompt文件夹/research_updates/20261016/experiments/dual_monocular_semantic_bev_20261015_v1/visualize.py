from common import *
from projection import *
from category_memory import CategoryMemory
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
R,cp=GROUPS['B65'];cfg=json.loads(cp.read_text());masks=dict(np.load(cfg['static_masks']));p='frame_470.npz';f=np.load(R/'frames'/p);src=np.load(R/'source_inputs'/p);a=np.load(D/'predictions/B65'/p);pose=np.load(R/'known_poses'/p);origin=a['origin'];target=[]
fig,axs=plt.subplots(2,5,figsize=(19,8))
for j,e in enumerate(['left','right']):
 gt=f[e+'_depth_m'];z=a[e+'_depth_m'];valid=np.isfinite(gt)&(gt>=.5)&(gt<=3)&~masks[e].astype(bool);axs[j,0].imshow(src[e+'_rgb']);axs[j,0].set_title(e+' RGB')
 im=axs[j,1].imshow(z,vmin=.5,vmax=5,cmap='viridis');axs[j,1].set_title('Monocular prior depth (m)');fig.colorbar(im,ax=axs[j,1],shrink=.6)
 im=axs[j,2].imshow(np.where(np.isfinite(gt)&(gt>0),gt,np.nan),vmin=.5,vmax=5,cmap='viridis');axs[j,2].set_title('Rendered GT depth (m)');fig.colorbar(im,ax=axs[j,2],shrink=.6)
 im=axs[j,3].imshow(np.where(valid,np.abs(z-gt),np.nan),vmin=0,vmax=2,cmap='magma');axs[j,3].set_title('Absolute error (m)');fig.colorbar(im,ax=axs[j,3],shrink=.6)
 axs[j,4].imshow(a[e+'_category'],vmin=0,vmax=2,cmap=ListedColormap(['#999999','#d73027','#2166ac']));axs[j,4].set_title('Red obstacle / blue ground / gray unknown')
 w,q=world_points(gt,f[e+'_camera_usd_world'],masks[e],pose['root_position_world_m'],stride=1);k=(w[:,2]>.05)&(w[:,2]<=1.5);target.append(cells(w[k],origin))
for ax in axs.flat:ax.axis('off')
fig.suptitle('B65 E1 47.0s — frame selected by prior v75 oracle failure, not current model worst-frame search\nPredictions sealed before GT; white=no valid score/GT. Depth>5m clipped for display, NOT excluded from accuracy.')
fig.tight_layout();fig.savefig(D/'b65_47s_depth_semantics.png',dpi=120);plt.close(fig)
target=np.unique(np.concatenate(target),axis=0);truexy=origin+target*.1;fig,axs=plt.subplots(1,5,figsize=(20,5));axs[0].scatter(*truexy.T,c='gray',s=8);axs[0].set_title('Current union visible GT')
for j,k in enumerate(['left_cells','right_cells','fused_cells']):
 c=a[k];xy=origin+c*.1;dist=cKDTree(target*.1).query(c*.1)[0] if len(c) else np.empty(0);axs[j+1].scatter(*truexy.T,c='lightgray',s=8);axs[j+1].scatter(*xy.T,c=np.where(dist>.15,'red','green'),s=10);axs[j+1].set_title(['Left prior','Right prior','Fused prior'][j])
init=np.load(R/'maps/frame_000.npz');memory=CategoryMemory(origin,init['labels'].shape,.1,True)
for pp in sorted((D/'predictions/B65').glob('frame_*.npz')):
 b=np.load(pp);i=int(b['frame']);t=float(b['time_s'])
 if i>470:break
 c=b['fused_cells'];w=np.column_stack([origin+c*.1,np.full(len(c),.5)]);ps=np.load(R/'known_poses'/pp.name);memory.update(w,np.full(len(c),2),i,t,ps['left_camera_usd_world'][:3,3])
 if i==450:
  check=np.load(D/'predictions/B65/memory_450.npz');assert np.array_equal(memory.snapshot(t)['labels'],check['labels']);assert np.array_equal(memory.snapshot(t)['clock'],check['clock'])
m=memory.snapshot(float(a['time_s']));occ=np.load(cfg['occupancy']);ref=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);ids=np.argwhere(m['labels']==1);xy=origin+ids*.1;dist=cKDTree(ref).query(xy)[0];old=float(a['time_s'])-m['clock'][tuple(ids.T)]>10;axs[4].scatter(*ref.T,c='lightgray',s=6);axs[4].scatter(*xy.T,c=np.where(dist>.15,'red','green'),s=8);axs[4].scatter(*xy[old].T,s=30,facecolors='none',edgecolors='purple');axs[4].set_title('Risk + age>10s purple rings')
for ax in axs:ax.set(aspect='equal',xlim=(-4,5),ylim=(-4,4),xlabel='World X (m)',ylabel='World Y (m)')
fig.suptitle('Same B65 E1 47.0s — current maps scored vs visible union; memory vs full body-band reference\nGreen within .15m reference; red farther; blanks unknown. Current & memory denominators differ.')
fig.tight_layout();fig.savefig(D/'b65_47s_bev_memory.png',dpi=120);plt.close(fig)
save('b65_47s_plot_counts.json',dict(current_cells=[len(a[k]) for k in ['left_cells','right_cells','fused_cells']],visible_union=len(target),memory_risk=len(ids),memory_wrong=int((dist>.15).sum()),old=int(old.sum()),old_wrong=int((old&(dist>.15)).sum()),figure_choice='prior v75 failure frame, not current model worst frame'))
print('figures_saved')
