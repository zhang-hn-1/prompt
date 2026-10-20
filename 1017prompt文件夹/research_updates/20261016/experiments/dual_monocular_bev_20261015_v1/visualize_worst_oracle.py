from common import *
from projection import *
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R,cp=GROUPS['B65'];cfg=json.loads(cp.read_text());masks=dict(np.load(cfg['static_masks']));p='frame_470.npz';a=np.load(R/'source_inputs'/p);pose=dict(np.load(R/'known_poses'/p));f=np.load(R/'frames'/p);origin=np.load(R/'maps/frame_000.npz')['origin'];zs=[f[e+'_depth_m'] for e in ['left','right']];pred,conflict=project_pair(zs,pose,masks,origin);ts=[]
for j,e in enumerate(['left','right']):
 w,_=per_eye(zs[j],f[e+'_camera_usd_world'],masks[e],pose['root_position_world_m'],stride=1);ts.append(cells(w,origin))
target=np.unique(np.concatenate(ts),axis=0);fig,axs=plt.subplots(2,4,figsize=(17,8))
for j,e in enumerate(['left','right']):
 axs[0,j].imshow(a[e+'_rgb']);axs[0,j].set_title(e+' RGB');im=axs[0,j+2].imshow(np.where(np.isfinite(zs[j])&(zs[j]>=.5)&(zs[j]<=3),zs[j],np.nan),vmin=.5,vmax=3,cmap='viridis');axs[0,j+2].set_title(e+' rendered GT depth (m)');fig.colorbar(im,ax=axs[0,j+2],shrink=.6)
axs[1,0].scatter(*(origin+target*.1).T,c='gray',s=8);axs[1,0].set_title('Full union visible GT target')
for j,c in enumerate(pred):
 d=cKDTree(target*.1).query(c*.1)[0] if len(c) and len(target) else np.full(len(c),np.inf);xy=origin+c*.1;axs[1,j+1].scatter(*xy.T,c=np.where(d>.15,'red','green'),s=10);axs[1,j+1].set_title(['Left oracle projection','Right oracle projection','Fused oracle projection'][j])
for ax in axs[0]:ax.axis('off')
for ax in axs[1]:ax.set(aspect='equal',xlim=(-4,4),ylim=(-4,4),xlabel='world X (m)',ylabel='world Y (m)')
fig.suptitle('B65 E1 47.0s post-selected worst precision frame, >=20 predicted cells; GT-only geometry probe, NO monocular model inference\nGreen within .15m visible target; red farther; blank unknown. Rendered depth never used online.')
fig.tight_layout();fig.savefig(D/'b65_worst_oracle_geometry.png',dpi=130);plt.close(fig)
# Explain false height classification with same-pixel true world surface.
rows=[]
for j,e in enumerate(['left','right']):
 w,q=per_eye(zs[j],pose[e+'_camera_usd_world'],masks[e],pose['root_position_world_m']);y=q//640;x=q%640;ray=np.column_stack([(x-319.5)/F,(y-239.5)/F,np.ones(len(q))]);T=f[e+'_camera_usd_world'];tw=T[:3,3]+(ray*zs[j][y,x,None])@(T[:3,:3]@AX).T
 rows.append(dict(eye=e,inferred_obstacle_points=len(q),true_ground=int((np.abs(tw[:,2])<=.03).sum()),pose_point_p90_m=float(np.quantile(np.linalg.norm(w-tw,axis=1),.9)) if len(q) else None))
save('b65_worst_explanation.json',dict(rows=rows,conflicts=conflict,scope='GT-only same-pixel classification diagnostic, post-selected B65 minimum precision among frames with >=20 predicted cells; no model inference.'))
print(rows)
