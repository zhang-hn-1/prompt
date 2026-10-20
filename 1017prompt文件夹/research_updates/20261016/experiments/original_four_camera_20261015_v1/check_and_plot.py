from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from four_camera import NAMES,validate_complete,nominal_transforms
D=Path(__file__).resolve().parent;R=D/'native_run';cfg=json.loads((D/'protocol.json').read_text());r=json.loads((D/'geometry_result.json').read_text());s=validate_complete(json.loads((D/'native_run_status.json').read_text()),R);occ=np.load(cfg['occupancy']);o=occ['global_origin_xy'];step=float(occ['resolution_m']);ref=np.array([[o[0]+i*step,o[1]+j*step] for i in range(occ['grid'].shape[0]) for j in range(occ['grid'].shape[1]) if occ['grid'][i,j]>0]);tree=cKDTree(ref);counts={};maxview=0.;maxround=0.;allsets=[]
for row,sealrow in zip(r['frames'],s['rows']):
 a=np.load(R/'frames'/row['frame']);b=np.load(D/(Path(row['frame']).stem+'_audit.npz'));sets=[]
 np.testing.assert_allclose(ref,b['reference_xy'],atol=0,rtol=0)
 for name,ar in zip(NAMES,row['arms']):
  T=np.linalg.inv(a[name+'_view_raw'].reshape(4,4)).T;maxview=max(maxview,float(np.max(abs(T-a[name+'_camera_usd_world_label']))));z=a[name+'_depth_label'];mask=b[name+'_CAD'];yy,xx=np.where(np.isfinite(z)&(z>=.5)&(z<=3)&~mask);d=z[yy,xx];p=np.stack([(xx-319.5)*d/cfg['focal_px'],-(yy-239.5)*d/cfg['focal_px'],-d,np.ones(len(d))],axis=1);w=(T@p.T).T[:,:3];k=(np.linalg.norm(w-a['root_position_physics_label'],axis=1)>=.55)&(w[:,2]>.05)&(w[:,2]<=1.5);cc=np.unique(np.floor((w[k,:2]-o)/.1+.5).astype(int),axis=0);assert np.array_equal(cc,b[name+'_cells']);sets.append(set(map(tuple,cc)))
  dist=tree.query(cc*.1+o)[0];assert int((dist>.15).sum())==ar['far_from_reference_cells']
  assert float(a[name+'_render_time'])==sealrow['clock'][2]
 un=set.union(*sets);assert len(un)==row['all_union'];assert len((sets[2]|sets[3])-(sets[0]|sets[1]))==row['rear_new'];allsets.append(un)
for p,h in r['source_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
for p,h in cfg['source_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
out=dict(pass_check=True,frames=8,actual_RGB_frames=32,all_four_same_exposure_time=True,all_visible_bodyband_pixel_homogeneous_projection_exact=True,all_reference_cell_XY_order_checked=True,all_eight_grid_sets_and_distances_checked=True,renderer_inverse_view_max_difference=maxview,original_project_source_SHA_unchanged=True,raw_source_SHA_checked=True,initial_scoring_error='Initial scorer swapped occupancy [x,y] to image [y,x]; preserved initial code/result; corrected to repository grid convention. Actual runs, masks, points and source thresholds unchanged.')
(D/'independent_check.json').write_text(json.dumps(out,indent=2)+'\n');print(out,flush=True)
a=np.load(R/'frames/frame_000.npz');b=np.load(D/'frame_000_audit.npz');row=r['frames'][0]
fig,axes=plt.subplots(2,4,figsize=(16,8),constrained_layout=True)
for j,(name,ar) in enumerate(zip(NAMES,row['arms'])):
 axes[0,j].imshow(np.rot90(a[name+'_rgb']));axes[0,j].contour(np.rot90(b[name+'_CAD']),levels=[.5],colors=['red'],linewidths=.65);axes[0,j].set_title(name.replace('_',' ')+'\nCAD silhouette '+f"{ar['CAD_pixels']/307200:.1%}")
 z=a[name+'_depth_label'].copy();z[~np.isfinite(z)]=np.nan;im=axes[1,j].imshow(np.rot90(z),vmin=.5,vmax=3,cmap='viridis');fig.colorbar(im,ax=axes[1,j],shrink=.7);axes[1,j].set_title(f"GT depth (m); finite {ar['finite_pixels']}/307200")
 for k in [0,1]:axes[k,j].axis('off')
fig.suptitle('Original fixed four cameras / E1 / first exposure 0.5 s (preselected)\nRed outline=known CAD silhouette; white=unavailable depth; color clipped .5-3 m. Model prediction/error=N/A.',fontsize=12)
fig.savefig(D/'first_four_rgb_depth.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(15,10));fig.subplots_adjust(top=.86,bottom=.08,hspace=.36,wspace=.26);root=b['root'];ref=b['reference_xy'];cols=['tab:blue','tab:cyan','tab:orange','tab:purple']
for k,ax in enumerate(axes.ravel()[:5]):
 ax.scatter(ref[:,0],ref[:,1],color='0.86',s=3);ax.scatter(*root[:2],marker='*',s=80,c='black');ax.set_xlim(root[0]-3,root[0]+4);ax.set_ylim(root[1]-3,root[1]+3);ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')
 if k<4:
  cc=b[NAMES[k]+'_cells'];xy=cc*.1+o;ax.scatter(xy[:,0],xy[:,1],s=14,c=cols[k]);ax.set_title(NAMES[k].replace('_',' ')+f' / {len(cc)} GT-visible cells')
 else:
  for keys,col in [('front_union','tab:blue'),('rear_union','tab:orange')]:
   xy=b[keys]*.1+o;ax.scatter(xy[:,0],xy[:,1],s=14,c=col,label=f'{keys}: {len(xy)}')
  ax.legend(fontsize=8);ax.set_title(f"Four-view GT union {row['all_union']} / no learned map")
ax=axes[1,2];p=b['ring_points']-root;v=b['ring_visible'];ax.scatter(p[v,0],p[v,1],c='tab:green',s=3,label=f'Eligible {v.sum()}/3600');ax.scatter(p[~v,0],p[~v,1],c='0.7',s=3,label=f'Blind {(~v).sum()}/3600');ax.scatter(0,0,c='black',marker='*');ax.legend(fontsize=8);ax.set_aspect('equal');ax.set_title('2 m ring: FOV/range/CAD only');ax.set_xlabel('Relative world X (m)');ax.set_ylabel('Relative world Y (m)')
fig.suptitle('Original four-camera geometry / first exposure 0.5 s\nGrey=bodyband obstacle reference; blank=unknown. GT-only, no predicted free cells, permanent memory or navigation.',fontsize=12)
fig.savefig(D/'first_four_bev.png',dpi=150);plt.close(fig)
