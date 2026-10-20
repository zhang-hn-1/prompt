from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;O=H/'fixed_appearance_hypothesis_source_v1';D=H/'native_fixed_appearance_probe_run_v1';A=H/'fixed_appearance_hypothesis_rejection_cause_v1';A.mkdir(exist_ok=False)
assessment=json.loads((O/'assessment_v1.json').read_text());Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);cad=RobotSelfMask();cache={};rows=[]
for v in assessment['class_views']:
 f=v['frame'];eye=v['eye'];cl=np.load(O/'class_masks'/f"birth_{v['window']:03d}_{eye}_{f:03d}.npz")['potential_hazard_mask']
 if f not in cache:
  g=dict(np.load(D/'frames'/f'frame_{f:03d}.npz'));m=cad.project(joints_from_proprio(np.load(D/'history'/f'frame_{f:03d}.npz')['proprio_history']));cache[f]=(g,m)
 g,m=cache[f];z=g[eye+'_depth_m'];T=g[eye+'_camera_usd_world']
 with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 remaining=cl.copy();reasons={}
 for name,bad in [('invalid_or_out_of_range_depth',~np.isfinite(z)|(z<.5)|(z>3)),('invalid_world',~np.isfinite(w).all(-1)),('height_outside_hazard_domain',(w[:,:,2]<=.05)|(w[:,:,2]>1.5)),('robot_proximity',np.linalg.norm(w-g['root_position_world_m'],axis=-1)<=.55),('CAD_occlusion',m[eye])]:
  reasons[name]=int((remaining&bad).sum());remaining&=~bad
 assert int(remaining.sum())+sum(reasons.values())==int(cl.sum())
 rows.append(dict(v,valid_count=int(remaining.sum()),disjoint_invalid_reasons=reasons))
truth=np.load(H/'controlled_obstacle_fixture_v1/occupancy.npz');xy=truth['global_origin_xy']+np.argwhere(truth['grid']>0)*float(truth['resolution_m']);seed=np.load(O/'seed_pretruth.npz');possible=seed['possible_xy'];covered=cKDTree(possible).query(xy)[0]<=np.sqrt(2)*.05+1e-9;np.savez_compressed(A/'coverage_diagnostic.npz',proxy_xy=xy,covered=covered,possible_xy=possible,cores=seed['raw_world_voxel_centres'])
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(6,6));ax.scatter(possible[:,0],possible[:,1],marker='s',s=100,alpha=.2,label='MODEL possible XY');ax.scatter(xy[covered,0],xy[covered,1],s=10,label='proxy covered');ax.scatter(xy[~covered,0],xy[~covered,1],s=18,c='red',label='proxy missing');ax.scatter(seed['raw_world_voxel_centres'][:,0],seed['raw_world_voxel_centres'][:,1],s=8,c='black',label='MODEL cores');ax.set_aspect('equal');ax.set(xlabel='world X [m]',ylabel='world Y [m]',title='Rejected frozen source: 134 / 185 proxy cells');ax.legend();fig.tight_layout();fig.savefig(A/'coverage.png',dpi=150)
a=dict(decision='ATTRIBUTION_ONLY_NO_NEW_CANDIDATE',class_views=rows,failed_views=[r for r in rows if r['precision']<.95],missing_proxy_cells=int((~covered).sum()),no_actor_truth_repair=True,not_fresh=True,no_navigation_trigger=True);(A/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({'views':len(rows),'failed_views':len(a['failed_views']),'missing_cells':a['missing_proxy_cells'],'worst':min(rows,key=lambda r:r['precision'])},indent=2))
