from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial import cKDTree
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;O=H/'fixed_appearance_hypothesis_source_v1';D=H/'native_fixed_appearance_probe_run_v1'
seal=json.loads((O/'actor_seal_pretruth.json').read_text())
for group in ('input_sha256','source_sha256','output_sha256'):
 for f,s in seal[group].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
seed=dict(np.load(O/'seed_pretruth.npz'));raw=np.load(O/'unfiltered_pool.npz')['raw_world_voxel_centres'];veto=np.load(O/'core_veto.npz')['veto'];truth=np.load(H/'controlled_obstacle_fixture_v1/occupancy.npz');xy=truth['global_origin_xy']+np.argwhere(truth['grid']>0)*float(truth['resolution_m']);coverage=cKDTree(seed['possible_xy']).query(xy)[0]<=np.sqrt(2)*.05+1e-9
inside=(np.linalg.norm(raw[:,:2],axis=1)<=.35)&(raw[:,2]>=0)&(raw[:,2]<=1);prov=json.loads((O/'core_provenance.json').read_text());cad=RobotSelfMask();Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);views=[]
for w in prov['windows']:
 for frame in w['support_frames']:
  gt=np.load(D/'frames'/f'frame_{frame:03d}.npz');hist=np.load(D/'history'/f'frame_{frame:03d}.npz');masks=cad.project(joints_from_proprio(hist['proprio_history']))
  for eye in ('left','right'):
   cl=np.load(O/'class_masks'/f"birth_{w['frame']:03d}_{eye}_{frame:03d}.npz")['potential_hazard_mask'];z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world']
   with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
   valid=np.isfinite(z)&np.isfinite(world).all(-1)&(z>=.5)&(z<=3)&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)&~masks[eye]
   views.append(dict(window=w['frame'],frame=frame,eye=eye,count=int(cl.sum()),precision=float((cl&valid).sum()/cl.sum()) if cl.any() else 0.))
checks=dict(coverage_ge95=float(coverage.mean())>=.95,area_le6=len(seed['possible_xy'])*.01<=6,all_contributing_class_precision_ge95=bool(views) and all(v['precision']>=.95 for v in views),no_true_occupied_veto=int((inside&veto).sum())==0)
a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_DEVELOPMENT_MODEL_APPLICABILITY_ONLY' if all(checks.values()) else 'REJECT_NEW_APPEARANCE_FROZEN_HYPOTHESIS_SOURCE',checks=checks,covered_physical_cells=int(coverage.sum()),physical_cells=len(coverage),coverage=float(coverage.mean()),MODELarea_m2=len(seed['possible_xy'])*.01,unfiltered_cores=len(raw),filtered_cores=len(seed['raw_world_voxel_centres']),wrong_veto=int((inside&veto).sum()),class_min=min(v['precision'] for v in views),class_views=views,actor_seal_verified=True,not_fresh=True,source_not_observation_or_full3D=True,no_navigation_or_formal35_trigger=True,whole_research_complete=False)
(O/'assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k!='class_views'},indent=2))
