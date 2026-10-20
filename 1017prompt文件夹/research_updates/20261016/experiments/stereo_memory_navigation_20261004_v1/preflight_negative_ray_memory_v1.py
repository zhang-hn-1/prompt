from pathlib import Path
import sys,json,datetime,hashlib,numpy as np,cv2
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from bilateral_core_memory import BilateralCoreMemory
from ray_contradiction_memory import contradict_cores
from partial_cue_source_v2 import inflate_xy
from persistent_core_grid import PersistentCoreGrid
from tracking_tube_planner import local_curve_for_direction_tube,planning_clearance
from oracle_rgb_shadow import make_oracle
from training.action_protocol import load_action_protocol
from scipy.spatial.transform import Rotation
D=H/'bank_rock_observation_run_v1';O=H/'negative_ray_memory_preflight_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
man=json.loads((D/'artifact_manifest_post_close.json').read_text());assert man['complete'] and all(sha(f)==s for f,s in man['artifact_sha256'].items())
m=BilateralCoreMemory()
for i in range(0,141,5):
 q=dict(np.load(D/'possible_sets'/f'frame_{i:03d}.npz'));m.update(q,float(q['time_s']))
pool=m.freeze(m.last_time);pairs=[(dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')),dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'))) for i in range(0,141,5)];raw=pool['raw_world_voxel_centres'].copy();veto,proof=contradict_cores(raw,pairs,m.last_time)
original_cores=retained_provenance;retained_provenance=[r for r,b in zip(original_cores,veto) if not b];pool['raw_world_voxel_centres']=raw[~veto];pool['possible_xy']=inflate_xy(raw[~veto,:2],.2);pool['source_consumer']='persistent_bilateral_negative_ray_v1';np.savez_compressed(O/'pool.npz',**pool)
(O/'ray_proofs.json').write_text(json.dumps([dict(core_xyz=c.tolist(),contradicted=bool(b),past_ray_proofs=p) for c,b,p in zip(raw,veto,proof)],indent=2)+'\n')
(O/'core_provenance.json').write_text(json.dumps(dict(windows=m.windows,cores=retained_provenance,per_point_measured_timestamps_available=False),indent=2)+'\n')
(O/'outputs_pretruth_seal.json').write_text(json.dumps({str(O/n):sha(O/n) for n in ['pool.npz','core_provenance.json','ray_proofs.json']},indent=2)+'\n')
# Frozen planner consumer runs before truth labels; no policy.act or actual physics.
init=np.load(D/'estimated_poses/frame_000.npz');pose=np.load(D/'estimated_poses/frame_140.npz');g=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60);g.seed(pool,float(pose['time_s']))
g.set_source(dict(np.load(D/'sensor_packets/source_140.npz')));g.update_points(np.empty((0,3)),pose['root_position_world_m'],float(pose['time_s']))
route,status=g.plan(pose['root_position_world_m'][:2],np.array([2.2,.15]));cell=g.cells(pose['root_position_world_m'][None,:2])[0];clear=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');tr=make_oracle(load_action_protocol(P/'configs/data_contract.yaml'));yaw=float(pose['yaw_world_rad'])
plan,meta=local_curve_for_direction_tube(g,pose['root_position_world_m'][:2],yaw,[2.2,.15],tr.drive_model,initial_direction=None)
planner=dict(route_available=route is not None,route_status=status,raster_start_clearance_m=float(clear[tuple(cell)]),required_raster_clearance_m=planning_clearance(g),local_kinodynamic_available=plan is not None,local_meta=meta,measured_free=int(np.isfinite(g.memory.floor_time).sum()),measured_hazard=int(np.isfinite(g.memory.hazard_time).sum()))
(O/'planner_pretruth.json').write_text(json.dumps(planner,indent=2)+'\n')
# Offline diagnostic labels only after actor artifacts were frozen.
truth=np.load(D/'offline_occupancy_label.npz');gtxy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(pool['possible_xy']).query(gtxy)[0]<=np.sqrt(2)*.05+1e-9).mean());area=len(pool['possible_xy'])*.01
Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);views=[]
for w in m.windows:
 if not w['raw_core_count']:continue
 for f in sorted((D/'partial_class_masks').glob(f"birth_{w['frame']:03d}_*.npz")):
  eye,vf=f.stem.split('_')[-2:];vf=int(vf);assert vf<=w['frame'];cl=np.load(f)['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world'];mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(z)&np.isfinite(world).all(-1)&(z>=.5)&(z<=3)&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)&~mask
  views.append(dict(window_frame=w['frame'],exposure_frame=vf,eye=eye,pixels=int(cl.sum()),precision=float((cl&valid).sum()/cl.sum()) if cl.any() else 0.))
true_volume=(np.linalg.norm(raw[:,:2],axis=1)<=.35)&(raw[:,2]>=0)&(raw[:,2]<=1);wrong_veto=int((veto&true_volume).sum())
checks=dict(no_true_occupied_veto=wrong_veto==0,coverage_ge95=coverage>=.95,area_le6=area<=6,all_contributing_class_precision_ge95=bool(views) and all(v['precision']>=.95 for v in views),frozen_raster_route=route is not None,frozen_local_kinodynamic=plan is not None,not_measured=planner['measured_free']==0 and planner['measured_hazard']==0)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_PREFLIGHT_ONLY' if all(checks.values()) else 'REJECT_UNIQUE_NEGATIVE_RAY_MEMORY',checks=checks,coverage=coverage,possible_area_m2=area,raw_core_count=len(pool['raw_world_voxel_centres']),contradicted_core_count=int(veto.sum()),wrong_veto=wrong_veto,windows=len(m.windows),minimum_contributing_class_precision=min(v['precision'] for v in views),class_views=views,planner=planner,historical_not_fresh=True,no_GT_actor_or_future=True,static_persistence_is_prior=True,old_rocking_end_gate_reject_preserved=True,actual_navigation_triggered=False,whole_research_complete=False,source_sha256={str(H/n):sha(H/n) for n in ['bilateral_core_memory.py','persistent_core_grid.py','preflight_negative_ray_memory_v1.py','ray_contradiction_memory.py']})
(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['class_views','source_sha256','planner']},indent=2));print(json.dumps({k:v for k,v in planner.items() if k!='local_meta'},indent=2))
