from pathlib import Path
import json,hashlib,datetime,numpy as np
from bilateral_core_memory import BilateralCoreMemory
from ray_contradiction_memory import contradict_cores
from partial_cue_source_v2 import inflate_xy
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'first_ready_core_handoff_preflight_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
op={r['frame']:r for r in json.loads((D/'source_opportunity_records.json').read_text())};m=BilateralCoreMemory();pairs=[];inputs={};init=np.load(D/'estimated_poses/frame_000.npz')['root_position_world_m'][:2];selected=None
for i in range(0,141,5):
 name=f'frame_{i:03d}.npz';files=[D/'possible_sets'/name,D/'sensor_packets'/f'source_{i:03d}.npz',D/'estimated_poses'/name,D/'policy_inputs'/name]
 for f in files:inputs[str(f)]=sha(f)
 q=dict(np.load(files[0]));packet=dict(np.load(files[1]));pose=dict(np.load(files[2]));policy=np.load(files[3]);t=float(q['time_s']);m.update(q,t);pairs.append((packet,pose));speeds=np.linalg.norm(policy['proprio_history'][-5:,:2],axis=1);eligible=op[i]['source_event'] is not None and not bool(q['pending_seed']) and len(q['raw_world_voxel_centres'])>=100 and len(speeds)==5 and np.isfinite(speeds).all() and bool((speeds<.05).all()) and np.linalg.norm(pose['root_position_world_m'][:2]-init)<=.7
 if eligible:
  selected=dict(frame=i,time_s=t,selection='FIRST_READY_EVENT_NO_LATER_DATE_SEARCH',speed_max=float(speeds.max()),displacement=float(np.linalg.norm(pose['root_position_world_m'][:2]-init)));break
assert selected is not None,'noeligiblefirsthandoff';pool=m.freeze(t);raw=pool['raw_world_voxel_centres'];veto,proof=contradict_cores(raw,pairs,t);seed=dict(pool,raw_world_voxel_centres=raw[~veto],possible_xy=inflate_xy(raw[~veto,:2],.2),source_consumer='first_ready_negative_ray_history',not_measured_free=True);np.savez_compressed(O/'first_seed_pretruth.npz',**seed);(O/'selected_actor_context.json').write_text(json.dumps(selected,indent=2)+'\n');(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=now(),inputs=inputs,code_sha256=sha(__file__),outputs={str(f):sha(f) for f in O.iterdir()},no_future_after_selected=True),indent=2)+'\n')
# Original source checks after sealed FIRST selection only.
truth=np.load(D/'offline_occupancy_label.npz');gtxy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);covered=cKDTree(seed['possible_xy']).query(gtxy)[0]<=np.sqrt(2)*.05+1e-9;inside=(np.linalg.norm(raw[:,:2],axis=1)<=.35)&(raw[:,2]>=0)&(raw[:,2]<=1);Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);views=[]
for window in m.windows:
 for vf in window['support_frames']:
  for eye in ('left','right'):
   cl=np.load(D/'partial_class_masks'/f"birth_{window['frame']:03d}_{eye}_{vf:03d}.npz")['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world'];mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
   with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
   valid=np.isfinite(z)&np.isfinite(world).all(-1)&(z>=.5)&(z<=3)&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)&~mask;views.append(dict(window=window['frame'],frame=vf,eye=eye,precision=float((cl&valid).sum()/cl.sum()) if cl.any() else 0.))
checks=dict(coverage_ge95=float(covered.mean())>=.95,area_le6=len(seed['possible_xy'])*.01<=6,all_contributing_class_precision_ge95=bool(views) and all(v['precision']>=.95 for v in views),no_true_occupied_veto=int((inside&veto).sum())==0);a=dict(utc=now(),decision='PASS_FIRST_READY_HISTORICAL_SOURCE_ONLY' if all(checks.values()) else 'REJECT_FIRST_READY_SOURCE_NO_LATER_DATE_PICK',selected=selected,checks=checks,covered_physical_cells=int(covered.sum()),physical_cells=len(covered),coverage=float(covered.mean()),MODELarea_m2=len(seed['possible_xy'])*.01,cores=len(seed['raw_world_voxel_centres']),wrong_veto=int((inside&veto).sum()),class_min=min(v['precision'] for v in views),class_views=views,source_not_observation_or_full3D=True,no_actual_or_formal35_trigger=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
