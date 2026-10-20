from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial import cKDTree
from ray_contradiction_memory import contradict_cores
from partial_cue_source_v2 import inflate_xy
from persistent_core_grid import PersistentCoreGrid
from tracking_tube_planner import planning_clearance
H=Path(__file__).resolve().parent;D=H/'negative_ray_memory_R1_run_v2';O=H/'moving_ray_stop_boundary_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
full=dict(np.load(D/'unfiltered_historical_pool.npz'));old=dict(np.load(D/'frozen_memory_seed.npz'));raw=full['raw_world_voxel_centres'];pairs=[];inputs={}
for i in range(0,266,5):
 pp=D/'sensor_packets'/f'source_{i:03d}.npz';ep=D/'estimated_poses'/f'frame_{i:03d}.npz';inputs[str(pp)]=sha(pp);inputs[str(ep)]=sha(ep);pairs.append((dict(np.load(pp)),dict(np.load(ep))))
now=float(pairs[-1][1]['time_s']);veto,proof=contradict_cores(raw,pairs,now);packet=dict(old,raw_world_voxel_centres=raw[~veto],possible_xy=inflate_xy(raw[~veto,:2],.2));np.savez_compressed(O/'causal_hypothesis_packet.npz',**packet)
(O/'proofs.json').write_text(json.dumps([dict(core_xyz=c.tolist(),contradicted=bool(v),past_ray_proofs=p) for c,v,p in zip(raw,veto,proof)],indent=2)+'\n')
init=np.load(D/'estimated_poses/frame_000.npz');pose=pairs[-1][1];g=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60);g.set_source(dict(time_s=float(init['time_s'])));g.update_points(np.empty((0,3)),init['root_position_world_m'],float(init['time_s']));g.seed(packet,float(packet['time_s']));g.set_source(pairs[-1][0]);g.update_points(np.empty((0,3)),pose['root_position_world_m'],now);route,status=g.plan(pose['root_position_world_m'][:2],np.array([2.2,.15]));np.savez_compressed(O/'query.npz',labels=g.labels,root=pose['root_position_world_m'],time_s=now,route=np.empty((0,2)) if route is None else route)
(O/'actor_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),code_sha256=sha(__file__),inputs=inputs,outputs={str(p):sha(p) for p in O.iterdir() if p.suffix in ['.npz','.json']},only_actual_past_through_frame265=True,no_policy_act=True),indent=2)+'\n')
# Post-seal actual labels; no point is removed based on these labels.
truth=np.load(D/'offline_occupancy_label.npz');xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(packet['possible_xy']).query(xy)[0]<=np.sqrt(2)*.05+1e-9).mean());inside=(np.linalg.norm(raw[:,:2],axis=1)<=.35)&(raw[:,2]>=0)&(raw[:,2]<=1);wrong=int((inside&veto).sum());area=len(packet['possible_xy'])*.01
checks=dict(coverage_ge95=coverage>=.95,area_le6=area<=6,no_true_occupied_veto=wrong==0,original_planner_route_exists=route is not None)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_MOVING_RAY_FEASIBILITY_ONLY' if all(checks.values()) else 'REJECT_UNIQUE_MOVING_RAY_AT_STOP_BOUNDARY',checks=checks,time_s=now,last_frame=265,source_updates=len(pairs),vetoed_cores=int(veto.sum()),retained_cores=int((~veto).sum()),coverage=coverage,possible_area_m2=area,wrong_true_occupied_veto=wrong,original_plan_status=status,planning_clearance_m=planning_clearance(g),not_fresh=True,not_online=True,not_new_navigation=True,prior_pose_and_navigation_rejects_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
