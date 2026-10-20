from pathlib import Path
import sys,json,datetime,numpy as np,hashlib
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
sys.path.insert(0,str(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
sys.path.insert(0,str(H.parent/'clean_visual_hard0_intervention_20261003_v1'))
sys.path.insert(0,str(H.parent/'visible_surface_connectivity_20261004_v1'))
from tracking_tube_planner import TrackingTubeGrid
from scipy.ndimage import distance_transform_edt
rows=[]
for case,y in [('flat_ground_pose_D3_run_v1',0.),('offset_replication_D4_run_v1',.15)]:
 D=H/case;seed=dict(np.load(H/'occluded_eye_preflight_v1'/(case+'.npz')));init=np.load(D/'estimated_poses/frame_000.npz');pose=np.load(D/'estimated_poses/frame_060.npz');g=TrackingTubeGrid(init['root_position_world_m'][:2]-20,retention_s=60);g.seed(seed,float(seed['time_s']));g.update_points(np.empty((0,3)),pose['root_position_world_m'],float(pose['time_s']));route,status=g.plan(pose['root_position_world_m'][:2],np.array([2.2,y]));cell=g.cells(pose['root_position_world_m'][None,:2])[0];clear=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution
 rows.append(dict(case=case,actor_pose_current_estimated=True,only_source_candidate_changed=True,time_s=float(pose['time_s']),raster_start_clearance_m=float(clear[tuple(cell)]),required_raster_clearance_m=.8707106781186548,route_available=route is not None,route_status=status,path_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()) if route is not None else None))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,decision='PASS_LATTICE_START_ONLY' if all(v['route_available'] for v in rows) else 'REJECT_SOURCE_PLANNER_CONSUMPTION',historical_not_fresh=True,no_actual_or_policy_act=True,no_priors_written_as_observation=True,whole_research_complete=False,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());p=H/'occluded_eye_planner_preflight_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
