"""One first native reference rejection: current exposure teacher labels, offline only."""
from pathlib import Path
import json,numpy as np,hashlib
from reference_continuity_audit import reference_geometry_status
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
H=Path(__file__).resolve().parent;D=H/'rolling_endpoint_native_e0_b0_run_v1';f=75
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,d in seal['artifact_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==d,p
rows=json.loads((D/'policy_records.json').read_text());r=rows[f];t=r['time_s'];xy=np.asarray(r['native_current_pose']['root_position_world_m'][:2]);oldpath=rows[f-1]['installed_reference_xy_after_act'];m=np.load(D/'maps'/f'frame_{f:03d}.npz');original=reference_geometry_status(xy,oldpath,m['labels'],m['origin'],float(m['resolution']));assert original==r['reference_decision']['guard']
snap=json.loads((D/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text());filtered=[];labels=[]
for rec in snap['records']:
 sf=rec['frame'];st=rec['time_s'];assert st<=t+1e-8
 data=np.load(D/'frames'/f'frame_{sf:03d}.npz');ids=np.array(rec['query_pixel_id'],int);x=ids%640;y=ids//640;depth=data['left_depth_m'].ravel()[ids];T=data['left_camera_usd_world'];ray=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))]);teacher=T[:3,3]+(ray*depth[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 ground=np.isfinite(depth)&(depth>0)&(teacher[:,2]<=.05);keep=~ground
 labels.append(dict(source_frame=sf,source_time_s=st,query_pixels=ids.tolist(),predicted_world_height_m=np.asarray(rec['world']).reshape(-1,3)[:,2].tolist(),teacher_world_height_m=teacher[:,2].tolist(),teacher_depth_m=depth.tolist(),teacher_floor_threshold_m=.05,teacher_ground_count=int(ground.sum()),retained_non_ground_count=int(keep.sum())))
 nr=dict(rec)
 for k in ('world','query_pixel_id','partner_pixel_id'):nr[k]=(np.asarray(rec[k]).reshape(-1,3) if k=='world' else np.asarray(rec[k]))[keep]
 filtered.append(nr)
grid=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);grid.set_source(dict(frame=r['source_frame'],time_s=r['source_time_s'],records=filtered));grid.update_points([],np.array(r['native_current_pose']['root_position_world_m']),t);diagnostic=reference_geometry_status(xy,oldpath,grid.labels,grid.origin,grid.resolution)
result=dict(frame=f,time_s=t,original_guard=original,teacher_ground_removed_guard=diagnostic,labels=labels,original_reference_decision=r['reference_decision']['decision'],installed_plan_direction_changes=r['plan']['direction_changes'],same_current_pose_old_reference=True,teacher_labels_never_actor_inputs=True,offline_oracle_ablation_not_online_candidate=True,unknown_not_observed_free=True,not_collision_counterfactual=True,no_new_physics_or_act=True,source_accuracy_not_navigation_prerequisite=True,scope='specific first current depth-derived floor hazard changes retention predicate; does not prove whole navigation result or prescribe threshold/net tuning',whole_mission_complete=False,input_manifest_sha256=hashlib.sha256((D/'artifact_manifest_post_close.json').read_bytes()).hexdigest())
p=H/'native_e0_first_ground_risk_audit_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(labels=labels,original_pass=original['retention_necessary_screen_pass'],teacher_ground_removed_pass=diagnostic['retention_necessary_screen_pass'],original_hazard_count=original['observed_hazard_count'],removed_hazard_count=diagnostic['observed_hazard_count'],installed_changes=r['plan']['direction_changes'])))
