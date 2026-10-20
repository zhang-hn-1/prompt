"""Readonly measured failure diagnosis, not an action counterfactual label."""
from pathlib import Path
import numpy as np,json,datetime,hashlib
from qualified_hazard_consumer import subset
from dense_source_points import records
from point_support_shadow import PointSupportMap
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';states=json.loads((O/'states.json').read_text());policy=json.loads((O/'policy_records.json').read_text());seednp=np.load(O/'qualified_sensor_packets/source_180.npz');seed={k:(seednp[k].item() if seednp[k].ndim==0 else seednp[k]) for k in seednp.files};seed=subset(seed,(seed['world'][:,2]>.05)&(seed['world'][:,2]<=1.5));rows=[]
truth=np.load(O/'offline_occupancy_label.npz');all_occ=np.where(truth['labels']>0,1,-1).astype(np.int8)
for frame in [180,205,230,280,285,295]:
 m=np.load(O/'maps'/f'frame_{frame:03d}.npz');t=float(m['time_s']);plan=np.load(O/'local_plans'/f'frame_{frame:03d}.npz');path=plan['pose'][:,:2];xy=np.asarray(states[frame]['position'])[:2]
 saved=reference_geometry_status(xy,path,m['labels'],m['origin'],float(m['resolution']))
 historic=PointSupportMap(m['origin'],m['labels'].shape,float(m['resolution']));historic.update(records(seed),t,0.);h=historic.snapshot(t);merged=m['labels'].copy();merged[h['labels']==1]=1;past=reference_geometry_status(xy,path,merged,m['origin'],float(m['resolution']))
 # Independent occupancy proxy only for AFTER-CLOSE diagnosis, never online decision.
 actual=reference_geometry_status(xy,path,all_occ,truth['origin'],float(truth['resolution']))
 rows.append(dict(frame=frame,time_s=t,seed_age_s=t-seed['time_s'],current_hazard_cells=int((m['labels']==1).sum()),unexpired_seed_hazard_cells=int((h['labels']==1).sum()),current_reference_guard=saved,readonly_old_geometry_overlay_guard=past,offline_proxy_truth_reference_guard=actual,reference_created_time_s=float(plan['time_s']),reference_version=int(plan['version']),actual_request=policy[frame]['local']['request']))
ph=np.load(O/'physics.npz');speed=np.linalg.norm(ph['linear_velocity_world_m_s'][:,:2],axis=1);qualified=speed[ph['simulation_time_s']>=18-1e-6];bodycurve=[]
for i in range(180,len(policy)):
 plan=np.load(O/'local_plans'/f'frame_{i:03d}.npz');s=reference_geometry_status(np.asarray(states[i]['position'])[:2],plan['pose'][:,:2],all_occ,truth['origin'],float(truth['resolution']));bodycurve.append(dict(frame=i,time_s=states[i]['time_s'],actual_projection_distance_m=s['projection_distance_m'],offline_proxy_nominal_suffix_clearance_m=s['suffix_clearance_m']))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,posthandoff_actual_XY_speed_peak_m_s=float(qualified.max()),posthandoff_actual_XY_speed_mean_m_s=float(qualified.mean()),same_scene_known_grid_point_path_length_prior_m=json.loads((H/'short_task_route_length_diagnostic.json').read_text())['point_grid_path_m'],remaining_original_budget_after_handoff_s=17.,not_a_physical_speed_upper_bound=True,not_valid_counterfactual_trajectory=True,offline_truth_only_diagnosis=True,no_future_inputs_or_action_change=True,old_candidate_REJECT_preserved=True,bodycurve=bodycurve,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
f=H/'qualified_hazard_failure_audit_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print([(r['frame'],r['current_hazard_cells'],r['unexpired_seed_hazard_cells'],r['current_reference_guard']['retention_necessary_screen_pass'],r['readonly_old_geometry_overlay_guard']['retention_necessary_screen_pass'],r['offline_proxy_truth_reference_guard']['suffix_clearance_m']) for r in rows]);print('actualspeed',result['posthandoff_actual_XY_speed_peak_m_s'],result['posthandoff_actual_XY_speed_mean_m_s'])
