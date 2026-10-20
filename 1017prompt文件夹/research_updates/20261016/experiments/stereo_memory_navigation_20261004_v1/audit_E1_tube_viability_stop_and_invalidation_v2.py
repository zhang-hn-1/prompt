"""Bounded factual sequencing: qualification is measured, never an alternate action label."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'E1_tube_viability_stop_and_invalidation_v2';O.mkdir(exist_ok=False)
for p,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==s
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());z=np.load(D/'physics.npz');speed=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1)[::5]
arm=next(i for i,r in enumerate(rows) if r['tube_viability_event_armed']);rebuild=next(i for i,r in enumerate(rows) if i>arm and r['local_plan_created']);stop=next(i for i,r in enumerate(rows) if i>rebuild and r['tracker_state_after_act']['pending_direction']=='reverse');block=next(i for i,r in enumerate(rows) if r['plan'].get('status')=='planning_commitment_blocks_endpoint')
assert arm<rebuild<stop<block
counter=0;qualifications=[];events=[]
for i in range(stop,len(rows)):
 counter=counter+1 if speed[i]<.05 else 0
 if counter==5:qualifications.append(i)
 if i%5==0 or i in [stop,block] or counter==5:
  m=np.load(D/'maps'/f'frame_{i:03d}.npz');g=StaticSparsePointRiskGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'];c=g.cells(np.asarray(states[i]['position'][:2])[None])[0];clear=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution
  events.append(dict(frame=i,time_s=rows[i]['time_s'],actual_XY_speed_m_s=float(speed[i]),measured_low_speed_counter=counter,recorded_tracker_counter=rows[i]['tracker_state_after_act']['stopped_cycles'],current_start_raster_clearance_m=float(clear[tuple(c)]),plan_status=rows[i]['plan'].get('status'),tracker_act_called_by_actual_branch=rows[i]['plan'].get('status')=='success',outer_policy_called=True,request_zero=bool(np.array_equal(rows[i]['request'],np.zeros(10)))))
assert qualifications
r=dict(decision='FACTUAL_STOP_AND_PLAN_INVALIDATION_SEQUENCE_ONLY',armed_frame=arm,first_postarm_rebuild_frame=rebuild,first_postarm_rebuild_direction_changes=rows[rebuild]['plan']['direction_changes'],next_reverse_pending_frame=stop,first_no_route_frame=block,first_measured_five_low_speed_frame=qualifications[0],first_measured_five_low_speed_time_s=rows[qualifications[0]]['time_s'],measured_qualification_after_plan_invalid=True,tracker_counter_does_not_update_without_valid_plan=all(rows[i]['tracker_state_after_act']['stopped_cycles']==0 for i in range(block,len(rows))),outer_planner_act_called_flag_is_not_inner_tracker_act_evidence=True,not_safe_to_execute_stale_reference=True,not_proof_counter_fix_recovers_route=True,no_actions_or_gates_changed=True,no_alternate_physical_labels=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(O/'events.json').write_text(json.dumps(events,indent=2)+'\n');print(json.dumps(r,indent=2))
