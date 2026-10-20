"""Offline causal boundary evidence only; no alternative controls or new runs."""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;r=H/'online_bank_hold_navigation_run';rows=json.loads((r/'vision_records.json').read_text());states=json.loads((r/'policy_states.json').read_text());p=np.load(r/'physics.npz')
selected=(60,100,150,200,250,284,298,300,340,349);output=[]
for i in selected:
 row=rows[i];j=i*5;xy=np.asarray(row['position_world_xy']);yaw=float(p['yaw_world_rad'][j]);v=p['linear_velocity_world_m_s'][j,:2];speed=float(np.linalg.norm(v))
 target=np.asarray(row['target_base_xy']) if row['target_base_xy'] is not None else None
 desired=None if target is None else float(np.arctan2(target[1],target[0]))
 course=None if speed<.05 else float(np.arctan2(v[1],v[0]));gap=None if course is None else float(np.arctan2(np.sin(course-yaw),np.cos(course-yaw)))
 state=states[i];path=np.asarray(state['path_xy']) if state['path_xy'] is not None else None
 output.append(dict(frame=i,time_s=row['time_s'],route_reason=row['route_reason'],current_xy=xy.tolist(),actual_xy_speed_m_s=speed,actual_xy_course_rad=course,body_yaw_rad=yaw,course_minus_body_yaw_rad=gap,
     target_heading_in_body_rad=desired,baseline_final_request=row['baseline_final_request'][:2],final_request=row['request'][:2],stop_veto=row['stopping_consumer']['stop_veto'],held=row['bank_stop_hold']['active'],path_vertices=None if path is None else len(path),path_endpoint=None if path is None else path[-1].tolist(),direction=row['direction'],latch_low_count=row['low_speed_count'],physical_stop_low_count=row['stopping_consumer']['low_speed_count']))
# Exact causal last map already replayed in mainassessment; saved1Hz map340 gives independently visible start/goal clearance.
m=np.load(r/'belief_snapshots/frame_340.npz');labels=m['labels'];origin=m['origin'];res=float(m['resolution']);row=rows[340];xy=np.asarray(row['position_world_xy']);cell=np.rint((xy-origin)/res).astype(int)
clear=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res;haz=np.argwhere(labels==1);hazworld=origin+haz*res
nearest=float(np.linalg.norm(hazworld-xy,axis=1).min())
result=dict(scope='offline current/past row diagnostics; no future labels consumed by controller; no new acceptance/threshold changes',rows=output,
  post_event=dict(held_blocks=sum(v['bank_stop_hold']['active'] for v in rows),zero_main_blocks_after_event=int(sum(np.all(np.asarray(v['request'])[0::2]==0) for v in rows[284:])),actual_resume=False,
    veto_blocks_after_event=sum(v['stopping_consumer']['stop_veto'] for v in rows[284:]),route_reason_counts={k:sum(v['route_reason']==k for v in rows[284:]) for k in sorted({v['route_reason'] for v in rows[284:]})}),
  last_saved_map340=dict(start_cell=cell.tolist(),start_cell_observed_hazard_clearance_m=float(clear[tuple(cell)]),nearest_observed_hazard_center_distance_m=nearest,start_passes_original_Astar_point6_rule=bool(clear[tuple(cell)]>=.6-1e-8),stopping_veto_occupied_cells=rows[340]['stopping_consumer']['occupied_cells'],unknown_cells=int((labels<0).sum()),body_prior_observed_free=rows[340]['body_prior_observed_free']),
  causal_conclusion='unchanged A* often still proposes motion after held-bank brake, but unchanged observed-hazard stopping consumer vetoes all66postevent main requests. This is no evidence that latch/qualification blocks resumption. Pre-event large tracking/reference error already exists and is unchanged by a stop-only intervention.',
  next_bounded_audit='reuse archived pre-event motion, A* paths and provenance hazards; quantify actualvelocitycourse/bodyyaw/target vs path and truehazard distances. Establish whether deviation originates in chosen unknown-risk route, geometric tracking request or measuredexecution before one path/feedback consumer change; no new brake radius/bank/PID/model scan',
  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),r/'vision_records.json',r/'policy_states.json',r/'physics.npz',r/'belief_snapshots/frame_340.npz')})
(H/'online_bank_hold_remaining_gap.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
