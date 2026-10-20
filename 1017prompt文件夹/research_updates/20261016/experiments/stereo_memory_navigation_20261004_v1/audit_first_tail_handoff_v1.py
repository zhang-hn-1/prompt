"""Current-boundary original first-reference tail, historical E1 only; no act/replay."""
from pathlib import Path
import json,hashlib,numpy as np
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;D=H/'native_pose_baseline35_e1_coverage_run_v1';O=H/'first_tail_handoff_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();sealed=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in sealed['artifact_sha256'].items());rows=json.loads((D/'policy_records.json').read_text());native=json.loads((D/'native_predictions.json').read_text());lookup={round(float(a['time_s']),6):a for a in native if a['available']};result=[]
for i,row in enumerate(rows):
 if i==0 or not row.get('planner_act_called') or not row.get('guard_measured_this_call'):continue
 decision=row.get('reference_decision') or {};guard=decision.get('guard')
 if guard is None or abs(float(decision['measured_time_s'])-row['time_s'])>1e-6:continue
 before=rows[i-1];ref=before.get('installed_reference_xy_after_act')
 if ref is None or before.get('local_reference_version')!=1:continue
 f=D/'maps'/f'frame_{i:03d}.npz';a=np.load(f);xy=np.asarray(lookup[round(row['time_s'],6)]['position_world'])[:2];calc=reference_geometry_status(xy,np.asarray(ref),a['labels'],a['origin'],float(a['resolution']))
 for key in ('projected_arc_m','remaining_arc_m','projection_distance_m'):assert abs(float(calc[key])-float(guard[key]))<1e-7,key
 assert calc['reject_reasons']==guard['reject_reasons'];result.append(dict(frame=i,time_s=row['time_s'],remaining_arc_m=calc['remaining_arc_m'],endpoint_distance_m=float(np.linalg.norm(xy-np.asarray(ref)[-1])),reject_reasons=calc['reject_reasons'],decision=decision['decision'],before_state=decision['state_before'],actual_request=row['request'][:2],original_reference_version_before=1,new_reference_version_after=row['local_reference_version'],source_and_native_pose_current_not_future=True))
(O/'assessment.json').write_text(json.dumps(dict(rows=result,guard_recomputed_exact=True,no_second_policy_act=True,no_actor_or_threshold_change=True,first_tail_only=True,not_bug_or_control_success_claim=True),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()}),indent=2)+'\n');print(json.dumps(result[-3:],indent=2))
