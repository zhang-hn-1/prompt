"""Bounded archived current-feedback arithmetic audit; no policy act or physics."""
from pathlib import Path
import json, hashlib, datetime
import numpy as np
from ball_wm.benchmark.drive_model import DriveModel
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
D=H/'endpoint_tail_stop_rule_audit_v1';D.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
inputs=[P/'benchmark/policies.py',P/'benchmark/drive_model.py',P/'configs/benchmark_v1/drive_dynamics_v1.json',H/'estimated_navigation_observation.py']
for tag in ['endpoint_tail_e1_run_v1','native_pose_baseline35_e1_coverage_run_v1']:
 inputs.extend(H/tag/f for f in ['native_predictions.json','policy_records.json','task_episode.json','artifact_manifest_post_close.json'])
protocol=dict(question='Does original local-reference stopping-distance rule numerically explain first goal_braking, without changing any controller?',scope='trial frames160..165 and historical B0 last tracking/first goal_braking in first reference if present',no_policy_replay=True,no_GT_input=True,no_new_physics=True,no_threshold_change=True,frozen_before_arithmetic_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs})
(D/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
model=DriveModel.from_json(P/'configs/benchmark_v1/drive_dynamics_v1.json')
out=[]
for tag in ['endpoint_tail_e1_run_v1','native_pose_baseline35_e1_coverage_run_v1']:
 d=H/tag;seal=json.loads((d/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in seal['artifact_sha256'].items())
 rows=json.loads((d/'policy_records.json').read_text());native=json.loads((d/'native_predictions.json').read_text());nt=np.array([r['time_s'] for r in native]);ep=json.loads((d/'task_episode.json').read_text())['actual_episode'];goal=np.array(ep['goal_position_world_m'][:2]);indices=list(range(160,166)) if tag.startswith('endpoint') else [r['frame'] for r in rows if r.get('local_reference_version')==1 and r.get('tracker_state_after_act',{}).get('braking_state')=='goal_braking'][:2]
 for i in indices:
  row=rows[i];t=row['time_s'];j=int(np.searchsorted(nt,t+1e-9,side='right')-1);n=native[j];assert n['available'] and abs(n['time_s']-t)<1e-8
  pos=np.array(n['position_world'][:2]);speed=float(np.linalg.norm(n['velocity_world_m_s'][:2]));ref=np.array(row['installed_reference_xy_after_act']);k=row['tracker_state_after_act']['path_index'];direction=row['tracker_state_after_act']['active_direction'];remaining=float(np.linalg.norm(ref[k]-pos)+np.linalg.norm(np.diff(ref[k:],axis=0),axis=1).sum());stopping=model.stopping_distance(speed,direction);bound=stopping+.1
  out.append(dict(run=tag,frame=i,time_s=t,native_index=j,native_clock=n['time_s'],local_reference_version=row['local_reference_version'],path_index=k,feedback_speed_m_s=speed,remaining_waypoint_polyline_m=remaining,V1_stopping_distance_m=stopping,original_stop_bound_m=bound,local_stop_condition=remaining<=bound,global_pointgoal_distance_m=float(np.linalg.norm(goal-pos)),global_goal_condition=float(np.linalg.norm(goal-pos))<=.2,actual_braking_state=row['tracker_state_after_act']['braking_state'],actual_request=row['request'][:2],scope='after-act reference; replacement frame165 excluded from first-reference stop attribution' if i==165 and tag.startswith('endpoint') else 'same installed reference/current native feedback'))
first=next(r for r in out if r['run'].startswith('endpoint') and r['frame']==164);assert first['local_reference_version']==1 and first['local_stop_condition'] and not first['global_goal_condition'] and first['actual_braking_state']=='goal_braking' and first['actual_request']==[0.,0.]
prev=next(r for r in out if r['run'].startswith('endpoint') and r['frame']==163);assert not prev['local_stop_condition'] and prev['actual_braking_state']=='tracking'
assessment=dict(rows=out,decision='Original local-reference stopping rule explains first trial braking; global navigation goal remains distant. Intentional local endpoint braking is not itself a proven bug.',candidate_rejection_preserved=True,hazard_screen_not_trigger_of_frame165_rebuild=True,not_fullcausal_AB=True,no_control_change=True,whole_mission_complete=False)
(D/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n');(D/'artifact_manifest.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in D.iterdir() if p.is_file()}),indent=2)+'\n');print(json.dumps(assessment,indent=2))
