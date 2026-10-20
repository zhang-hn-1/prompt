"""Archived causal-input harness, not an online shadow/interface validation."""
from pathlib import Path
import json,copy,hashlib,datetime,numpy as np
from local_endpoint_event_v1 import observe_local_endpoint
from benchmark.drive_model import DriveModel
H=Path(__file__).resolve().parent;R=H/'endpoint_tail_e1_run_v1';D=H/'local_endpoint_event_causal_v2';D.mkdir(exist_ok=False);P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
files=[H/'local_endpoint_event_v1.py',H/'test_local_endpoint_event_v1.py',R/'native_predictions.json',R/'policy_records.json',R/'requests.npz',R/'task_episode.json',R/'camera_clocks.json',P/'configs/benchmark_v1/drive_dynamics_v1.json'];(D/'protocol.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),frames=list(range(155,166)),scope='current native capture feedback + last completed tracker state/reference; no new projection, no plan search, no current after-act diagnostics',tests=['future-native-feedback and policy-row replacement','future truncation','input bytes unchanged'],offline_not_online=True,no_actor_consumer=True,no_fresh_or_model_acceptance=True,input_sha256={str(p):sha(p) for p in files}),indent=2)+'\n')
rows=json.loads((R/'policy_records.json').read_text());native=json.loads((R/'native_predictions.json').read_text());clocks=json.loads((R/'camera_clocks.json').read_text());goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2];drive=DriveModel.from_json(P/'configs/benchmark_v1/drive_dynamics_v1.json')
def run(rs,ns,last):
 result=[]
 for i in range(155,last+1):
  n=ns[i*5];prev=rs[i-1];state=prev['tracker_state_after_act'];t=float(clocks[i]['episode_time_s']);assert clocks[i]['frame']==i
  assert n['available'] and abs(n['time_s']-t)<1e-8 and prev['time_s']<t
  snapshot=dict(time_s=t,feedback_time_s=n['time_s'],position_xy=n['position_world'][:2],velocity_xy=n['velocity_world_m_s'][:2],reference_xy=prev['installed_reference_xy_after_act'],reference_version=prev['local_reference_version'],path_index=state['path_index'],active_direction=state['active_direction'],pending_direction=state['pending_direction'],braking_state=state['braking_state'],global_goal_xy=goal)
  result.append(observe_local_endpoint(snapshot,drive.stopping_distance))
 return result
before={str(p):sha(p) for p in files};full=run(rows,native,165);past=run(rows[:165],native[:821],164);poison_rows=copy.deepcopy(rows);poison_native=copy.deepcopy(native)
for i in range(164,len(poison_rows)):poison_rows[i]={'future_poison':True}
for i in range(821,len(poison_native)):poison_native[i]={'future_poison':True}
poison=run(poison_rows,poison_native,164);assert full[:-1]==past==poison;assert before=={str(p):sha(p) for p in files};event=full[9];assert event['local_stop_condition'] and not event['global_goal_reached'] and event['input_path_index']==6 and event['reference_version']==1
result=dict(rows=full,future_replacement_and_truncation_prefix_exact=True,raw_inputs_unchanged=True,requests_unchanged_sha256=sha(R/'requests.npz'),tests_passed=3,observed_event_at_164=True,offline_reader_only_not_online_shadow=True,shadow_on_off_B0_test_not_run=True,no_new_actor_or_candidate=True,whole_mission_complete=False);(D/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(D/'artifact_manifest.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in D.iterdir() if p.is_file()}),indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
