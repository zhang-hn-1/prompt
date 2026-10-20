"""One archived requested-direction travel decomposition; requests != actual direction."""
from pathlib import Path
import numpy as np,json,hashlib
H=Path(__file__).resolve().parent;D=H/'baseline35_requested_direction_budget_audit_v1';D.mkdir(exist_ok=False);result=[];inputs={}
for condition in ('e0','e1','e2'):
 B=H/(f'native_pose_baseline35_{condition}_run_v1' if condition=='e0' else f'native_pose_baseline35_{condition}_coverage_run_v1');ep=json.loads((B/'task_episode.json').read_text())['actual_episode'];s=json.loads((B/'states.json').read_text());q=np.load(B/'requests.npz')['requests'];contacts=json.loads((B/'physics_contacts.json').read_text());hits=[r['simulation_time_s'] for r in contacts if r['available'] and r['contact']];cut=min(35.,min(hits) if hits else 35.);start=np.array(ep['start_position_world_m'][:2]);goal=np.array(ep['goal_position_world_m'][:2]);axis=(goal-start)/np.linalg.norm(goal-start);bins={k:dict(intervals=0,time_s=0.,true_path_m=0.,true_axis_progress_m=0.) for k in ('request_main_zero','request_main_positive','request_main_negative')}
 for i,request in enumerate(q):
  if i+1>=len(s) or s[i+1]['time_s']>cut+1e-5:continue
  main=float(request[0]);key='request_main_zero' if main==0. else 'request_main_positive' if main>0. else 'request_main_negative';row=bins[key];delta=np.array(s[i+1]['position'][:2])-np.array(s[i]['position'][:2]);row['intervals']+=1;row['time_s']+=s[i+1]['time_s']-s[i]['time_s'];row['true_path_m']+=float(np.linalg.norm(delta));row['true_axis_progress_m']+=float(delta@axis)
 result.append(dict(condition=condition,cut_s=cut,bins=bins,classification_is_issued_request_not_physical_direction=True,zero_request_does_not_mean_zero_motion=True,contact_last_fraction_not_extrapolated=True))
 for f in ('states.json','requests.npz','physics_contacts.json','task_episode.json'):
  p=B/f;inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(D/'assessment.json').write_text(json.dumps(dict(rows=result,no_new_physics=True,no_model_or_control_change=True,no_causal_sole_failure_attribution=True),indent=2)+'\n');(D/'inputs.json').write_text(json.dumps(inputs,indent=2)+'\n');print(json.dumps(result,indent=2))
