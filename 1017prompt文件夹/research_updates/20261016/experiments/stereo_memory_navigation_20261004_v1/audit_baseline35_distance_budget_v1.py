"""One measured-path budget audit. Observed path is not a maximum achievable speed bound."""
from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent;D=H/'baseline35_distance_budget_audit_v1';D.mkdir(exist_ok=False);out=[];inputs={}
for condition in ('e0','e1','e2'):
 B=H/(f'native_pose_baseline35_{condition}_run_v1' if condition=='e0' else f'native_pose_baseline35_{condition}_coverage_run_v1')
 if not B.exists():
  candidates=list(H.glob(f'native_pose_baseline35*{condition}*run_v1'));assert len(candidates)==1,candidates;B=candidates[0]
 ep=json.loads((B/'task_episode.json').read_text())['actual_episode'];ss=json.loads((B/'states.json').read_text());cc=json.loads((B/'physics_contacts.json').read_text());hit=[x['simulation_time_s'] for x in cc if x['available'] and x['contact']];cut=min(35.,min(hit) if hit else 35.);states=[s for s in ss if s['time_s']<=cut+1e-5];xy=np.array([s['position'][:2] for s in states]);start=np.array(ep['start_position_world_m'][:2]);goal=np.array(ep['goal_position_world_m'][:2]);distance=float(np.linalg.norm(goal-start));path=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum());axis=(goal-start)/distance;progress=float((xy[-1]-start)@axis)
 out.append(dict(condition=condition,scored_cut_s=cut,goal_initial_distance_m=distance,straight_distance_necessary_for_original_goal_radius_m=distance-.2,minimum_direct_average_speed_over_full35_m_s=(distance-.2)/35.,minimum_direct_average_speed_after_frozen6settling_m_s=(distance-.2)/29.,actual_path_to_scored_cut_m=path,actual_axis_progress_m=progress,actual_axis_progress_per_path=progress/path,actual_path_shortfall_vs_necessary_straight_m=max(0.,distance-.2-path),actual_path_is_not_maximum_achievable_bound=True,collision_censors_e2=bool(hit)))
 for f in ('states.json','task_episode.json','physics_contacts.json'):
  p=B/f;inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
r=dict(question='Does measured amount of travel alone suffice for direct original arrival, and how much was net goal progress?',rows=out,no_new_physics=True,no_model_fit_or_speed_change=True,no_claim_task_impossible=True,no_candidate_admission=True,no_relaxation_of35=True)
(D/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(D/'inputs.json').write_text(json.dumps(inputs,indent=2)+'\n');print(json.dumps(r,indent=2))
