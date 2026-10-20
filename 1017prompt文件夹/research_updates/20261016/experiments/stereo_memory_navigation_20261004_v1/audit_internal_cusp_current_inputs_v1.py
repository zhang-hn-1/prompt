"""Current archived installed-reference queries only, not online or alternate trajectories."""
from pathlib import Path
from types import SimpleNamespace as NS
import json,hashlib,numpy as np
from internal_cusp_reserve_v1 import cusp_context
H=Path(__file__).resolve().parent;D=H/'forward_intent_navigation_e1_candidate_run_v1';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());latest=None;observed=[];inputs={}
 for x in rows:
  f=x['frame'];p=D/'local_plans'/f'frame_{f:03d}.json'
  if p.exists():latest=json.loads(p.read_text());inputs[str(p)]=sha(p)
  if not x['planner_act_called'] or not x['measured_speed_control_applied'] or x['plan']['status']!='success':continue
  assert latest is not None and latest['time_s']<=x['time_s']+1e-7
  state=x['tracker_state_after_act'];wps=[NS(**w) for w in latest['waypoints']];xy=np.array([[w.x_m,w.y_m] for w in wps]);assert np.array_equal(xy,np.array(x['installed_reference_xy_after_act']))
  tr=NS(_reference=wps,_reference_xy=xy,_path_index=state['path_index'],_active_drive_direction=state['active_direction'],_pending_drive_direction=state['pending_direction'],_braking_state=state['braking_state']);policy=NS(time=x['time_s'],reference_decision=x['reference_decision'],tracker=tr);c=cusp_context(policy,NS(position_world_xy=np.asarray(states[f]['position'][:2])))
  if c['eligible']:observed.append(dict(frame=f,time_s=x['time_s'],parent_main=x['request'][0],restored_main=.5,remaining_arc_m=c['remaining_arc_m'],actual_XY_speed_m_s=float(np.linalg.norm(states[f]['velocity'][:2])),plan_capture_frame=latest['frame'],same_installed_reference=True))
 for n in ('policy_records.json','states.json','artifact_manifest_post_close.json'):inputs[str(D/n)]=sha(D/n)
 for n in ('internal_cusp_reserve_design_v1.json','internal_cusp_reserve_v1.py',Path(__file__).name):inputs[str(H/n)]=sha(H/n)
 result=dict(eligible_current_requests=observed,count=len(observed),not_online=True,no_future_request_or_state_reads_for_each_query=True,no_counterfactual_motion_labels=True,no_new_physics=True,old_rejected_navigation_preserved=True,input_sha256=inputs);p=H/'internal_cusp_current_inputs_audit_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(count=len(observed),first=observed[:2],last=observed[-2:])))
if __name__=='__main__':main()
