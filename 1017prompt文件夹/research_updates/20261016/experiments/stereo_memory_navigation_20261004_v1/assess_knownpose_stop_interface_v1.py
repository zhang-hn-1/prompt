"""Current/past snapshot reconstruction of already online calls. No physics replay or score rescue."""
from pathlib import Path
import json,hashlib,sys,collections
import numpy as np
from local_endpoint_event_v2 import observe_local_endpoint
from outgoing_lookahead import make_endpoint_oracle
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent
D=H/'fixed_reference_pacing_knownpose_paced_run_v3'

def replay(i,states,rows,goal,stop):
 previous=rows[i-1];old=previous['tracker_state_after_act'];history=np.load(D/'history'/f'frame_{i:03d}.npz')
 return observe_local_endpoint(dict(time_s=states[i]['time_s'],feedback_time_s=states[i]['time_s'],position_xy=np.asarray(states[i]['position'][:2]),velocity_xy=np.asarray(history['proprio_history'][-1,:2],float),reference_xy=np.asarray(previous['installed_reference_xy_after_act']),reference_version=previous['local_reference_version'],path_index=old['path_index'],active_direction=old['active_direction'],pending_direction=old['pending_direction'],braking_state=old['braking_state'],global_goal_xy=np.asarray(goal[:2])),stop)

if __name__=='__main__':
 states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());goal=json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'];protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));stop=make_endpoint_oracle(protocol).drive_model.stopping_distance;counter=collections.Counter();checked=[]
 for i,r in enumerate(rows):
  live=r.get('local_endpoint_live_before_act')
  if not live or not live.get('valid'):continue
  value=replay(i,states,rows,goal,stop)
  assert all(value[k]==live[k] for k in value),(i,value,live)
  # Replace future with unusable payloads, and independently truncate; current/past remains identical.
  assert replay(i,states[:i+1],rows[:i],goal,stop)==value
  assert replay(i,states[:i+1]+[None]*(len(states)-i-1),rows[:i]+[None]*(len(rows)-i),goal,stop)==value
  counter[value['event']]+=1;checked.append(i)
 out=H/'knownpose_stop_interface_assessment_v1.json';out.write_text(json.dumps(dict(valid_live_calls_checked=len(checked),events=dict(counter),current_past_reconstruction_exact=True,future_replaced_and_truncated_equal=True,actually_online_stop_branch_exercised=counter['local_endpoint_stop_with_distant_global_goal']>0,scope='known current pose run only; branch arithmetic/interface, not zero-plan future physical prediction or stopping-bound certification',actual_policy_requests_not_relabelled_as_another_future_plan=True,model_prediction_accept=None,control_candidate_remains_rejected=True,formal_navigation_not_run=True,whole_mission_complete=False,input_sha256={str(D/n):hashlib.sha256((D/n).read_bytes()).hexdigest() for n in ['policy_records.json','states.json','task_episode.json']}),indent=2)+'\n');print(out.read_text())
