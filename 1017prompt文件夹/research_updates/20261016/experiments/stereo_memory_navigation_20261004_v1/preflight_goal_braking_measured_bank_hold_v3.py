from pathlib import Path
import copy,json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
from goal_braking_measured_bank_hold import GoalBrakingMeasuredBankHold
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'goal_braking_measured_bank_hold_preflight_v3';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();rows=json.loads((D/'policy_records.json').read_text());es=json.loads((D/'execution_states.json').read_text());phy=dict(np.load(D/'physics.npz'))
def select(records):
 for j,r in enumerate(records):
  if r.get('local',{}).get('tracker_state_after_act',{}).get('braking_state')=='goal_braking' and np.asarray(r['request']).reshape(5,2)[0,0]==0:return j
 raise ValueError('no first event')
i=select(rows);posepath=D/'estimated_poses'/f'frame_{i:03d}.npz';pose=dict(np.load(posepath));assert rows[i]['planner_act_called'] and rows[i]['local']['reference_snapshot_valid'];now=float(rows[i]['time_s']);assert now==float(phy['simulation_time_s'][i*5]);proto=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),first_current_event_frame=i,maximum_ticks=5,current_known_request_block_only=True,no_future_episode_end_read=True,source_sha256={str(H/n):sha(H/n) for n in ['goal_braking_measured_bank_hold.py','preflight_goal_braking_measured_bank_hold_v3.py']},input_sha256={str(D/n):sha(D/n) for n in ['policy_records.json','execution_states.json','physics.npz']}|{str(posepath):sha(posepath)},old_V2_pending_freshness_and_future_gate_not_admitted=True);(O/'protocol.json').write_text(json.dumps(proto,indent=2)+'\n')
def run(records,snapshots,feedback,measurement):
 frame=select(records);r=records[frame];now=float(r['time_s']);parent=np.asarray(r['request'],np.float32).reshape(5,2);assert np.all(parent[:,0]==0);C=GoalBrakingMeasuredBankHold();base=ExecutionClone.from_snapshot(snapshots[frame]);cand=ExecutionClone.from_snapshot(snapshots[frame]);quat=Rotation.from_matrix(measurement['root_rotation_world']).as_quat();bank=bank_from_quaternion(quat[[3,0,1,2]]);candidate=C.request(parent[0],r['local']['tracker_state_after_act']['braking_state'],bank,float(measurement['time_s']),now);trace=[]
 for slot in range(5):
  tick=frame*5+slot;kwargs=dict(bank_rad=bank_from_quaternion(feedback['root_quaternion_world_wxyz'][tick]),aux_position_rad=feedback['joint_position_rad'][tick,1],aux_velocity_rad_s=feedback['joint_velocity_rad_s'][tick,1]);ba,bs=base.step(parent[slot],**kwargs);np.testing.assert_array_equal(ba,feedback['applied_action'][tick+1]);np.testing.assert_array_equal(bs,feedback['smoothed_target'][tick+1]);ca,cs=cand.step(candidate,**kwargs);np.testing.assert_array_equal(ca[0],ba[0]);different=not np.array_equal(ca,ba);trace.append(dict(frame=frame,tick=tick,current_bank_from_native_rad=float(bank),measurement_time_s=float(measurement['time_s']),independent_now_s=now,parent_request=parent[slot].tolist(),candidate_request=candidate.tolist(),base_applied=ba.tolist(),candidate_applied=ca.tolist(),applied_different=different))
  if different:break
 return trace
trace=run(rows,es,phy,pose);assert trace[-1]['applied_different'];last=trace[-1]['tick'];short=run(rows[:i+1],es[:i+1],{k:v[:last+2] for k,v in phy.items()},pose);assert short==trace
poisonrows=copy.deepcopy(rows[:i+1])+[None]*(len(rows)-i-1);poisonsnap=copy.deepcopy(es[:i+1])+[None]*(len(es)-i-1);poisonphys={}
for k,v in phy.items():
 v=v.copy()
 if np.issubdtype(v.dtype,np.floating):v[last+2:]=np.nan
 else:v[last+2:]=999
 poisonphys[k]=v
assert run(poisonrows,poisonsnap,poisonphys,pose)==trace
for offset in [-.3,.01]:
 bad=copy.deepcopy(pose);bad['time_s']=now+offset;badtrace=run(rows[:i+1],es[:i+1],phy,bad);assert not any(v['applied_different'] for v in badtrace),offset
(O/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_CURRENT_BLOCK_FIRST_EFFECT_AND_CAUSAL_PREFIX_INTERFACE_ONLY',first_frame=i,first_applied_difference_tick=last,first_applied_difference_s=float(phy['simulation_time_s'][last+1]),prefix_truncate_exact=True,future_payload_poison_exact=True,independent_current_clock_pose_stale_and_future_rejected=True,current_pose_SHA_sealed=True,current_bank_actor_native_only=True,main_applied_preserved=True,no_feedback_after_first_divergence_read=True,no_future_episode_or_request_read=True,no_physics_prediction_or_NAV_or_AB_admission=True,needs_actual_source_reference_reset_prefix_contract_and_review_resolution=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
