from pathlib import Path
import json,hashlib,datetime,numpy as np
from terminal_goal_bank_source import TerminalGoalBankSource
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'terminal_goal_bank_full_event_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();trace=json.loads((H/'native_terminal_factual_chain_v2/trace.json').read_text());latch=TerminalGoalBankSource();requests={};first=None
for row in trace:
 parent=np.asarray(row['request'],np.float32);final,info=latch.apply(parent,row['request_diagnostics']['request_source']);requests[row['frame']]=(parent,final)
 if first is None and not np.array_equal(final,parent):first=row['frame']
assert first==555
es=json.loads((D/'execution_states.json').read_text());ph=np.load(D/'physics.npz');a=ExecutionClone.from_snapshot(es[first]);b=ExecutionClone.from_snapshot(es[first]);ticks=[];divergence=None
for k in range(25):
 idx=5*first+k;i=idx//5;j=idx%5
 assert divergence is None and all(not t['applied_different'] for t in ticks)
 parent,final=requests[i];kwargs=dict(bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][idx]),aux_position_rad=ph['joint_position_rad'][idx,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][idx,1]);aa,ss=a.step(parent.reshape(5,2)[j],**kwargs);bb,tt=b.step(final.reshape(5,2)[j],**kwargs)
 np.testing.assert_array_equal(aa,ph['applied_action'][idx+1]);np.testing.assert_array_equal(ss,ph['smoothed_target'][idx+1]);np.testing.assert_array_equal(aa[0],bb[0]);different=not np.array_equal(aa,bb);ticks.append(dict(physics_feedback_index=idx,applied_index=idx+1,frame=i,parent_applied=aa.tolist(),candidate_applied=bb.tolist(),applied_different=different,execution_state_identical=a.snapshot()==b.snapshot(),feedback_common_prefix_conditional=True))
 if different:divergence=idx+1;break
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_EXISTING_CANDIDATE_EXECUTION_EFFECT_ONLY' if divergence is not None else 'NO_EFFECT_WITHIN_FROZEN_25TICK_EVENT_BOUND_ONLY',event_frame=first,bound_ticks=25,ticks=ticks,first_applied_divergence_index=divergence,no_feedback_after_divergence=True,no_counterfactual_trajectory_or_goal_labels=True,old_five_tick_no_effect_scope_preserved=True,no_threshold_or_structure_change=True,not_physical_prediction_or_navigation=True,input_sha256={str(f):sha(f) for f in [H/'native_terminal_factual_chain_v2/trace.json',D/'execution_states.json',D/'physics.npz',H/'terminal_goal_bank_source.py',H/'terminal_goal_bank_stateful_audit_v2/assessment.json',H.parent.parent/'docs/superpowers/plans/2026-10-08-terminal-bank-full-event-effect.md']},code_sha256=sha(__file__),whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
