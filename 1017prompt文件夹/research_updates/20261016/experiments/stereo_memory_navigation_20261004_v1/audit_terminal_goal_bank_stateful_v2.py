from pathlib import Path
import json,hashlib,datetime,numpy as np
from terminal_goal_bank_source import TerminalGoalBankSource
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'terminal_goal_bank_stateful_audit_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();trace=json.loads((H/'native_terminal_factual_chain_v2/trace.json').read_text());latch=TerminalGoalBankSource();selected=None
for row in trace:
 parent=np.asarray(row['request'],np.float32);final,info=latch.apply(parent,row['request_diagnostics']['request_source'])
 if not np.array_equal(final,parent):selected=(row,parent,final);break
assert selected is not None;row,parent,final=selected;i=row['frame'];assert i==555
es=json.loads((D/'execution_states.json').read_text());ph=np.load(D/'physics.npz');a=ExecutionClone.from_snapshot(es[i]);b=ExecutionClone.from_snapshot(es[i]);ticks=[];divergence=None
for k in range(5):
 idx=5*i+k
 # Feedback idx is a conditional common state only if all PRIOR commands appliedidentical.
 assert divergence is None and all(not t['applied_different'] for t in ticks)
 kwargs=dict(bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][idx]),aux_position_rad=ph['joint_position_rad'][idx,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][idx,1]);aa,ss=a.step(parent.reshape(5,2)[k],**kwargs);bb,tt=b.step(final.reshape(5,2)[k],**kwargs)
 np.testing.assert_array_equal(aa,ph['applied_action'][idx+1]);np.testing.assert_array_equal(ss,ph['smoothed_target'][idx+1]);np.testing.assert_array_equal(aa[0],bb[0]);different=not np.array_equal(aa,bb);ticks.append(dict(physics_feedback_index=idx,applied_index=idx+1,parent_applied=aa.tolist(),candidate_applied=bb.tolist(),applied_different=different,execution_state_identical=a.snapshot()==b.snapshot(),feedback_common_prefix_conditional=True))
 if different:divergence=idx+1;break
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_EXISTING_CANDIDATE_STATEFUL_EXECUTION_EFFECT_ONLY' if divergence is not None else 'NO_EFFECT_WITHIN_CURRENT_FIVE_TICKS_ONLY',event_frame=i,ticks=ticks,first_applied_divergence_index=divergence,feedback_last_index=ticks[-1]['physics_feedback_index'],no_feedback_after_divergence=True,no_counterfactual_trajectory_or_goal_labels=True,original_probe_v1_preserved=True,not_physical_response_prediction=True,not_new_navigation=True,not_fresh_heldout=True,input_sha256={str(f):sha(f) for f in [H/'native_terminal_factual_chain_v2/trace.json',D/'execution_states.json',D/'physics.npz',H/'terminal_goal_bank_source.py',H/'terminal_goal_bank_source_probe_v1/assessment.json',H.parent.parent/'docs/superpowers/plans/2026-10-08-terminal-goal-bank-stateful-execution-audit.md']},code_sha256=sha(__file__),whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
