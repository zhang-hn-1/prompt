"""Conditional factual-prefix execution test, never labels a divergent trajectory."""
import numpy as np
from terminal_goal_bank_source import TerminalGoalBankSource
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion

def audit(trace, execution_states, physics, bound_ticks=25):
 if bound_ticks!=25:raise ValueError('frozen event coverage bound')
 latch=TerminalGoalBankSource();iterator=iter(trace);selected=None
 for row in iterator:
  parent=np.asarray(row['request'],np.float32);final,info=latch.apply(parent,row['request_diagnostics']['request_source'])
  if not np.array_equal(parent,final):selected=(row,parent,final);break
 if selected is None:raise ValueError('no intervention event')
 row,parent,final=selected;first=row['frame'];a=ExecutionClone.from_snapshot(execution_states[first]);b=ExecutionClone.from_snapshot(execution_states[first]);ticks=[];divergence=None;last_frame=first
 for k in range(bound_ticks):
  idx=5*first+k;i=idx//5;j=idx%5
  if i!=last_frame:
   row=next(iterator);assert row['frame']==i
   parent=np.asarray(row['request'],np.float32);final,info=latch.apply(parent,row['request_diagnostics']['request_source']);last_frame=i
  assert divergence is None and all(not t['applied_different'] for t in ticks)
  kwargs=dict(bank_rad=bank_from_quaternion(physics['root_quaternion_world_wxyz'][idx]),aux_position_rad=physics['joint_position_rad'][idx,1],aux_velocity_rad_s=physics['joint_velocity_rad_s'][idx,1]);aa,ss=a.step(parent.reshape(5,2)[j],**kwargs);bb,tt=b.step(final.reshape(5,2)[j],**kwargs)
  np.testing.assert_array_equal(aa,physics['applied_action'][idx+1]);np.testing.assert_array_equal(ss,physics['smoothed_target'][idx+1]);np.testing.assert_array_equal(aa[0],bb[0]);different=not np.array_equal(aa,bb);ticks.append(dict(physics_feedback_index=idx,applied_index=idx+1,frame=i,parent_applied=aa.tolist(),candidate_applied=bb.tolist(),applied_different=different,execution_state_identical=a.snapshot()==b.snapshot(),feedback_common_prefix_conditional=True))
  if different:divergence=idx+1;break
 return dict(event_frame=first,bound_ticks=bound_ticks,ticks=ticks,first_applied_divergence_index=divergence,last_request_frame=last_frame,feedback_last_index=ticks[-1]['physics_feedback_index'],no_feedback_after_divergence=True,no_counterfactual_trajectory_or_goal_labels=True)
