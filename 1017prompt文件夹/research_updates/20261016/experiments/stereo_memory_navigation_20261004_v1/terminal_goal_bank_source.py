"""Only fresh goal_braking branch consumes frozen past-motion bank latch."""
import numpy as np
from online_bank_hold import OnlineBankHold
class TerminalGoalBankSource:
 def __init__(self):self.hold=OnlineBankHold()
 def snapshot(self):return self.hold.snapshot()
 def apply(self,request,source):
  if not isinstance(source,str) or not source:raise ValueError('fresh current request source required')
  parent=np.asarray(request,np.float32).copy()
  if parent.shape!=(10,) or not np.isfinite(parent).all():raise ValueError('finite current five-block request required')
  if np.any(parent[::2]!=0) or source=='goal_braking':final,info=self.hold.apply(parent)
  else:
   self.hold.active=False;final=parent.copy();info=dict(active=False,entered=False,source='unchanged_parent_request',state=self.snapshot())
  np.testing.assert_array_equal(final[::2],parent[::2]);info.update(parent_request_source=source,terminal_site_only=True)
  return final,info

def with_terminal_goal_bank(base):
 class TerminalGoalBank(base):
  def reset(self,context):super().reset(context);self.terminal_goal_bank=TerminalGoalBankSource()
  def act(self,observation,goal):
   parent=np.asarray(super().act(observation,goal),np.float32).copy();source=self.tracker.request_diagnostics['request_source'];before=self.terminal_goal_bank.snapshot();final,info=self.terminal_goal_bank.apply(parent,source)
   self.records[-1].update(parent_final_request=parent.tolist(),request=final.tolist(),terminal_goal_bank=info,terminal_goal_bank_state_before=before,final_request_source='terminal_past_motion_bank' if info['active'] else source)
   return final
 return TerminalGoalBank
