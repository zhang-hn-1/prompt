"""One geometry-triggered terminal main-axis cap, on the frozen nominal parent."""
import numpy as np
from nominal_unknown_cap import with_nominal_unknown_cap

def terminal_action(action,distance_m,latched):
 a=np.asarray(action,np.float32);d=float(distance_m)
 if a.shape!=(10,) or not np.isfinite(a).all() or not np.isfinite(d) or d<0:raise ValueError('finite10D action and nonnegative current goal distance required')
 on=bool(latched) or d<=.60;out=a.copy()
 if on:out[::2]=np.clip(out[::2],-.3,.3)
 return out,on

def with_terminal_goal_consumer(base_class):
 parent=with_nominal_unknown_cap(base_class)
 class TerminalGoalConsumer(parent):
  policy_name='experimental_rgb_knownpose_history_terminal_goal_consumer'
  def __init__(self,*args,**kwargs):self.terminal_mode=False;super().__init__(*args,**kwargs)
  def reset(self,context):super().reset(context);self.terminal_mode=False
  def act(self,observation,goal):
   # Runtime pointgoal_base_xy is the declared current relative goal. No future state.
   g=np.asarray(goal,float)
   if g.shape!=(2,) or not np.isfinite(g).all():raise ValueError('finite current 2D pointgoal required')
   pre=super().act(observation,goal);d=float(np.linalg.norm(g));out,self.terminal_mode=terminal_action(pre,d,self.terminal_mode)
   self.records[-1].update(pre_terminal_request=pre.tolist(),terminal_mode=self.terminal_mode,terminal_goal_distance_m=d,terminal_entry_distance_m=.60,terminal_main_cap_rad_s=.3,request=out.tolist())
   return out
 return TerminalGoalConsumer
