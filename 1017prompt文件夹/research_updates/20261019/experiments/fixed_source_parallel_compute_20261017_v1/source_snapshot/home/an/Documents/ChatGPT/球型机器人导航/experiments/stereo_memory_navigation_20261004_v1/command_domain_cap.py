"""One final main-request cap, not a physical prediction or observed-free claim."""
import math
import numpy as np

def with_command_domain_cap(base):
 class CommandDomainCap(base):
  def act(self,observation,goal):
   action=np.asarray(super().act(observation,goal),np.float32).copy();row=self.records[-1];nominal=np.asarray(row['uncapped_request'],np.float32);unknown=float(row['unknown_fraction'])
   if action.shape!=(10,) or nominal.shape!=(10,) or not np.isfinite(action).all() or not np.isfinite(nominal).all() or not math.isfinite(unknown):raise ValueError('current finite five-block request required')
   old=action.copy()
   if unknown>0:action[::2]=np.where(nominal[::2]>0,np.minimum(nominal[::2],.5),old[::2])
   assert np.array_equal(action[1::2],old[1::2])
   row.update(old_task_cap_request=old.tolist(),request=action.tolist(),unknown_main_cap_rad_s=.5,unknown_reverse_main_cap_rad_s=.3,task_cap_candidate='single_command_domain_cap',tracking_fields_scope='tracker_before_task_cap',prior_or_unknown_not_observed_free=True)
   return action
 return CommandDomainCap
