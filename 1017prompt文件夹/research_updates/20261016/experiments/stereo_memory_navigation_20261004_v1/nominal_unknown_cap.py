"""One final-request cap change; inherited controller/reference executes once."""
import numpy as np
from prior_rgb_navigation import PriorRGBPolicy
from same_stream_memory_shadow import ReadOnlyMemoryShadow

def nominal_unknown_action(action,unknown_fraction):
 a=np.asarray(action,np.float32);u=float(unknown_fraction)
 if a.shape!=(10,) or not np.isfinite(a).all() or not np.isfinite(u) or not 0<=u<=1:raise ValueError('finite10D request and unknown fraction in[0,1] required')
 out=a.copy()
 if u>0:out[::2]=np.clip(out[::2],-1.,1.)
 return out

def with_nominal_unknown_cap(base_class):
 class NominalUnknownCap(base_class):
  policy_name='experimental_rgb_knownpose_history_nominal_unknown_cap'
  def act(self,observation,goal):
   super().act(observation,goal)
   row=self.records[-1];action=nominal_unknown_action(row['uncapped_request'],row['unknown_fraction']);row.update(request=action.tolist(),unknown_main_cap_rad_s=1.,cap_layer='post_inherited_tracking_request');return action
 return NominalUnknownCap

class ReadOnlyHistorySlowShadow(ReadOnlyMemoryShadow):
 def consume(self,source,packet,observation,goal):
  row=super().consume(source,packet,observation,goal);row.update(source_filter='dynamic_current_joint_CAD',memory_role='history10s_same_as_primary',unknown_main_cap_rad_s=.3);return row
