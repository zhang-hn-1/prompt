"""Independent same-source history consumer; its commands are diagnostic only."""
import copy
import numpy as np
from prior_rgb_navigation import PriorRGBPolicy

class ReadOnlyMemoryShadow:
 def __init__(self,protocol):self.policy=PriorRGBPolicy(protocol,'history');self.records=[]
 def reset(self,context):self.policy.reset(copy.deepcopy(context));self.records=[]
 def consume(self,source,packet,observation,goal):
  t=float(packet['time_s']);st=float(source['time_s'])
  if not np.isfinite(t) or not np.isfinite(st) or st>t:raise ValueError('current or past source required before payload')
  self.policy.set_source_packet(copy.deepcopy(source));self.policy.set_current_packet(copy.deepcopy(packet));action=self.policy.act(copy.deepcopy(observation),copy.deepcopy(goal))
  row=dict(frame=int(packet['frame']),time_s=t,source_frame=int(source['frame']),source_time_s=st,request=action.tolist(),local=copy.deepcopy(self.policy.records[-1]),read_only=True,executed=False,alternative_future_truth_available=False);self.records.append(row);return row
