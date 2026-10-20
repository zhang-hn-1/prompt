"""One current-joint CAD endpoint filter; oldmask shadow never executes."""
import numpy as np
from dense_source_points import point_packet
from dynamic_self_mask import joints_from_proprio
from prior_rgb_navigation import PriorRGBPolicy
from same_stream_memory_shadow import ReadOnlyMemoryShadow

def paired_sources(prediction,known_pose,proprio_history,cad,current_time_s):
 t=float(prediction['time_s']);now=float(current_time_s)
 if not np.isfinite(t) or not np.isfinite(now) or t!=now:raise ValueError('current source time required before payload')
 if set(prediction)!={'depth_m','confidence','disparity_native_px','frame','time_s'}:raise ValueError('prediction input whitelist')
 q=joints_from_proprio(proprio_history);m=cad.project(q)
 old=point_packet(prediction,known_pose,cad.frozen['left'],cad.frozen['right'])
 new=point_packet(prediction,known_pose,m['left'],m['right'])
 return old,new,m

class ReadOnlyMaskShadow(ReadOnlyMemoryShadow):
 def __init__(self,protocol):self.policy=PriorRGBPolicy(protocol,'current');self.records=[]
 def consume(self,source,packet,observation,goal):
  row=super().consume(source,packet,observation,goal);row['source_filter']='frozen_base_only';return row
