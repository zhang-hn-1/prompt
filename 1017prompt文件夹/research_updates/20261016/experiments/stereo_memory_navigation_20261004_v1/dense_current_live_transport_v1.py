"""One live packet sampling change; sparse frozen-mask shadow unchanged.

Current measurements remain provisional risk evidence. No clearing or free-space
certification is supplied by this transport adapter.
"""
import numpy as np
from dense_source_points import point_packet
from dense_current_source_packet_v1 import dense_point_packet
from dynamic_self_mask import joints_from_proprio

def paired_sources(prediction,known_pose,proprio_history,cad,current_time_s):
 t=float(prediction['time_s']);now=float(current_time_s);pt=float(known_pose['time_s'])
 if not np.isfinite([t,now,pt]).all() or t!=now or pt!=now:
  raise ValueError('current source and pose clocks required before payload/CAD')
 if set(prediction)!={'depth_m','confidence','disparity_native_px','frame','time_s'}:
  raise ValueError('prediction input whitelist')
 q=joints_from_proprio(proprio_history);m=cad.project(q)
 old=point_packet(prediction,known_pose,cad.frozen['left'],cad.frozen['right'])
 new=dense_point_packet(prediction,known_pose,m['left'],m['right'])
 return old,new,m
