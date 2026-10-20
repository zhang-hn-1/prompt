"""Necessary current nominal two-forward-primitive screen; no transient guarantee."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from minimum_prefix_transition_planning_v1 import sample_motion_primitive,primitive_is_collision_free
from execution_domain_four_point_source_v2 import supported_requests,supported_curvature
from execution_domain_four_point_consumer_v2 import planning_clearance

def current_two_forward_prefix(grid,xy,yaw):
 xy=np.asarray(xy,float);yaw=float(yaw)
 if xy.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(yaw):raise ValueError('finite currentXY/yaw')
 start=np.r_[xy,yaw];occ=(distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution<planning_clearance(grid)-1e-8).astype(np.uint8);commands=[c for c in supported_requests() if c[0]>0];feasible=0
 for a in commands:
  one=sample_motion_primitive(start_pose=start,signed_arc_length_m=.3,curvature_1_m=supported_curvature(*a),sample_spacing_m=.05)
  for b in commands:
   two=sample_motion_primitive(start_pose=one[-1],signed_arc_length_m=.3,curvature_1_m=supported_curvature(*b),sample_spacing_m=.05);path=np.vstack([one,two[1:]])
   feasible+=bool(primitive_is_collision_free(path,occupancy_grid=occ,origin_xy=grid.origin,resolution_m=grid.resolution))
 return dict(**{'pass':bool(feasible)},tested_pairs=len(commands)**2,feasible_pairs=int(feasible),current_nominal_geometry_only=True,physical_or_safety_certificate=False,margin_unchanged=True,transient_response_not_predicted=True)
