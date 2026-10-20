"""One isolated geometric input candidate; no root attitude replacement."""
import numpy as np
from current_motion_heading_feasibility_v1 import derive_motion_heading

def select_forward_geometry(parent,world_delta,*,now,times,velocity,main_velocity,active,pending,braking,reference_direction):
 parent=float(parent)
 if not np.isfinite(parent):raise ValueError('finite parent geometry required')
 meta=dict(used=False,parent_curvature=parent,course_heading_rad=None,source='ineligible_parent_geometry',not_future_heading_prediction=True)
 if active!='forward' or pending is not None or braking!='tracking' or reference_direction!='forward':return parent,meta
 heading=derive_motion_heading(now,times,velocity,main_velocity)
 if heading is None:return parent,meta
 # Same measured positive main domain as active forward; a reverse-coasting
 # velocity measurement cannot substitute reverse-qualified heading here.
 if float(np.asarray(main_velocity)[-1])<=0:return parent,meta
 delta=np.asarray(world_delta,float)
 if delta.shape!=(2,) or not np.isfinite(delta).all():raise ValueError('current finite lookahead delta required')
 distance=float(delta@delta)
 lateral=-np.sin(heading)*delta[0]+np.cos(heading)*delta[1]
 selected=0. if distance<=1e-9 else 2.*float(lateral)/distance
 meta.update(used=True,course_heading_rad=heading,selected_curvature=float(selected),source='qualified_current_forward_motion_course')
 return float(selected),meta
