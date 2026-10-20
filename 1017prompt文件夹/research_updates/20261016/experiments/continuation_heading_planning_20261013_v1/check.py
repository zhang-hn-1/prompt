import bootstrap
import numpy as np
from benchmark.drive_model import DriveModel
from scripts.run_tracking_acceptance import DEFAULT_DRIVE_PROFILE
import continuation_search as c
m=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
r=c.KinodynamicPlanRequest(np.zeros((61,61),'uint8'),.1,(-3.,-3.),(0.,0.),0.,(-.9,0.),node_budget=100)
kw=dict(initial_direction='forward',primitive_curvature_source=lambda a,b:0.,primitive_requests=((.5,0.),(-.5,0.)))
assert c.plan_kinodynamic_path(r,m,terminal_motion_vector=(-1.,0.),**kw).status=='success'
assert c.plan_kinodynamic_path(r,m,terminal_motion_vector=(0.,1.),**kw).status!='success'
p=c.plan_kinodynamic_path(r,m,terminal_motion_vector=(1.,0.),**kw);assert p.status=='success' and p.waypoints[-1].drive_direction=='forward'
# Identical old behavior when there is no outgoing route at the final goal.
import minimum_prefix_transition_planning_v1 as old
from dataclasses import asdict
assert asdict(c.plan_kinodynamic_path(r,m,**kw))==asdict(old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**vars(r)),m,**kw))
print('4 terminal motion tests pass')
