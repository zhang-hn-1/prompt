import dataclasses,math
import numpy as np,pytest
from terminal_node_admission import goal_approach_allowed,plan_kinodynamic_path
import supported_four_point_planning as old
from benchmark.drive_model import DriveModel
from benchmark.policies import DEFAULT_DRIVE_PROFILE

def test_predicate_rejects_goal_behind_sideways_and_reverse():
 assert goal_approach_allowed(0,0,0,"forward",(1,0))
 assert goal_approach_allowed(1,0,0,"forward",(1,0))
 assert not goal_approach_allowed(0,0,math.pi,"forward",(1,0))
 assert not goal_approach_allowed(0,0,math.pi/2,"forward",(1,0))
 assert not goal_approach_allowed(0,0,0,"reverse",(1,0))
 assert not goal_approach_allowed(0,0,0,"forward",(.1,.1))

def test_nonfinite_not_admitted():
 with pytest.raises(ValueError):goal_approach_allowed(np.nan,0,0,"forward",(1,0))

@pytest.mark.parametrize("goal",[(1,.5),(-1,0),(0,0)])
def test_disabled_hook_parent_exact_and_original_request_identity(goal):
 req=old.KinodynamicPlanRequest(np.zeros((80,80),np.uint8),.1,(-4,-4),(0,0),0,goal,node_budget=3000)
 model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
 assert dataclasses.asdict(plan_kinodynamic_path(req,model))==dataclasses.asdict(old.plan_kinodynamic_path(req,model))

def test_hook_is_consumed_including_at_start_and_respects_budget():
 req=old.KinodynamicPlanRequest(np.zeros((80,80),np.uint8),.1,(-4,-4),(0,0),0,(0,0),node_budget=3)
 model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE);seen=[]
 def deny(x,y,yaw,direction):seen.append((x,y,yaw,direction));return False
 result=plan_kinodynamic_path(req,model,goal_acceptance=deny)
 assert seen and result.status!="success" and result.expanded_nodes<=3


def test_rotated_coordinates_asymmetric_bounds_and_zero_reverse_exception():
 yaw=.7;c,s=math.cos(yaw),math.sin(yaw)
 for k,allowed in [(-.827224,True),(-.827226,False),(.796602,True),(.796604,False)]:
  y=k/(1+math.sqrt(1-k*k))
  assert goal_approach_allowed(0,0,yaw,"forward",(c-s*y,s+c*y))==allowed
 assert goal_approach_allowed(1,2,.7,"reverse",(1,2))
