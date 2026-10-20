"""Missing normalized time term must fail literals and actual selected command."""
from pathlib import Path
import os,sys,importlib,json,math
import numpy as np
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark.drive_model import DriveModel
from direction_root_planning import KinodynamicPlanRequest,plan_kinodynamic_path as original_plan
m=importlib.import_module(os.environ.get('TIME_COST_MODULE','nominal_time_cost'))
planner=importlib.import_module(os.environ.get('TIME_PLANNER_MODULE','time_cost_planning'))

@pytest.mark.parametrize('length,speed,maximum,want',[(.3,.2,.6,.9),(.3,.6,.6,.3),(.3,0.,.6,3.6)])
def test_normalized_time_literals(length,speed,maximum,want):
 assert m.nominal_edge_cost(length,speed,maximum)==pytest.approx(want,abs=1e-12)

@pytest.mark.parametrize('args',[(-.3,.2,.6),(.3,-.1,.6),(.3,.2,0.),(.3,float('nan'),.6),(.3,.2,float('inf')),(.3,.8,.6)])
def test_invalid_nominal_proxy_inputs_rejected(args):
 with pytest.raises(ValueError):m.nominal_edge_cost(*args)

def synthetic():
 payload=json.loads((P/'configs/benchmark_v1/drive_dynamics_v1.json').read_text())
 for r in payload['steering_samples']:
  r['curvature_1_m']=-r['auxiliary_angle_rad'];r['speed_m_s']={.5:.4,1.:.2,1.5:.1}[abs(r['main_command_rad_s'])]
 return DriveModel.from_mapping(payload)

def test_actual_planner_selects_faster_nominal_main_on_same_two_straight_edges():
 req=KinodynamicPlanRequest(np.zeros((21,21),np.uint8),.1,(-1.,-1.),(0.,0.),0.,(.6,0.),node_budget=100,goal_tolerance_m=.01)
 model=synthetic();old=original_plan(req,model,initial_direction='forward');new=planner.plan_kinodynamic_path(req,model,initial_direction='forward')
 assert old.status==new.status=='success'
 assert [w.main_command_rad_s for w in old.waypoints[:-1]]==[1.5,1.5]
 assert [w.main_command_rad_s for w in new.waypoints[:-1]]==[.5,.5]
 np.testing.assert_allclose([[w.x_m,w.y_m] for w in new.waypoints],[[0,0],[.3,0],[.6,0]],atol=1e-12,rtol=0)
 assert new.direction_changes==0 and new.path_length_m==pytest.approx(.6)
