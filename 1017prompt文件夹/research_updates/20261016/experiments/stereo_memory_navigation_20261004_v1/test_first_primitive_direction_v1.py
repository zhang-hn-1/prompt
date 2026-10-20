import dataclasses
import numpy as np
import pytest
import first_primitive_direction_planning_v1 as m
import supported_four_point_planning as b
from oracle_rgb_shadow import make_oracle
from training.action_protocol import load_action_protocol
from depth_local_curve import P

def run(module,goal,first=None):
 req=module.KinodynamicPlanRequest(np.zeros((61,61),np.uint8),.1,(-3.,-3.),(0.,0.),0.,goal,node_budget=3000)
 drive=make_oracle(load_action_protocol(P/'configs/data_contract.yaml')).drive_model
 kwargs=dict(initial_direction='forward',primitive_curvature_source=lambda main,aux:0.,primitive_requests=((.5,0.),(-.5,0.)))
 if module is m:kwargs['first_primitive_direction']=first
 return module.plan_kinodynamic_path(req,drive,**kwargs)

def test_default_matches_original_search_exactly():
 for goal in ((.9,0.),(-.9,0.)):
  assert dataclasses.asdict(run(m,goal))==dataclasses.asdict(run(b,goal))

def test_forward_first_still_allows_later_reverse():
 p=run(m,(-.9,0.),'forward')
 assert p.status=='success'
 assert p.waypoints[0].main_command_rad_s>0
 assert any(w.main_command_rad_s<0 for w in p.waypoints)
 assert p.direction_changes>=1

def test_reverse_first_can_reach_forward_goal_later():
 p=run(m,(.9,0.),'reverse')
 assert p.status=='success'
 assert p.waypoints[0].main_command_rad_s<0
 assert any(w.main_command_rad_s>0 for w in p.waypoints)

@pytest.mark.parametrize('bad',['stopped','FORWARD',1])
def test_invalid_direction_is_rejected(bad):
 with pytest.raises(ValueError,match='first_primitive_direction'):run(m,(.9,0.),bad)

def test_current_motion_domain_excludes_braking_reverse_and_rebound():
 from first_primitive_direction_consumer_v1 import measured_first_direction
 from types import SimpleNamespace
 o=SimpleNamespace(proprio_history=np.zeros((1,17)),yaw_world_rad=0.)
 o.proprio_history[-1,0]=.16;o.proprio_history[-1,12]=.44
 state=dict(active_direction='forward',pending_direction=None,braking_state='tracking')
 assert measured_first_direction(o,state)=='forward'
 for k,v in [('active_direction','reverse'),('pending_direction','reverse'),('braking_state','direction_change')]:
  assert measured_first_direction(o,dict(state,**{k:v})) is None
 o.proprio_history[-1,0]=-.16
 assert measured_first_direction(o,state) is None
 o.proprio_history[-1,0]=.04
 assert measured_first_direction(o,state) is None
 o.proprio_history[-1,0]=.16;o.proprio_history[-1,12]=-.44
 assert measured_first_direction(o,state) is None
