import dataclasses,numpy as np
import minimum_prefix_transition_planning_v1 as m
import supported_four_point_planning as b
from oracle_rgb_shadow import make_oracle
from training.action_protocol import load_action_protocol
from depth_local_curve import P

def run(module,goal,first=None):
 req=module.KinodynamicPlanRequest(np.zeros((61,61),np.uint8),.1,(-3.,-3.),(0.,0.),0.,goal,node_budget=3000)
 drive=make_oracle(load_action_protocol(P/'configs/data_contract.yaml')).drive_model
 kw=dict(initial_direction='forward',primitive_curvature_source=lambda main,aux:0.,primitive_requests=((.5,0.),(-.5,0.)))
 if module is m:kw['first_primitive_direction']=first
 return module.plan_kinodynamic_path(req,drive,**kw)

def test_unconstrained_is_exact_original():
 for goal in ((.9,0.),(-.9,0.)):
  assert dataclasses.asdict(run(m,goal))==dataclasses.asdict(run(b,goal))

def test_two_first_forward_primitives_before_legal_later_reverse():
 p=run(m,(-.9,0.),'forward');assert p.status=='success'
 assert all(w.main_command_rad_s>0 for w in p.waypoints[:2])
 assert any(w.main_command_rad_s<0 for w in p.waypoints[2:])

def test_two_first_reverse_primitives_before_legal_later_forward():
 p=run(m,(.9,0.),'reverse');assert p.status=='success'
 assert all(w.main_command_rad_s<0 for w in p.waypoints[:2])
 assert any(w.main_command_rad_s>0 for w in p.waypoints[2:])

def test_current_coast_measured_domain_only_no_extrapolation():
 from types import SimpleNamespace
 from minimum_prefix_transition_consumer_v1 import measured_first_direction
 # Ensure native frozen roll module is loaded by the formal tracker constructor.
 make_oracle(load_action_protocol(P/'configs/data_contract.yaml'))
 o=SimpleNamespace(proprio_history=np.zeros((1,17)),yaw_world_rad=0.,quaternion_world_wxyz_history=np.array([[1.,0,0,0]]))
 o.proprio_history[-1,0]=.165;o.proprio_history[-1,12]=.445
 state=dict(active_direction='forward',pending_direction=None,braking_state='tracking')
 assert measured_first_direction(o,state)=='forward'
 o.proprio_history[-1,0]=.25;assert measured_first_direction(o,state) is None
 o.proprio_history[-1,0]=.165;o.proprio_history[-1,12]=.5;assert measured_first_direction(o,state) is None
 o.proprio_history[-1,12]=.445;o.quaternion_world_wxyz_history[0]=[np.cos(.5),np.sin(.5),0,0]
 assert measured_first_direction(o,state) is None
