from pathlib import Path
import sys,numpy as np,pytest
H=Path(__file__).resolve().parent
for n in ['oracle_feasible_route_20260929_v9/hybrid','bilateral_temporal_geometry_navigation_20261003_v1','clean_visual_hard0_intervention_20261003_v1','visible_surface_connectivity_20261004_v1']:sys.path.insert(0,str(H.parent/n))

def policy():
 from training.action_protocol import load_action_protocol
 from prior_rgb_navigation import PriorRGBPolicy
 from tracking_tube_planner import with_tracking_tube
 from command_domain_cap import with_command_domain_cap
 return with_command_domain_cap(with_tracking_tube(PriorRGBPolicy))(load_action_protocol('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'),'current')
def packet(i):return dict(frame=i,time_s=i*.1,points_world=np.empty((0,3)),root_xyz=np.array([-2.2,.15,.4]))

def test_real_controller_accepts_frame140_without_prefix_acts_or_reset():
 from observation_clock_adapter import observe_clock_without_act
 from guarded_reference import boundary_state
 p=policy();before=boundary_state(p.tracker);old_tracker=p.tracker
 with pytest.raises(ValueError):p.set_current_packet(packet(140))
 for i in range(140):observe_clock_without_act(p,packet(i),i*.1)
 assert not p.records and p.plan is None and p.last_plan==-np.inf and p.tracker is old_tracker and boundary_state(p.tracker)==before
 p.set_current_packet(packet(140));assert p._packet['frame']==140

def test_future_and_frame_gap_fail_without_advancing_capture_or_tracking_state():
 from observation_clock_adapter import observe_clock_without_act
 from guarded_reference import boundary_state
 p=policy();observe_clock_without_act(p,packet(0),0.);before=boundary_state(p.tracker)
 with pytest.raises(ValueError):observe_clock_without_act(p,{'time_s':1.},.1)
 with pytest.raises(ValueError):observe_clock_without_act(p,packet(2),.2)
 assert p.last_frame==0 and p.time==0. and p._packet is None and not p.records and boundary_state(p.tracker)==before
