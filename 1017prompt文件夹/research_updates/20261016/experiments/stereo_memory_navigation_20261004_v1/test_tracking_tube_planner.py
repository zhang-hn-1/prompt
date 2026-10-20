from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import tracking_tube_planner as module
from route_geometry_audit import hazard_clearance
H=Path(__file__).resolve().parent
D=H/'bridge_consumer_C2_run_v1'
def grid():
 m=np.load(D/'maps/frame_060.npz');g=module.TrackingTubeGrid(m['origin'],retention_s=60);g.labels=m['labels'].copy();return g

def test_same_map_route_strengthens_clearance_without_claiming_observation():
 g=grid();before=g.labels.copy();path,status=g.plan([-2.2,0],[2.2,0]);assert path is not None
 np.testing.assert_equal(g.labels,before);assert hazard_clearance(path,g.origin+np.argwhere(before==1)*g.resolution)>=.8
 assert not np.isfinite(g.memory.floor_time).any() and not np.isfinite(g.memory.hazard_time).any();assert g.exploration_unknown_fraction==1.

def test_same_direction_nominal_reference_covers_empirical_tube():
 from training.action_protocol import load_action_protocol
 from oracle_rgb_shadow import make_oracle
 p=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');tr=make_oracle(load_action_protocol(p/'configs/data_contract.yaml'));g=grid();plan,meta=module.local_curve_for_direction_tube(g,[-2.2,0],0,[2.2,0],tr.drive_model,initial_direction=None)
 assert plan is not None and meta['initial_direction'] is None and meta['planning_clearance_m']==pytest.approx(.8707106781186548)
 xy=np.array([[w.x_m,w.y_m] for w in plan.waypoints]);assert hazard_clearance(xy,g.origin+np.argwhere(g.labels==1)*g.resolution)>=.8
 assert not meta['certified_safe'] and not meta['empirical_tracking_error_certified']

def test_larger_margin_does_not_allow_original_raster_failed_start():
 g=grid();path,status=g.plan([-1.298,-.474],[2.2,0]);assert path is None and status=='observed_obstacle_blocks_route'

def test_original_boundary_guard_restored_no_alias_or_direction_reset(monkeypatch):
 from prior_rgb_navigation import PriorRGBPolicy
 cls=module.with_tracking_tube(PriorRGBPolicy);o=object.__new__(cls);g=grid();m=np.load(D/'maps/frame_200.npz');g.labels=m['labels'];o.grid=g;o.plan=object();o.plan_meta={};o.local_reference_version=1;o.reference_installed_time=6.;lp=np.load(D/'local_plans/frame_199.npz');o.tracker=SimpleNamespace(_path_index=5,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=0,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[],_reference_xy=lp['pose'][:,:2],_context=object(),drive_model=object());calls=[]
 def fake(g,xy,yaw,goal,drive_model,*,initial_direction):calls.append(initial_direction);return None,{'status':'observed_obstacle_blocks_route'}
 monkeypatch.setattr(module,'local_curve_for_direction_tube',fake);inp=np.load(D/'policy_inputs/frame_200.npz');created,dec=o.plan_boundary(inp['position_world_xy'],float(inp['yaw_world_rad']),[2.2,0],float(m['time_s']))
 assert created and dec['decision']=='rebuild' and not dec['guard']['retention_necessary_screen_pass'];assert calls==['forward'];assert o.tracker._active_drive_direction=='forward' and o.tracker._stopped_cycles==0
