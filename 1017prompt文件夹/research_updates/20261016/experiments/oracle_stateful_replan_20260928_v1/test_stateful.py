import dataclasses
import numpy as np
import pytest
from benchmark.drive_model import DriveModel
from benchmark.kinodynamic_planning import KinodynamicPlanRequest as OriginalRequest, plan_kinodynamic_path as original_plan
from stateful_planner import KinodynamicPlanRequest, plan_kinodynamic_path
from pathlib import Path
PROFILE=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')
MODEL=DriveModel.from_json(PROFILE)
def request(**kw):
    return KinodynamicPlanRequest(occupancy_grid=np.zeros((41,41),dtype=np.uint8),resolution_m=.1,origin_xy=(-2.,-2.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(-.9,0.),node_budget=3000,**kw)
def test_original_default_exact_parity():
    r=request();old=OriginalRequest(**{f.name:getattr(r,f.name) for f in dataclasses.fields(OriginalRequest)})
    assert dataclasses.asdict(plan_kinodynamic_path(r,MODEL))==dataclasses.asdict(original_plan(old,MODEL))
def test_moving_start_can_still_reverse_when_goal_behind():
    r=request(initial_direction='forward',direction_change_cost_m=1.)
    result=plan_kinodynamic_path(r,MODEL)
    assert result.status=='success'
    assert result.waypoints[0].drive_direction=='reverse'
    assert result.waypoints[-1].x_m<-.6
def test_initial_direction_invalid():
    with pytest.raises(ValueError):request(initial_direction='sideways')
def test_motion_cost_is_finite_speed_dependent():
    from motion_cost import reversal_cost
    assert reversal_cost(0.,'forward',MODEL)==pytest.approx(.5)
    assert reversal_cost(.4,'forward',MODEL)>reversal_cost(.2,'forward',MODEL)>.5
    for speed in [-1.,float('nan'),float('inf')]:
        with pytest.raises(ValueError):reversal_cost(speed,'forward',MODEL)

def test_original_map_adapter_initializes_with_real_profile():
    from source_map import SourceMapReplanner
    adapter=SourceMapReplanner('0002',MODEL)
    assert adapter.grid.ndim==2 and np.any(adapter.grid==1)
    assert adapter.provenance['node_budget']==5000

def test_initial_switch_penalty_changes_selected_route():
    # Synthetic constant-curvature plant keeps the geometric regression small.
    # Restoring two zero-cost roots would choose the direct reverse route.
    class TightTurningPlant(DriveModel):
        def curvature(self,main,aux):return 8.*aux
    plant=TightTurningPlant(**{f.name:getattr(MODEL,f.name) for f in dataclasses.fields(MODEL) if f.init})
    r=request(initial_direction='forward',direction_change_cost_m=100.)
    result=plan_kinodynamic_path(r,plant)
    assert result.status=='success'
    assert all(w.drive_direction=='forward' for w in result.waypoints)
    free=plan_kinodynamic_path(dataclasses.replace(r,initial_direction=None),plant)
    assert free.status=='success' and free.waypoints[0].drive_direction=='reverse'
