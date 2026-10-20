import numpy as np
import pytest
from source_map import SourceMapReplanner
from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
from test_candidate import h
from test_stateful import MODEL

@pytest.mark.parametrize('case',['0002','0004'])
def test_replan_uses_original_obstacles_and_transforms_both_ways(case):
    captured=[]
    def fake(request,model):
        captured.append(request)
        assert np.count_nonzero(request.occupancy_grid)>0
        assert request.node_budget==5000
        return KinodynamicPlanResult('success',(KinodynamicWaypoint(*request.start_xy,request.start_yaw_rad,'forward',.5,0.),KinodynamicWaypoint(*request.goal_xy,request.start_yaw_rad,'forward',0.,0.)),1,1.,0)
    fn=SourceMapReplanner(case,MODEL,planner=fake)
    observation=h._observation(position=(.2,-.1),yaw=.3)
    goal=np.array([2.,-3.])
    result=fn(observation,goal,active_direction="forward")
    assert captured[0].start_xy==pytest.approx(fn.origin+fn.rotation@observation.position_world_xy)
    assert result.waypoints[0].x_m==pytest.approx(.2)
    assert result.waypoints[0].y_m==pytest.approx(-.1)
    assert result.waypoints[0].yaw_rad==pytest.approx(.3)
    assert (result.waypoints[-1].x_m,result.waypoints[-1].y_m)==pytest.approx(goal)

@pytest.mark.parametrize('yaw,velocity,latch,expected',[(0.,(.049,0.),'reverse','reverse'),(0.,(.05,0.),'forward','forward'),(np.pi,(-.2,0.),'forward','forward'),(0.,(-.2,0.),'reverse','reverse')])
def test_direction_from_measured_motion_or_low_speed_latch(yaw,velocity,latch,expected):
    captured=[]
    def planner(request,model):
        captured.append(request)
        return KinodynamicPlanResult('node_budget_exhausted',(),0,0.,0)
    adapter=SourceMapReplanner('0002',MODEL,planner=planner)
    adapter(h._observation(yaw=yaw,velocity=velocity),np.array([2.,0.]),active_direction=latch)
    assert captured[0].initial_direction==expected
    assert captured[0].direction_change_cost_m>.5

def test_measured_motion_latch_conflict_refuses_plan():
    adapter=SourceMapReplanner('0002',MODEL)
    with pytest.raises(ValueError,match='conflicts'):
        adapter(h._observation(velocity=(-.2,0.)),np.array([2.,0.]),active_direction='forward')
