import numpy as np
import pytest
from source_map import SourceMapReplanner
from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
from test_candidate import h

@pytest.mark.parametrize('case',['0002','0004'])
def test_replan_uses_original_obstacles_and_transforms_both_ways(case):
    captured=[]
    def fake(request,model):
        captured.append(request)
        assert np.count_nonzero(request.occupancy_grid)>0
        assert request.node_budget==5000
        return KinodynamicPlanResult('success',(KinodynamicWaypoint(*request.start_xy,request.start_yaw_rad,'forward',.5,0.),KinodynamicWaypoint(*request.goal_xy,request.start_yaw_rad,'forward',0.,0.)),1,1.,0)
    fn=SourceMapReplanner(case,None,planner=fake)
    observation=h._observation(position=(.2,-.1),yaw=.3)
    goal=np.array([2.,-3.])
    result=fn(observation,goal)
    assert captured[0].start_xy==pytest.approx(fn.origin+fn.rotation@observation.position_world_xy)
    assert result.waypoints[0].x_m==pytest.approx(.2)
    assert result.waypoints[0].y_m==pytest.approx(-.1)
    assert result.waypoints[0].yaw_rad==pytest.approx(.3)
    assert (result.waypoints[-1].x_m,result.waypoints[-1].y_m)==pytest.approx(goal)
