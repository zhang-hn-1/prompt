import numpy as np,pytest
from short_obstacle_task import short_obstacle_endpoints

def test_declared_symmetric_endpoints_preserve_height_and_direction():
 a=np.array([-3.3,0,.45]);b=np.array([3.3,0,.45]);x,y=short_obstacle_endpoints(a,b);np.testing.assert_allclose(x,[-1.3,0,.45]);np.testing.assert_allclose(y,[1.3,0,.45]);np.testing.assert_equal(a,[-3.3,0,.45]);np.testing.assert_equal(b,[3.3,0,.45])
 x,y=short_obstacle_endpoints([0,-3.3,.45],[0,3.3,.45]);np.testing.assert_allclose(x,[0,-1.3,.45]);np.testing.assert_allclose(y,[0,1.3,.45])

def test_invalid_or_too_short_goal_cannot_silently_move():
 for a,b in [([0,0,.45],[4,0,.45]),([0,0],[6,0]),([np.nan,0,.45],[6,0,.45])]:
  with pytest.raises(ValueError):short_obstacle_endpoints(a,b)
