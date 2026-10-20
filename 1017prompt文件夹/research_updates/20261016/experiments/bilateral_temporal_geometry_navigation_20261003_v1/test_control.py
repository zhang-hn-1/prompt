import numpy as np
from control import observed_obstacles, front_blocked, choose_side, make_detour


def test_floor_and_near_body_are_not_obstacle_clearance():
    points=np.array([[1,0,0],[.1,.1,.4],[2,0,.5],[2,0,.51]])
    cells=observed_obstacles(points,np.zeros(2))
    assert len(cells)==1 and np.allclose(cells[0],[2,0])


def test_side_walls_do_not_trigger_central_detour():
    x=np.linspace(1,3,20);points=np.column_stack((x,np.full(20,1.)))
    assert not front_blocked(points,np.zeros(2),np.array([6.,0.]))
    wall=np.column_stack((np.full(20,2.),np.linspace(-.4,.4,20)))
    assert front_blocked(wall,np.zeros(2),np.array([6.,0.]))


def test_right_eye_obstacle_can_change_side_choice():
    left=np.array([[2.,.8],[2.,1.]])
    right=np.array([[2.,-1.5],[2.,-1.4]])
    assert choose_side(left,np.zeros(2),np.array([8.,0.]))[0]==-1
    assert choose_side(np.vstack((left,right)),np.zeros(2),np.array([8.,0.]))[0]==1


def test_detour_has_same_start_goal_and_signed_offset():
    for side in (-1,1):
        path=make_detour(np.zeros(2),np.array([8.,0.]),side)
        assert np.allclose(path[0],[0,0]) and np.allclose(path[-1],[8,0])
        assert np.isclose(side*path[len(path)//2,1],2.3)
