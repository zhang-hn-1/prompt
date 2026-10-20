import numpy as np
from tangent import tangent_path
from gate import clearance

def test_heading_and_goal_preserved():
 p=tangent_path(np.array([0.,0.]),0.,np.array([2.,3.]),np.array([20.,20.]))
 assert p is not None and np.allclose(p[-1],[2,3])
 assert abs(np.arctan2(*(p[1]-p[0])[::-1]))<.03
 assert np.linalg.norm(np.diff(p,axis=0),axis=1).max()<=.101

def test_start_in_collision_rejects():
 assert tangent_path(np.zeros(2),0.,np.array([2.,3.]),np.zeros(2)) is None
