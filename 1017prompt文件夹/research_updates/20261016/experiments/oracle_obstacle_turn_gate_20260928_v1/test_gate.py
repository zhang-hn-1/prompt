import numpy as np
from gate import clearance

def test_obstacle_between_samples_detected():
 assert clearance(np.array([[0.,0.],[2.,0.]]),[1.,0.],.65)<0

def test_tangent_and_clear():
 assert abs(clearance(np.array([[0.,0.],[2.,0.]]),[1.,.65],.65))<1e-12
 assert abs(clearance(np.array([[0.,0.],[2.,0.]]),[1.,1.],.65)-.35)<1e-12
