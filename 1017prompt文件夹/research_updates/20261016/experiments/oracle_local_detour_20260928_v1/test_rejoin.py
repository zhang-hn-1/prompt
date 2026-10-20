import numpy as np
from rejoin import rejoin_path
from gate import clearance

def test_blocked_direct_uses_clear_same_side_checkpoint():
 p=rejoin_path(np.array([0.,-1.]),np.array([4.,0.]),np.array([2.,0.]),np.array([1.,0.]))
 assert p is not None and clearance(p,[2,0],.65)>0
 assert np.allclose(p[0],[0,-1]) and np.allclose(p[-1],[4,0])
 assert np.linalg.norm(np.diff(p,axis=0),axis=1).max()<=.20000001

def test_inside_obstacle_envelope_rejected():
 assert rejoin_path(np.array([2.,0.]),np.array([4.,0.]),np.array([2.,0.]),np.array([1.,0.])) is None
