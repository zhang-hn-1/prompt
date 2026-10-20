import bootstrap
import numpy as np
from end_subgoal import select_target

class Grid:
    resolution=.1
    origin=np.array([-5.,-5.])
    def __init__(self):self.labels=np.full((101,101),-1,dtype=np.int8)
    def cells(self,xy):return np.floor((xy-self.origin)/self.resolution+.5).astype(int)

g=Grid();g.labels[50,40:71]=1
before=g.labels.copy();q,d=select_target(g,np.array([-2.,0.]),np.array([3.,0.]),np.array([0.,-1.8]))
assert q is not None and q[0]<-.85 and q[1]<-1.
assert d['exploratory_unknown'] and not d['observed_free']
np.testing.assert_array_equal(g.labels,before)
empty=Grid();assert select_target(empty,np.array([-2.,0.]),np.array([3.,0.]),np.array([0.,-1.8]))[0] is None
g=Grid();g.labels[49:52,49:52]=1
assert select_target(g,np.array([-2.,0.]),np.array([3.,0.]),np.array([0.,-1.8]))[0] is None
g=Grid();g.labels[50,40:71]=1
assert select_target(g,np.array([-4.,0.]),np.array([3.,0.]),np.array([0.,-1.8]))[0] is None
print('PASS 5 target geometry/map immutability/unknown/degenerate/horizon cases')
