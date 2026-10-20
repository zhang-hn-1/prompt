import numpy as np
from kernel import aggregate
cost=np.array([[[0.,20.,1000.]]]);support=np.array([[[False,True,True]]])
total,paths=aggregate(cost,support)
assert np.isinf(total[0,0,0]) and np.isfinite(total[0,0,1])
