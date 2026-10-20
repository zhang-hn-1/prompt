import numpy as np
from reference import build_cost
rgb=np.zeros((9,12,3),np.uint8)
c,m=build_cost(rgb,rgb,np.array([0,1]))
assert c.shape==m.shape==(9,12,2)
