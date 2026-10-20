import numpy as np
from anchor_validation_schedule import request

def test_fixed_known_request_and_clock_boundaries():
 for t,main,aux in [(0,0,0),(6,-.3,0),(12,0,0),(18,.3,np.deg2rad(10)),(24,0,0),(35,0,0),(17.9999996,.3,np.deg2rad(10))]:
  a=request(t);np.testing.assert_array_equal(a,np.tile(np.array([main,aux],np.float32),5))
