import numpy as np,pytest
from retreat_observation_schedule import request
@pytest.mark.parametrize('t,main',[(0,0),(5.9,0),(6,-.3),(11.9,-.3),(12,0),(17.9,0)])
def test_exact_sequence(t,main):
 a=request(t);assert a.shape==(10,) and a.dtype==np.float32
 np.testing.assert_array_equal(a.reshape(5,2),np.tile(np.array([main,0],np.float32),(5,1)))
@pytest.mark.parametrize('t',[-1,np.inf,np.nan])
def test_reject_bad_clock(t):
 with pytest.raises(ValueError):request(t)
