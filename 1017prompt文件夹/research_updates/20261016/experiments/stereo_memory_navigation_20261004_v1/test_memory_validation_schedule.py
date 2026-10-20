import pytest,numpy as np
from memory_validation_schedule import request
@pytest.mark.parametrize('kind,main',[('near',-.3),('far',.3)])
@pytest.mark.parametrize('t',[6.,9.,11.9])
def test_known_requests(t,kind,main):np.testing.assert_equal(request(t,kind).reshape(5,2),np.tile(np.array([main,0],np.float32),(5,1)))
@pytest.mark.parametrize('t',[0.,12.,17.9])
def test_zero(t):np.testing.assert_equal(request(t,'far'),np.zeros(10,np.float32))
def test_invalid_kind():
 with pytest.raises(ValueError):request(6,'new_unplanned')
