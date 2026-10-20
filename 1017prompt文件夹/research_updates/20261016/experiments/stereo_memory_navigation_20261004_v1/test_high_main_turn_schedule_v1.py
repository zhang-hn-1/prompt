import numpy as np,pytest
from high_main_turn_schedule_v1 import schedule

def test_shared_prefix_and_single_main_change():
 a,b=schedule('low'),schedule('high')
 assert a.shape==b.shape==(300,10) and a.dtype==b.dtype==np.float32
 assert np.array_equal(a[:120],b[:120]) and np.array_equal(a[220:],b[220:])
 assert np.array_equal(a[:,1::2],b[:,1::2])
 assert np.all(a[60:220,::2]==.5) and np.all(b[120:220,::2]==1.)
 assert np.all(a[60:220,1::2]==np.float32(-np.pi/6))
 assert np.all(a[:60]==0) and np.all(b[220:]==0)

def test_unknown_arm_rejected():
 with pytest.raises(ValueError):schedule('arbitrary')
