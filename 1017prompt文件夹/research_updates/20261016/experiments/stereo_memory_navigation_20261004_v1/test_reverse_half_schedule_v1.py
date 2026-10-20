import numpy as np
from reverse_half_schedule_v1 import schedule
def test_only_new_reverse_point_with_full_zero_prefix_and_tail():
    a=schedule()
    assert a.shape==(320,10) and a.dtype==np.float32
    assert np.count_nonzero(a[:60])==np.count_nonzero(a[260:])==0
    assert np.all(a[60:260,::2]==-.5)
    assert np.count_nonzero(a[:,1::2])==0
