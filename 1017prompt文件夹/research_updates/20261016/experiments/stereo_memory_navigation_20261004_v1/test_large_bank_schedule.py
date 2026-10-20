import numpy as np
from large_bank_schedule import schedule

def test_two_whole_sequences_share_prefix_and_brake_without_internal_reset():
    plus=schedule('positive');minus=schedule('negative')
    assert plus.shape==minus.shape==(180,10)
    assert not plus[:60].any() and not minus[:60].any()
    np.testing.assert_array_equal(plus[60:120,::2],np.full((60,5),.3,np.float32))
    np.testing.assert_array_equal(plus[:,::2],minus[:,::2])
    np.testing.assert_array_equal(plus[:,1::2],-minus[:,1::2])
    assert abs(float(plus[60,1])-np.pi/6)<1e-7
    assert not plus[120:].any() and not minus[120:].any()
    assert np.max(abs(plus[:,::2]))<=.3+1e-7 and np.max(abs(plus[:,1::2]))<=np.pi/6+1e-7
