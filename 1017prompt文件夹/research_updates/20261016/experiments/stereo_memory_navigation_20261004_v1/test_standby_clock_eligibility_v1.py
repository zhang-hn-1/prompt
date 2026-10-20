import numpy as np
import pytest
from standby_clock_eligibility_v1 import ClockQualifiedStandby

def source(t):return dict(time_s=t,frame=int(t*10),records=[])
def test_five_actual_low_samples_trigger_without_six_second_floor():
    b=ClockQualifiedStandby(enabled=False);c=ClockQualifiedStandby(enabled=True)
    for i in range(4):
        t=i/10;np.testing.assert_array_equal(b.choose(t,[0.,0.],0.,source(t)),c.choose(t,[0.,0.],0.,source(t)))
    assert c.choose(.4,[0.,0.],0.,source(.4)) is None
    assert np.count_nonzero(b.choose(.4,[0.,0.],0.,source(.4)))==0
    assert c.handoff_time_s==.4 and c.standby_low_count==5

def test_rebound_resets_count_and_future_source_is_rejected_first():
    c=ClockQualifiedStandby(enabled=True)
    for i in range(4):c.choose(i/10,[0.,0.],0.,source(i/10))
    assert c.choose(.4,[0.,0.],.051,source(.4)) is not None and c.standby_low_count==0
    for i in range(5,9):assert c.choose(i/10,[0.,0.],0.,source(i/10)) is not None
    assert c.choose(.9,[0.,0.],0.,source(.9)) is None
    with pytest.raises(ValueError):ClockQualifiedStandby(enabled=True).choose(0,[0.,0.],0.,source(1))
