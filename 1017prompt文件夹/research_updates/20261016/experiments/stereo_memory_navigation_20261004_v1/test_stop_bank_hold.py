import numpy as np
import pytest
from short_branch import development_schedule
from stop_bank_hold import hold_bank_schedule

def test_hold_uses_last_known_request_and_preserves_main_brake_and_prefix():
    prefix=development_schedule()[:75].copy();before=prefix.copy()
    result=hold_bank_schedule(prefix)
    np.testing.assert_array_equal(prefix,before)
    np.testing.assert_array_equal(result[:75],prefix)
    assert result.shape==(135,10) and not result[75:,0::2].any()
    np.testing.assert_array_equal(result[75:,1::2],np.tile(prefix[-1,1::2],(60,1)))
    # Held reference comes from the request, never an applied/measured angle.
    prefix[:,1::2]=np.float32(-np.pi/12)
    changed=hold_bank_schedule(prefix)
    np.testing.assert_array_equal(changed[75:,1::2],np.full((60,5),np.float32(-np.pi/12)))

def test_no_future_requests_or_incomplete_prefix_are_admitted():
    with pytest.raises(ValueError):hold_bank_schedule(development_schedule())
    with pytest.raises(ValueError):hold_bank_schedule(development_schedule()[:74])
    bad=development_schedule()[:75].copy();bad[-1,0]=np.nan
    with pytest.raises(ValueError):hold_bank_schedule(bad)
