import numpy as np
import pytest
from actual_pair_request_fork_v1 import first_actual_request_fork

def test_invalid_shadow60_is_not_actual_intervention120():
 a=np.zeros((350,10),np.float32);b=a.copy();b[120:,::2]=1
 assert first_actual_request_fork(a,b)==120
 # An unrelated shadow would diverge at60; it is not passed as the physical B0.
 shadow=a.copy();shadow[60:,::2]=.5
 assert first_actual_request_fork(shadow,b)==60

def test_no_change_or_truncation_is_not_an_invented_intervention():
 a=np.ones((350,10),np.float32)
 assert first_actual_request_fork(a,a.copy()) is None
 assert first_actual_request_fork(a,a[:200]) is None
 bad=a.copy();bad[0,0]=np.nan
 with pytest.raises(ValueError):first_actual_request_fork(a,bad)
