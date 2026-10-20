import numpy as np
import pytest
from standby_handoff_v1 import StandbyHandoff
from front_view_acquisition_v1 import FrontViewAcquisition

def src(t):return dict(frame=int(round(t*10)),time_s=t,records=[])
def test_frozen_zero_prefix_then_handoff_uses_measured_standby_not_point_count():
 old=FrontViewAcquisition();new=StandbyHandoff()
 for i in range(60):
  t=i/10;np.testing.assert_array_equal(old.choose(t,[0,0],.01,src(t)),new.choose(t,[0,0],.01,src(t)))
 assert new.choose(6,[0,0],.01,src(6)) is None
 assert old.choose(6,[0,0],.01,src(6)) is not None
 assert new.diagnostics['handoff_not_observation_or_safety_certificate']
def test_rebound_resets_initial_standby_confirmation():
 p=StandbyHandoff()
 for i in range(5):p.choose(5.5+i/10,[0,0],.01,src(5.5+i/10))
 assert p.choose(6,[0,0],.051,src(6)) is not None
 for i in range(4):assert p.choose(6.1+i/10,[0,0],.01,src(6.1+i/10)) is not None
 assert p.choose(6.5,[0,0],.01,src(6.5)) is None

def test_handoff_does_not_allow_future_source_clock():
 p=StandbyHandoff()
 with pytest.raises(ValueError):p.choose(6,[0,0],.01,src(7))
