"""Breaks caught: probing without certification or omitting real braking time."""
import numpy as np
import observation_schedule as s

def test_four_bounded_whole_sequences_include_settle_and_brake():
    for name in ('zero','bank','forward','reverse'):
        a=s.schedule(name)
        assert a.shape==(120,10)
        assert np.all(a[:60]==0) and np.all(a[-20:]==0)
        assert np.abs(a[:,::2]).max()<=.3+1e-7
        assert np.abs(a[:,1::2]).max()<=np.deg2rad(15)+1e-7
    assert s.schedule('forward')[65,0]>.0
    assert s.schedule('reverse')[65,0]<.0

def test_unknown_start_cannot_authorize_motion():
    assert not s.start_authorized(0.,.6)
    assert not s.start_authorized(.4,.6)
    assert s.start_authorized(.8,.6)
