import numpy as np
from template import replay_prediction

def test_exact_start_reproduces_template():
    state={'joint':[1.,2.],'smoother':{'a':[0.,1.]}}
    xy=np.array([[0.,0.],[1.,2.]])
    assert np.array_equal(replay_prediction(state,state,xy),xy)

def test_history_mismatch_rejected():
    import pytest
    with pytest.raises(ValueError):replay_prediction({'joint':[1.],'history':[2.]},{'joint':[1.],'history':[3.]},np.zeros((2,2)))
