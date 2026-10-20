import numpy as np
from bank_rock_observation_schedule import request

def test_boundary_frames_and_main_are_frozen():
 for frame,bank in [(0,0),(59,0),(60,10),(79,10),(80,-10),(99,-10),(100,0),(139,0)]:
  a=request(frame);assert a.shape==(10,) and a.dtype==np.float32
  np.testing.assert_equal(a[::2],np.zeros(5,np.float32))
  np.testing.assert_allclose(a[1::2],np.deg2rad(bank/.75),atol=1e-8)

def test_no_commands_after_whole_sequence():
 import pytest
 with pytest.raises(ValueError):request(140)
 with pytest.raises(ValueError):request(-1)
