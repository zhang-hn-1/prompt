import numpy as np
from course_geometry import curvature_from_motion

def test_aligned_same_as_original():
 k,active=curvature_from_motion([1.,.5],[1.,0.],0.,'forward',.8)
 assert active and abs(k-.8)<1e-12

def test_actual_course_requires_opposite_turn():
 k,active=curvature_from_motion([1.,0.],[1.,1.],0.,'forward',0.)
 assert active and abs(k+2**.5)<1e-12

def test_reverse_and_low_speed_and_wrong_direction_keep_original():
 for velocity,direction in [([-.5,0.],'reverse'),([.01,.01],'forward'),([-.5,0.],'forward')]:
  k,active=curvature_from_motion([1.,.5],velocity,0.,direction,.123)
  assert not active and k==.123
