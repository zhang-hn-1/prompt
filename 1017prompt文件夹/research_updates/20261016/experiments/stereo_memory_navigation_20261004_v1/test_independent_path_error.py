import numpy as np,pytest
from independent_path_error import distance_to_complete_reference

def test_segment_interior_uses_geometry_not_waypoint_or_policy_index():
 ref=np.array([[0.,0.],[2.,0.]])
 assert distance_to_complete_reference([1.,.2],ref)==pytest.approx(.2)
 np.testing.assert_array_equal(ref,[[0.,0.],[2.,0.]])

def test_complete_polyline_handles_turn_endpoints_and_repeated_vertices():
 ref=np.array([[0.,0.],[1.,0.],[1.,0.],[1.,2.]])
 assert distance_to_complete_reference([1.2,1.],ref)==pytest.approx(.2)
 assert distance_to_complete_reference([1.,3.],ref)==pytest.approx(1.)
 assert distance_to_complete_reference([.5,0.],ref)==pytest.approx(0.)

def test_invalid_geometry_cannot_silently_produce_small_error():
 for xy,ref in [([np.nan,0],[[0,0],[1,0]]),([0,0],[[0,0]]),([0,0],[[0,0],[np.inf,0]])]:
  with pytest.raises(ValueError):distance_to_complete_reference(xy,ref)
