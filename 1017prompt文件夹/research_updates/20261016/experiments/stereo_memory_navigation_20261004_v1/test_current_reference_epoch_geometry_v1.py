import numpy as np
from current_reference_epoch_geometry_v1 import project

def test_ideal_straight_and_corner_actual_point_geometry():
 x=project([[0,0],[2,0]], [1,.3]);assert abs(x['distance_m']-.3)<1e-12 and x['arc_min_m']==x['arc_max_m']==1
 x=project([[0,0],[1,0],[1,1]], [1,.4]);assert x['distance_m']==0 and abs(x['arc_min_m']-1.4)<1e-12

def test_self_crossing_does_not_choose_cursor_or_largest_progress():
 x=project([[-1,-1],[1,1],[-1,1],[1,-1]], [0,0]);assert x['ambiguous'] and x['arc_max_m']>x['arc_min_m']+2

def test_retraversed_segment_preserves_two_possible_arcs():
 x=project([[0,0],[1,0],[0,0]], [.2,0]);assert x['ambiguous'] and np.allclose([x['arc_min_m'],x['arc_max_m']],[.2,1.8])
