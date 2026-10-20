import numpy as np
import pytest
from tracking_measurement import measure_turn

def test_straight():
 r=measure_turn([[0,0],[.1,0],[.2,0]],[0,0,0]);assert r['xy_curvature_1_m']==0 and r['body_yaw_change_rad']==0

def test_circle_forward():
 # radius .5, CCW from (0,-.5) through (.5,0) to (0,.5).
 r=measure_turn([[0,-.5],[.5,0],[0,.5]],[0,0,0]);assert r['xy_curvature_1_m']==pytest.approx(2)

def test_circle_reverse():
 r=measure_turn([[0,.5],[.5,0],[0,-.5]],[0,0,0]);assert r['xy_curvature_1_m']==pytest.approx(-2)

def test_body_rotation_not_xy_turn():
 r=measure_turn([[0,0],[.1,0],[.2,0]],[0,.1,.2]);assert r['xy_curvature_1_m']==0 and r['body_yaw_per_xy_arc_1_m']==pytest.approx(1)

def test_stationary_unavailable():
 r=measure_turn([[0,0],[0,0],[0,0]],[0,0,0]);assert not r['geometric_available'] and r['xy_curvature_1_m'] is None

def test_nonfinite_rejected():
 with pytest.raises(ValueError):measure_turn([[0,0],[.1,np.nan],[.2,0]],[0,0,0])
