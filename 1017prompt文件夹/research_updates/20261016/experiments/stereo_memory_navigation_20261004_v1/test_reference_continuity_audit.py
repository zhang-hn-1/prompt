import numpy as np
import pytest
import reference_continuity_audit as subject

def grid(hazard=()):
 a=np.full((100,100),-1,np.int8);origin=np.array([-5.,-5.])
 for x,y in hazard:a[int(round((x+5)*10)),int(round((y+5)*10))]=1
 return a,origin

def test_midsegment_projection_uses_old_remaining_reference():
 a,o=grid();r=subject.reference_geometry_status([.75,.2],[[0,0],[2,0],[2,2]],a,o)
 assert r['projected_arc_m']==pytest.approx(.75)
 assert r['projection_distance_m']==pytest.approx(.2)
 assert r['remaining_arc_m']==pytest.approx(3.25)
 np.testing.assert_allclose(r['suffix_xy'],[[.75,0],[2,0],[2,2]])

def test_hazard_behind_projection_is_not_remaining_path_collision():
 a,o=grid([(0,0)]);r=subject.reference_geometry_status([1,0],[[0,0],[2,0]],a,o)
 assert r['suffix_clearance_m']==pytest.approx(1)
 assert r['continuous_clearance_pass'] and r['raster_clearance_pass']

def test_bridge_hazard_invalidates_otherwise_clear_suffix():
 a,o=grid([(1,.8)]);r=subject.reference_geometry_status([1,1.5],[[0,0],[2,0]],a,o)
 assert r['suffix_clearance_m']==pytest.approx(.8)
 assert r['bridge_clearance_m']==pytest.approx(0)
 assert not r['continuous_clearance_pass'] and not r['raster_clearance_pass']

def test_segment_midpoint_hazard_is_detected():
 a,o=grid([(1,0)]);r=subject.reference_geometry_status([0,0],[[0,0],[2,0]],a,o)
 assert r['suffix_clearance_m']==pytest.approx(0)
 assert not r['continuous_clearance_pass'] and not r['raster_clearance_pass']

def test_no_hazards_does_not_fabricate_floor_evidence():
 a,o=grid();before=a.copy();r=subject.reference_geometry_status([0,0],[[0,0],[2,0]],a,o)
 assert r['suffix_clearance_m'] is None and r['observed_hazard_count']==0
 assert r['continuous_clearance_pass'] and r['raster_clearance_pass']
 assert r['nominal_geometry_only'] and not r['certified_safe']
 np.testing.assert_array_equal(a,before);assert (a==-1).all()

def test_invalid_geometry_does_not_become_empty_safe_path():
 a,o=grid()
 for path in [[[0,0]],[[0,0],[0,0]],[[0,0],[np.nan,1]]]:
  with pytest.raises(ValueError):subject.reference_geometry_status([0,0],path,a,o)
 with pytest.raises(ValueError):subject.reference_geometry_status([np.inf,0],[[0,0],[1,0]],a,o)
