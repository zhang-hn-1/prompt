import numpy as np
import pytest
from observed_reference_tube_retention_v1 import observed_tube_retention_screen
class Grid:
 resolution=.1
 origin=np.array([-2.,-2.])
 labels=np.full((51,51),-1,dtype=int)
def grid():
 g=Grid();g.labels=g.labels.copy();g.labels[20,20]=1;return g
def guard(x=.84):
 return dict(retention_necessary_screen_pass=True,projection_distance_m=1.-x,bridge_xy=[[x,0.],[1.,0.]],suffix_xy=[[1.,0.],[1.,2.]])
def test_current_bridge_inside_existing_tube():
 r=observed_tube_retention_screen(grid(),guard());assert r['pass'] and r['old_combined_pass']==False and r['exception_used'];assert r['suffix_continuous_clearance_m']-.2>=.6

def test_old_pass_behavior_preserved():
 r=observed_tube_retention_screen(grid(),guard(.95));assert r['pass'] and r['old_combined_pass'] and not r['exception_used']

def test_excess_cross_rejected():
 g=guard(.75);g['projection_distance_m']=.25;r=observed_tube_retention_screen(grid(),g);assert not r['pass']

def test_unsafe_suffix_rejected():
 g=guard();g['suffix_xy']=[[1.,0.],[.7,0.]];r=observed_tube_retention_screen(grid(),g);assert not r['pass']

def test_original_guard_failure_never_overridden():
 g=guard();g['retention_necessary_screen_pass']=False;assert not observed_tube_retention_screen(grid(),g)['pass']

def test_domain_escape_rejected():
 g=guard();g['suffix_xy']=[[1.,0.],[20.,0.]];assert not observed_tube_retention_screen(grid(),g)['pass']

def test_invalid_projection_and_payload_rejected():
 for x in [np.nan,-1.]:
  g=guard();g['projection_distance_m']=x
  with pytest.raises(ValueError):observed_tube_retention_screen(grid(),g)

def test_unknown_never_becomes_free():
 g=grid();before=g.labels.copy();observed_tube_retention_screen(g,guard());np.testing.assert_array_equal(g.labels,before)

def test_forged_bridge_distance_rejected():
 g=guard();g['projection_distance_m']=.01
 with pytest.raises(ValueError):observed_tube_retention_screen(grid(),g)

def test_insufficient_continuous_margin_blocks_exception(monkeypatch):
 # Isolate the exact geometry proof input from the already-covered raster predicate.
 monkeypatch.setattr('observed_reference_tube_retention_v1.hazard_clearance',lambda *args:.79)
 r=observed_tube_retention_screen(grid(),guard());assert not r['pass'] and not r['exception_used']
