import numpy as np,pytest
from types import SimpleNamespace
from continuous_cell_tube_v1 import continuous_cell_tube_screen

def grid(risk=None):
 g=SimpleNamespace(origin=np.array([-5.,-5.]),resolution=.1,labels=np.full((101,101),-1,np.int8))
 g.cells=lambda p:np.rint((np.asarray(p)-g.origin)/.1).astype(int)
 if risk is not None:g.labels[tuple(g.cells([risk])[0])]=1
 return g

def guard(line):return {'bridge_xy':[line[0]],'suffix_xy':line}

def test_close_point_rejects_and_preserves_grid():
 g=grid([0,.8]);before=g.labels.copy();v=continuous_cell_tube_screen(g,guard([[-1,0],[1,0]]));assert not v['pass'];assert abs(v['minimum_continuous_cell_clearance_m']-.8)<1e-10;assert np.array_equal(before,g.labels)

def test_small_subcell_displacement_continuous_distance():
 g=grid([0,1]);v=continuous_cell_tube_screen(g,guard([[-1,.01],[1,.01]]));assert v['pass'];assert abs(v['minimum_continuous_cell_clearance_m']-.99)<1e-10

def test_diagonal_interior_minimum():
 g=grid([0,1]);v=continuous_cell_tube_screen(g,guard([[-1,-1],[1,1]]));assert not v['pass'];assert abs(v['minimum_continuous_cell_clearance_m']-1/np.sqrt(2))<1e-10

def test_boundary_occupancy_kept():
 g=grid();v=continuous_cell_tube_screen(g,guard([[-4.5,0],[-4.5,1]]));assert not v['pass'];assert v['minimum_continuous_cell_clearance_m']<.8707106782

def test_outside_and_bad_geometry():
 g=grid();assert not continuous_cell_tube_screen(g,guard([[6,0],[6,1]]))['pass']
 with pytest.raises(ValueError):continuous_cell_tube_screen(g,guard([[np.nan,0],[1,0]]))

def test_translation_covariance_and_no_free_claim():
 g=grid([0,1]);a=continuous_cell_tube_screen(g,guard([[-1,0],[1,0]]));g.origin+=np.array([.037,-.022]);b=continuous_cell_tube_screen(g,guard([[-.963,-.022],[1.037,-.022]]));assert abs(a['minimum_continuous_cell_clearance_m']-b['minimum_continuous_cell_clearance_m'])<1e-10;assert not b['certified_safe'];assert a['existing_planning_clearance_m']==.6+.2+.1*np.sqrt(2)/2
