import numpy as np,pytest
from pixel_box_polytope_v2 import envelope

def fixture():
 a=np.diag([1.,-1.,-1.,1.]);b=a.copy();b[0,3]=.3;return a,b,np.diag([100.,100.,1.])
def test_exact_stereo_box_depth_bounds():
 a,b,k=fixture();r=envelope([0,0],[-10,0],a,b,k,1.,.5)
 assert r['status']=='bounded';np.testing.assert_allclose([r['lo_world_m'][2],r['hi_world_m'][2]],[2.5,3.75],atol=1e-6)
 assert not r['measured_geometry'] and not r['certified_free'] and not r['statistical_confidence_calibrated']
def test_parallel_pixel_cones_explicit_unbounded():
 a,b,k=fixture();r=envelope([0,0],[0,0],a,b,k,1.,.5);assert r['status']=='unbounded' and r['lo_world_m'] is None
def test_inconsistent_correspondence_explicit_empty():
 a,b,k=fixture();r=envelope([0,0],[-10,100],a,b,k,1.,.5);assert r['status']=='empty' and r['lo_world_m'] is None
def test_gauge_translation_invariance():
 a,b,k=fixture();r=envelope([0,0],[-10,0],a,b,k,1.,.5);shift=np.array([5.,-8.,1.]);a[:3,3]+=shift;b[:3,3]+=shift;s=envelope([0,0],[-10,0],a,b,k,1.,.5);np.testing.assert_allclose(np.array(s['lo_world_m'])-shift,r['lo_world_m'],atol=1e-6);np.testing.assert_allclose(np.array(s['hi_world_m'])-shift,r['hi_world_m'],atol=1e-6)
def test_future_clock_before_payload():
 with pytest.raises(ValueError,match='past'):envelope(None,None,None,None,None,1.,2.)

def test_coincident_inconsistent_cones_cannot_be_bounded():
 a,b,k=fixture();b=a.copy();r=envelope([0,0],[100,100],a,b,k,1.,.5);assert r['status'] in ['empty','solver_failure']
def test_solver_reported_success_with_invalid_point_is_rejected(monkeypatch):
 import pixel_box_polytope_v2 as module
 from types import SimpleNamespace
 monkeypatch.setattr(module,'linprog',lambda *a,**kw:SimpleNamespace(status=0,success=True,x=np.zeros(3)))
 a,b,k=fixture();assert module.envelope([0,0],[-10,0],a,b,k,1.,.5)['status']=='solver_failure'
