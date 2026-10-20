import numpy as np,pytest
from live_refined_surface_shadow import refined_surface
from stereo_geometry import FOCAL

def test_synthetic_vertical_geometry_and_paired_pixel_identity():
 rng=np.random.default_rng(7);left=rng.integers(0,255,(480,640,3),dtype=np.uint8);right=np.zeros_like(left);right[:280]=left[200:]
 masks={e:np.zeros((480,640),bool) for e in ("left","right")};surface=dict(acquisition_time_s=1.,frame=10,query_pixel_id=np.array([300*640+320]));T=np.eye(4);T[:3,:3]=np.diag([1,-1,-1]);p=refined_surface(surface,left,right,masks,np.ones((480,640),bool),np.full((480,640),200.),T,1.,{"rgb":"a"*64})
 assert p["query_pixel_id"].tolist()==[300*640+320] and p["partner_pixel_id"].tolist()==[100*640+320]
 assert np.isclose(p["world"][0,2],FOCAL*.6/200)

def test_future_clock_rejected_before_payload():
 class Trap(dict):
  def __getitem__(self,k):
   if k!="acquisition_time_s":raise AssertionError("future payload")
   return super().__getitem__(k)
 with pytest.raises(ValueError):refined_surface(Trap(acquisition_time_s=2),None,None,None,None,None,None,1.,None)
