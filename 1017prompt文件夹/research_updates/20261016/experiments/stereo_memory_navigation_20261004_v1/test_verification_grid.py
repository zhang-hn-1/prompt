import numpy as np,pytest
from prior_rgb_navigation import PriorRGBGrid
from verification_grid import VerificationGrid
@pytest.mark.parametrize('mode',['current','history'])
def test_whole_past_source_sequence_and_future_guard_equal(mode):
 a=PriorRGBGrid([-.5,-.5],(20,20),.1,mode);b=VerificationGrid([-.5,-.5],(20,20),.1,mode)
 for i,t in enumerate([0.,.1,.5,.6]):
  source=dict(world=np.array([[.1,0,.2],[.2,0,0]]),sigma_m=np.array([.01,.01]),query_pixel_id=np.array([1,2]),partner_pixel_id=np.array([3,4]),frame=i//2,time_s=0. if i<2 else .5)
  for x in (a,b):x.set_source(source);x.update_points([],np.array([0,0,.4]),t)
  for k in ('labels','codes','safe_start_prior','task_ground_prior'):np.testing.assert_equal(getattr(a,k),getattr(b,k))
  assert a.diagnostics()==b.diagnostics() and a.mapper.records==b.mapper.records
 for x in (a,b):
  before=x.labels.copy();x.set_source(dict(time_s=2.));
  with pytest.raises(ValueError):x.update_points([],np.array([0,0,.4]),.6)
  np.testing.assert_equal(before,x.labels)
