import numpy as np,pytest
from current_motion_heading_feasibility_v1 import derive_motion_heading

def test_forward_and_reverse_measured_not_requested():
 t=np.arange(5)*.1
 a=derive_motion_heading(.4,t,np.tile([.2,0.],(5,1)),np.full(5,.5));assert a==0.
 a=derive_motion_heading(.4,t,np.tile([-.2,0.],(5,1)),np.full(5,-.3));assert abs(a)<1e-12
 assert derive_motion_heading(.4,t,np.tile([0.,.2],(5,1)),np.full(5,.5))==pytest.approx(np.pi/2)
def test_unstable_or_coasting_direction_is_unsupported():
 t=np.arange(5)*.1;v=np.tile([.2,0.],(5,1))
 assert derive_motion_heading(.4,t,v,np.zeros(5)) is None
 assert derive_motion_heading(.4,t,np.zeros((5,2)),np.full(5,.5)) is None
 assert derive_motion_heading(.4,t,v,np.array([.5,.5,-.3,-.3,-.3])) is None
 assert derive_motion_heading(.3,t[:4],v[:4],np.full(4,.5)) is None
def test_future_clock_before_motion_payload():
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('future motion read')
 with pytest.raises(ValueError):derive_motion_heading(.3,np.arange(5)*.1,Poison(),Poison())
