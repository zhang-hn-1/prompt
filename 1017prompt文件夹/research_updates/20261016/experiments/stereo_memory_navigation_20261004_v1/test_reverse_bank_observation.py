import numpy as np
import pytest
from reverse_bank_observation import ReverseBankObservation

def args(t,frame,root=(0,0),axis=(1,.3,0)):
 return dict(now=t,source_frame=frame,source_time=t,point_count=0,axis_world=axis,axis_time=t,root_xy=root,goal_xy=(2,0),parent=(0,0))

def test_reverse_family_and_once_only_stop():
 o=ReverseBankObservation((0,0))
 np.testing.assert_array_equal(o.request(**args(0,0)),(0,0))
 np.testing.assert_allclose(o.request(**args(.5,5)),(-.3,-np.pi/6))
 np.testing.assert_allclose(o.request(**args(.6,5,axis=(1,-.01,0))),(0,-np.pi/6))
 np.testing.assert_allclose(o.request(**args(.7,5)),(0,-np.pi/6))
 assert o.records[-1]['request']==[0,-np.pi/6]

def test_duration_and_displacement_limits():
 for t,root in [(4.5,(0,0)),(.6,(.5,0))]:
  o=ReverseBankObservation((0,0));o.request(**args(0,0));o.request(**args(.5,5))
  assert o.request(**args(t,5,root=root))[0]==0

def test_future_clock_and_inapplicable_axis():
 o=ReverseBankObservation((0,0));a=args(0,0);a['axis_time']=.1
 with pytest.raises(ValueError):o.request(**a)
 o.request(**args(0,0,axis=(1,-.3,0)))
 np.testing.assert_array_equal(o.request(**args(.5,5,axis=(1,-.3,0))),(0,0))

def test_actual_runtime_positional_interface():
 o=ReverseBankObservation((0,0))
 v=args(0,0);np.testing.assert_array_equal(o.request(*v.values()),(0,0))
