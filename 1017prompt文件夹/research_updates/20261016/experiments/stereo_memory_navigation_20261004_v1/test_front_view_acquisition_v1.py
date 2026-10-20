import numpy as np,pytest
from front_view_acquisition_v1 import FrontViewAcquisition

def packet(t,frame=None,points=None):
 frame=int(round(t*10)) if frame is None else frame
 return dict(frame=frame,time_s=t,records=[dict(frame=frame,time_s=t,world=np.tile([3.,0.,.5],(100,1)) if points is None else points)])
def test_only_one_declared_motion_and_latched_stop():
 a=FrontViewAcquisition();np.testing.assert_array_equal(a.choose(0.,[0,0],0.,packet(0.)),np.zeros(10));np.testing.assert_allclose(a.choose(6.,[0,0],0.,packet(6.)),np.tile([.5,np.pi/6],5),atol=1e-7)
 a.choose(6.1,[.51,0],.2,packet(6.1));assert a.stopped
 np.testing.assert_array_equal(a.choose(6.2,[.2,0],.2,packet(6.2)),np.zeros(10));assert a.stopped

def test_stop_counter_requires_five_and_resets_on_rebound():
 a=FrontViewAcquisition();a.choose(0,[0,0],0,packet(0));a.choose(6,[.5,0],.1,packet(6))
 for t in [6.1,6.2,6.3,6.4]:assert a.choose(t,[.6,0],.04,packet(t)) is not None
 assert a.low_count==4;a.choose(6.5,[.6,0],.06,packet(6.5));assert a.low_count==0
 for t in [6.6,6.7,6.8,6.9]:a.choose(t,[.6,0],.04,packet(t))
 assert a.choose(7.,[.6,0],.04,packet(7.)) is None;assert a.handed_off

def test_future_source_rejected_before_payload():
 class Forbidden:
  def __array__(self,*args):raise AssertionError('future payload read')
 a=FrontViewAcquisition()
 with pytest.raises(ValueError,match='source clock'):a.choose(0,[0,0],0,dict(frame=1,time_s=1,records=[dict(frame=1,time_s=1,world=Forbidden())]))

def test_stale_or_too_few_points_cannot_handoff():
 a=FrontViewAcquisition();a.choose(0,[0,0],0,packet(0));a.choose(6,[.5,0],0,packet(6))
 for t in [6.1,6.2,6.3,6.4,6.5,6.6]:assert a.choose(t,[.6,0],0,packet(5.5)) is not None
 assert not a.handed_off
 assert a.choose(6.7,[.6,0],0,packet(6.7,points=np.empty((0,3)))) is not None
 assert not a.handed_off

def test_risk_halts_using_current_actual_points_not_scene():
 a=FrontViewAcquisition();a.choose(0,[0,0],0,packet(0));a.choose(6,[0,0],.1,packet(6,points=np.array([[.8,0,.5]])));assert a.stopped;assert a.halt_reason=='current_RGB_point_proximity'

def test_no_unbounded_initial_phase():
 a=FrontViewAcquisition();a.choose(0,[0,0],0,packet(0));a.choose(14,[0,0],.2,packet(14));assert a.stopped;assert a.choose(20,[0,0],.2,packet(20)) is not None;assert a.failed
