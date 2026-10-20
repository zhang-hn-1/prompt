import numpy as np,pytest
from source_event_handoff import SourceEventHandoff
from observation_handoff import ObservationHandoff

def packet(t=15.,frame=150):return dict(time_s=t,frame=frame,world=np.tile([[1.,0.,.4],[1.2,.2,.4]],(60,1)))
def event(t=15.,frame=150):return dict(time_s=t,frame=frame,confirmed=True)
def nominal():return np.r_[.3,-.2,np.zeros(8)].astype(np.float32)
def test_prefix_matches_B0_and_pre_event_waits_zero():
 a=SourceEventHandoff();b=ObservationHandoff()
 for t in np.arange(0,15,.1):np.testing.assert_array_equal(a.choose(float(t),0.,packet(float(t),int(t*10)),nominal(),None),b.choose(float(t),0.,packet(float(t),int(t*10)),nominal()))
 assert not a.handed_off

def test_event_after_low_five_hands_off_once_and_preserves_nominal():
 a=SourceEventHandoff();n=nominal();copy=n.copy()
 for t in [14.6,14.7,14.8,14.9]:assert not a.choose(t,0.,packet(t,int(t*10)),n,None).any()
 np.testing.assert_array_equal(a.choose(15.,0.,packet(),n,event()),n);assert a.handed_off and a.handoff_time_s==15.;np.testing.assert_array_equal(n,copy);np.testing.assert_array_equal(a.choose(15.1,.2,None,n,None),n)

def test_rebound_resets_five_sample_requirement_and_event_does_not_force_switch():
 a=SourceEventHandoff()
 for t in [14.6,14.7,14.8,14.9]:a.choose(t,0.,packet(t,int(t*10)),nominal(),None)
 a.choose(15.,.05,packet(),nominal(),event());assert a.low_count==0 and not a.handed_off
 a.choose(15.1,0.,packet(),nominal(),event());assert a.low_count==1 and not a.handed_off

def test_future_event_rejected_before_packet_geometry():
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('future geometry read')
 a=SourceEventHandoff()
 with pytest.raises(ValueError,match='event clock'):a.choose(15,0,dict(time_s=15,frame=150,world=Trap()),nominal(),event(15.1,151))

def test_stale_packet_or_source_preceding_confirmed_event_does_not_handoff():
 for e,p in [(event(14,140),packet(14,140)),(event(),packet(14.9,149))]:
  a=SourceEventHandoff()
  for t in [14.6,14.7,14.8,14.9]:a.choose(t,0.,packet(t,int(t*10)),nominal(),None)
  assert not a.choose(15,0,p,nominal(),e).any();assert not a.handed_off

def test_historical_confirmation_can_resume_after_speed_rebound_without_expiry_deadlock():
 a=SourceEventHandoff()
 for t in [14.6,14.7,14.8,14.9]:a.choose(t,0.,packet(t,int(t*10)),nominal(),None)
 a.choose(15,.05,packet(),nominal(),event())
 for t in [15.1,15.2,15.3,15.4]:a.choose(t,0.,packet(t,int(round(t*10))),nominal(),event())
 np.testing.assert_array_equal(a.choose(15.5,0.,packet(15.5,155),nominal(),event()),nominal());assert a.handed_off
