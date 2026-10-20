import numpy as np
from observation_handoff import ObservationHandoff

def packet(t,n=100):
 return dict(time_s=t,world=np.tile([0.,0.,.5],(n,1)))

def test_five_low_samples_and_current_source_required_then_latched():
 g=ObservationHandoff();base=np.tile([.3,.1],5)
 for t in [17.6,17.7,17.8,17.9]:
  assert not g.choose(t,.01,packet(t),base).any()
 np.testing.assert_equal(g.choose(18.,.01,packet(18),base),base)
 assert g.handed_off
 np.testing.assert_equal(g.choose(18.1,.2,packet(18.1,0),base),base)

def test_rebound_resets_and_absent_hazard_cannot_qualify():
 g=ObservationHandoff();base=np.ones(10)
 for t in [17.6,17.7,17.8,17.9]:g.choose(t,.01,packet(t),base)
 g.choose(18.,.05,packet(18),base)
 assert g.low_count==0 and not g.handed_off
 for t in [18.1,18.2,18.3,18.4,18.5]:g.choose(t,.01,packet(t,99),base)
 assert not g.handed_off

def test_initial_retreat_clock_only_and_future_payload_never_read():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return super().__getitem__(k)
 g=ObservationHandoff();base=np.ones(10)
 assert not g.choose(5.,0.,Bomb(time_s=20),base).any()
 np.testing.assert_equal(g.choose(6.,0.,Bomb(time_s=20),base),np.tile([-.3,0.],5).astype(np.float32))
 for t in [17.6,17.7,17.8,17.9,18.]:assert not g.choose(t,.01,Bomb(time_s=20),base).any()
 assert not g.handed_off
