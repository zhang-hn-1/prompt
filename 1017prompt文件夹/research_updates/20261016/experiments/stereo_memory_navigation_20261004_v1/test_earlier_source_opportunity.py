import numpy as np,pytest
from earlier_source_opportunity import EarlierSourceOpportunity

def source(t):
 xyz=np.tile([[1.,0.,.4],[1.2,.2,.4]],(60,1));return dict(time_s=t,world=xyz)
def stopped():return np.zeros((5,2))
def test_first_two_consecutive_qualifying_sources_latch_once():
 a=EarlierSourceOpportunity();assert a.update(source(14.5),stopped(),14.5,145) is None;assert a.update(source(15),stopped(),15,150)==[145,150];assert a.update(source(15.5),stopped(),15.5,155) is None

def test_velocity_rebound_clears_prior_count():
 a=EarlierSourceOpportunity();a.update(source(14.5),stopped(),14.5,145);v=stopped();v[2,0]=.05;assert a.update(source(15),v,15,150) is None;assert a.update(source(15.5),stopped(),15.5,155) is None;assert a.update(source(16),stopped(),16,160)==[155,160]

def test_empty_source_resets_and_does_not_fake_opportunity():
 a=EarlierSourceOpportunity();a.update(source(14.5),stopped(),14.5,145);s=source(15);s['world']=np.zeros((0,3));assert a.update(s,stopped(),15,150) is None;assert a.update(source(15.5),stopped(),15.5,155) is None

def test_future_clock_rejected_before_source_or_speed_payload():
 class Trap:
  def __array__(self,*a,**k):raise AssertionError('future payload consumed')
 a=EarlierSourceOpportunity()
 with pytest.raises(ValueError,match='clock'):a.update(dict(time_s=16,world=Trap()),Trap(),15,150)
