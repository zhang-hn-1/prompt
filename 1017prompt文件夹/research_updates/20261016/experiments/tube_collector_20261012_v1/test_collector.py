import pytest
from collector import Collector

def completed():
 c=Collector('s');c.observe(1.,estimated_xy=(0.,0.),estimated_velocity=(0.,0.),estimated_heading=0.);c.commit(tuple((.3,.1) for _ in range(25)))
 c.supervise(1.,(0.,0.))
 for i in range(25):
  c.consume(1.+i*.02,(.3,.1))
  c.supervise(1.+(i+1)*.02,(i*.01,0.))
 return c

def test_joint():assert completed().qualify()['scorable']
def test_future_field():
 with pytest.raises(TypeError):Collector('s').observe(1.,gt_xy=(0,0))
def test_no_precommit_consume():
 with pytest.raises(ValueError):Collector('s').consume(1.,(.3,.1))
def test_program_mismatch():
 c=completed();c.consumed[2]=(1.04,(.3,.2));assert c.qualify()['reason']=='program_mismatch'
def test_clock_mismatch():
 c=completed();c.consumed[2]=(1.05,(.3,.1));assert c.qualify()['reason']=='consumption_clock_mismatch'
def test_duplicate_commit():
 c=completed()
 with pytest.raises(ValueError):c.commit(tuple((0.,0.) for _ in range(25)))
def test_censored():
 c=Collector('s');c.observe(1.,estimated_xy=(0.,0.),estimated_velocity=(0.,0.),estimated_heading=0.);c.commit(tuple((0.,0.) for _ in range(25)))
 assert c.qualify()['reason']=='censored_execution'

def test_nan_consumption_clock():
 c=completed();c.consumed[2]=(float("nan"),(.3,.1));assert c.qualify()["reason"]=="consumption_clock_mismatch"
