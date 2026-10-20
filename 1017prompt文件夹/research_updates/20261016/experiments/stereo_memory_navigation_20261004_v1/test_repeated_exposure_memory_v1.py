import importlib.util
import numpy as np
import pytest

def make():
 assert importlib.util.find_spec('repeated_exposure_memory_v1') is not None,'persistent promotion consumer missing'
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 return RepeatedExposureMemoryGrid([-20,-20],10,static_scene_assumption=True)

def packet(frame,t):return dict(frame=frame,time_s=t,world=np.array([[0.,0.,.4],[.01,0.,.4]]))
def hazard(g):return g.labels[tuple(g.cells(np.array([[0.,0.]]))[0])]

def test_singleton_keeps_recent_risk_then_expires_to_unknown_even_reconsumed():
 g=make();a=packet(5,1.)
 for t in [1.,5.,10.9]:g.consume_records([a],t,[-3,0,.4]);assert hazard(g)==1
 g.consume_records([],12.,[-3,0,.4]);assert hazard(g)==-1 and not (g.labels==0).any()

def test_distinct_epochs_promote_and_preserve_actual_clock():
 g=make();g.consume_records([packet(5,1.)],1.,[-3,0,.4]);g.consume_records([packet(5,1.),packet(10,2.)],2.,[-3,0,.4]);g.consume_records([],20.,[-3,0,.4]);assert hazard(g)==1
 c=tuple(g.cells(np.array([[0.,0.]]))[0]);assert g.hazard_acquisition_time[c]==2. and g.diagnostics()['persistent_confirmation_required_epochs']==2

def test_future_acquisition_rejected_before_payload():
 g=make()
 class Poison:
  def __array__(self,*args,**kwargs):raise AssertionError('future payload read')
 with pytest.raises(ValueError,match='future acquisition'):g.consume_records([dict(frame=5,time_s=2.,world=Poison())],1.,[-3,0,.4])

def test_same_frame_cannot_change_its_acquisition_clock():
 g=make();g.consume_records([packet(5,1.)],1.,[-3,0,.4])
 with pytest.raises(ValueError,match='frame acquisition'):g.consume_records([packet(5,2.)],2.,[-3,0,.4])
