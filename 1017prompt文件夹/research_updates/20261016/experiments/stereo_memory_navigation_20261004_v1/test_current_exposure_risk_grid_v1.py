import numpy as np
import pytest
from current_exposure_risk_grid_v1 import CurrentExposureRiskGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid

def test_empty_new_exposure_forgets_risk_to_unknown_not_free():
 history=RepeatedExposureMemoryGrid([0,0],10,static_scene_assumption=True)
 current=CurrentExposureRiskGrid([0,0],10,static_scene_assumption=True)
 records=[]
 for f,t in [(0,0.),(1,.5)]:
  records.append(dict(frame=f,time_s=t,world=np.array([[1.,1.,.5]])))
  for g in [history,current]:
   g.set_source(dict(frame=f,time_s=t,records=records));g.update_points([],np.array([1.,1.,.4]),t)
 assert history.labels[10,10]==current.labels[10,10]==1
 for g in [history,current]:
  g.set_source(dict(frame=2,time_s=1.,records=records));g.update_points([],np.array([1.,1.,.4]),1.)
 assert history.labels[10,10]==1 and current.labels[10,10]==-1
 assert not np.any(current.labels==0)
 assert current.diagnostics()['discarded_history_is_observed_free'] is False

def test_future_clock_rejected_before_poisoned_payload():
 class Poison:
  def __array__(self,*a,**kw):raise AssertionError('future payload must not be touched')
 g=CurrentExposureRiskGrid([0,0],10,static_scene_assumption=True)
 r=dict(frame=2,time_s=2.,world=Poison());g.set_source(dict(frame=1,time_s=1.,records=[r]))
 with pytest.raises(ValueError):g.update_points([],np.array([1.,1.,.4]),1.)

def test_same_capture_id_cannot_silently_change_acquisition_clock():
 g=CurrentExposureRiskGrid([0,0],10,static_scene_assumption=True)
 r=dict(frame=1,time_s=.9,world=np.array([[1.,1.,.5]]))
 g.set_source(dict(frame=1,time_s=1.,records=[r]))
 with pytest.raises(ValueError,match='capture clock'):
  g.update_points([],np.array([1.,1.,.4]),1.)

def test_previous_envelope_cannot_resurrect_forgotten_history():
 g=CurrentExposureRiskGrid([0,0],10,static_scene_assumption=True)
 old=dict(frame=1,time_s=.5,world=np.array([[1.,1.,.5]]))
 g.set_source(dict(frame=1,time_s=.5,records=[old]));g.update_points([],np.array([1.,1.,.4]),.5)
 g.set_source(dict(frame=2,time_s=1.,records=[old]));g.update_points([],np.array([1.,1.,.4]),1.)
 g.update_points([],np.array([1.,1.,.4]),1.1) # holding the same latest envelope is legal
 assert g.labels[10,10]==-1
 g.set_source(dict(frame=1,time_s=.5,records=[old]))
 with pytest.raises(ValueError,match='monotone'):
  g.update_points([],np.array([1.,1.,.4]),1.2)
 assert g.labels[10,10]==-1
