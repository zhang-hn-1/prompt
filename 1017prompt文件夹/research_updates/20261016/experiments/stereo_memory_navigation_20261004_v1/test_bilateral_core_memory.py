import numpy as np
import pytest

def packet(t, xyz=()):
 return dict(time_s=t,frame=round(t*10),pending_seed=False,raw_world_voxel_centres=np.asarray(xyz,float).reshape(-1,3),model_hypothesis=True,measured_geometry=False,certified_free=False,calibrated_uncertainty_m=.2,source_model_birth_time_s=2.,source_seed_time_s=2.,support_frames=np.array([round((t-.5)*10),round(t*10)]),support_times_s=np.array([t-.5,t]))

def test_empty_later_window_does_not_erase_past_core_or_reinflate_it():
 from bilateral_core_memory import BilateralCoreMemory
 m=BilateralCoreMemory();m.update(packet(2.,[[0,0,.5]]),2.);m.update(packet(2.5),2.5)
 q=m.freeze(2.5)
 assert len(q['possible_xy'])==13
 np.testing.assert_equal(q['raw_world_voxel_centres'],[[0,0,.5]])
 assert q['source_model_birth_time_s']==2. and q['hypothesis_birth_time_s']==2.5
 assert not q['measured_geometry'] and not q['certified_free']
 assert m.core_provenance()[0]['first_derived_time_s']==2.

def test_clock_and_future_support_rejected_before_mutation():
 from bilateral_core_memory import BilateralCoreMemory
 m=BilateralCoreMemory()
 with pytest.raises(ValueError):m.update({'time_s':3.},2.)
 q=packet(2.,[[0,0,.5]]);q['support_times_s']=np.array([2.,3.])
 with pytest.raises(ValueError):m.update(q,2.)
 m.update(packet(2.,[[0,0,.5]]),2.)
 with pytest.raises(ValueError):m.update(packet(2.),2.)
 assert len(m.core_provenance())==1

def test_source_identity_and_frozen_pool_cannot_be_refreshed():
 from bilateral_core_memory import BilateralCoreMemory
 m=BilateralCoreMemory();m.update(packet(2.,[[0,0,.5]]),2.)
 for key,val in [('calibrated_uncertainty_m',.1),('source_model_birth_time_s',2.5),('measured_geometry',True)]:
  q=packet(2.5);q[key]=val
  with pytest.raises(ValueError):m.update(q,2.5)
 q=m.freeze(2.);q['possible_xy'][:]=999
 assert np.max(np.abs(m.freeze(2.)['possible_xy']))<=.2
 with pytest.raises(ValueError):m.update(packet(2.5),2.5)

def test_distinct_exposure_windows_provenance_preserved_for_same_core():
 from bilateral_core_memory import BilateralCoreMemory
 m=BilateralCoreMemory();m.update(packet(2.,[[0,0,.5]]),2.);m.update(packet(2.5,[[0,0,.5],[.1,0,.5]]),2.5)
 rows=m.core_provenance();assert len(rows)==2
 assert rows[0]['derived_frames']==[20,25] and rows[0]['last_derived_time_s']==2.5
 assert m.windows[1]['support_frames']==[20,25]

def test_nonfinite_current_clock_and_fractional_frames_cannot_enter_provenance():
 from bilateral_core_memory import BilateralCoreMemory
 for now in [float('nan'),float('inf')]:
  m=BilateralCoreMemory()
  with pytest.raises(ValueError):m.update(packet(2.,[[0,0,.5]]),now)
  assert not m.cores
 for key,value in [('frame',20.9),('support_frames',np.array([15.5,20.]))]:
  m=BilateralCoreMemory();q=packet(2.,[[0,0,.5]]);q[key]=value
  if key=='support_frames':q['support_times_s']=np.array([1.55,2.])
  with pytest.raises(ValueError):m.update(q,2.)
  assert not m.cores
