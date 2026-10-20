import numpy as np
import pytest

def packet(t,world):
 return dict(acquisition_time_s=t,world=world)

def args():
 return dict(past=packet(0.,np.array([[.0498,0.,.2],[-.0004,0.,.2]])),current=packet(.5,np.array([[.0506,0.,.2]])),pose=dict(time_s=.5,left_camera_usd_world=np.eye(4)),history=dict(proprio_history=np.zeros((5,17)),action_history=np.zeros((5,10))),history_time_s=.5,current_time_s=.5,memory_origin=np.zeros(2),memory_resolution=.1)

def test_selection_uses_actual_memory_cells_not_floor_bins():
 from memory_availability_features_v1 import past_only_features
 r=past_only_features(**args())
 assert r['source_indices'].tolist()==[0,1]
 assert r['features'].shape==(2,7)
 assert np.array_equal(r['features'][:,3],np.array([.5,.5]))

def test_current_same_cell_is_excluded_without_ground_or_truth():
 from memory_availability_features_v1 import past_only_features
 a=args();a['current']['world']=np.array([[.0004,0.,.2]])
 assert past_only_features(**a)['source_indices'].size==0

def test_old_first64_budget_before_current_support_filter():
 from memory_availability_features_v1 import past_only_features
 a=args();a['past']['world']=np.tile(np.array([.0498,0.,.2]),(80,1));a['current']['world']=np.empty((0,3))
 assert past_only_features(**a)['source_indices'].tolist()==list(range(64))

@pytest.mark.parametrize('bad',['past','current','pose','history'])
def test_clock_rejection_precedes_world_payload_read(bad):
 from memory_availability_features_v1 import past_only_features
 class Poison:
  def __array__(self,*a,**k):raise AssertionError('world touched before clock rejection')
 a=args();a['past']['world']=Poison()
 if bad in ('past','current'):a[bad]['acquisition_time_s']=1.
 if bad=='pose':a['pose']['time_s']=1.
 if bad=='history':a['history_time_s']=1.
 with pytest.raises(ValueError):past_only_features(**a)

def test_feature_action_is_only_supplied_previous_history():
 from memory_availability_features_v1 import past_only_features
 a=args();a['history']['action_history'][-1]=np.tile([.3,.1],5);a['history']['proprio_history'][-1,:2]=[3.,4.]
 r=past_only_features(**a)
 assert np.allclose(r['features'][:,4:],[[.3,.1,5.],[.3,.1,5.]])
