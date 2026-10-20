import numpy as np,pytest
from learned_cell_memory import CellMemory

def fixture(t=0.,shift=(0.,0.)):
 sh=np.array([*shift,0.]);p=dict(world=np.array([[.01,.01,.5]])+sh,sigma_m=np.array([.01]),query_pixel_id=np.array([18]),partner_pixel_id=np.array([10]),frame=int(t*10),time_s=t,frame_width=8)
 rgb=np.full((8,8,3),100,np.uint8);mask=np.zeros((8,8),bool);mask[0,0]=True;c=dict(time_s=t,left_rgb=rgb,right_rgb=rgb,left_mask=mask,left_camera_position=np.array([-1,.3,.45])+sh,root_position=np.array([-1,0,.4])+sh,previous_request=np.zeros(10));return p,c

def test_fresh_has_exact21_finite_features_and_no_truth_labels():
 p,c=fixture();m=CellMemory([-2,-2],(40,40),.1);m.ingest(p,c,0);s=m.snapshot(0);assert s['features'].shape==(1,21) and np.isfinite(s['features']).all();assert 'labels' not in s and s['model_outputs_are_observed'] is False

def test_repeated_identity_does_not_increase_count_or_refresh_time():
 p,c=fixture();m=CellMemory([-2,-2],(40,40),.1);m.ingest(p,c,0);old=m.snapshot(0);m.ingest(p,c,5);new=m.snapshot(5);np.testing.assert_equal(old['features'][:,:2],new['features'][:,:2]);assert new['last_query_times'][0]==0 and new['features'][0,2]==5/35

def test_future_payload_rejected_before_any_value_read():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return super().__getitem__(k)
 m=CellMemory([-2,-2],(40,40),.1)
 with pytest.raises(ValueError):m.ingest(Bomb(time_s=9,world=None),{},1)

def test_future_or_truncated_sequences_leave_prefix_identical():
 a=CellMemory([-2,-2],(40,40),.1);b=CellMemory([-2,-2],(40,40),.1)
 for t in [0.,.5,1.]:
  p,c=fixture(t);a.ingest(p,c,t);b.ingest(p,c,t)
 np.testing.assert_equal(a.snapshot(1)['features'],b.snapshot(1)['features'])
 p,c=fixture(2);p['world']*=999
 with pytest.raises(ValueError):a.ingest(p,c,1)
 np.testing.assert_equal(a.snapshot(1)['features'],b.snapshot(1)['features'])

def test_absolute_xy_cell_keys_cannot_change_features():
 a=CellMemory([-2,-2],(40,40),.1);b=CellMemory([98,-52],(40,40),.1)
 for t in [0.,.5]:
  p,c=fixture(t);a.ingest(p,c,t);q,d=fixture(t,(100,-50));b.ingest(q,d,t)
 np.testing.assert_allclose(a.snapshot(.5)['features'],b.snapshot(.5)['features'],rtol=0,atol=1e-12)

def test_observation_age_expires_without_refresh():
 p,c=fixture();m=CellMemory([-2,-2],(40,40),.1);m.ingest(p,c,0);assert len(m.snapshot(35)['cells'])==1;assert len(m.snapshot(35.1)['cells'])==0

def test_replay_changed_same_query_value_rejected():
 p,c=fixture();m=CellMemory([-2,-2],(40,40),.1);m.ingest(p,c,0);p['world'][0,2]=.6
 with pytest.raises(ValueError):m.ingest(p,c,1)
