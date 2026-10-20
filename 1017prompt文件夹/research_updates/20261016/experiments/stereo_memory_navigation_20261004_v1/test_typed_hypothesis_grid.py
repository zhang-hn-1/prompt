import numpy as np,pytest
from typed_hypothesis_grid import TypedHypothesisGrid

def packet(t=1.5):return dict(time_s=t,hypothesis_birth_time_s=t,source_seed_time_s=t,possible_xy=np.array([[0.,0.],[.1,0.]]),model_hypothesis=True,measured_geometry=False,certified_free=False)
def source():return dict(time_s=0.,frame=0,world=np.empty((0,3)))
def test_model_not_measured_and_expiry_only_difference():
 a=TypedHypothesisGrid([-2,-2],retention_s=10);b=TypedHypothesisGrid([-2,-2],retention_s=60)
 for g in [a,b]:
  g.seed(packet(),1.5);g.set_source(source());g.update_points(np.empty((0,3)),[-1,0,0],1.5)
  assert (g.provenance()==6).sum()==2 and not np.isfinite(g.memory.hazard_time).any() and not np.isfinite(g.memory.floor_time).any()
  g.update_points(np.empty((0,3)),[-1,0,0],11.6)
 assert not (a.labels==1).any() and (b.provenance()==7).sum()==2
 assert np.isfinite(b.hypothesis_birth_time).sum()==2 and (b.hypothesis_birth_time[np.isfinite(b.hypothesis_birth_time)]==1.5).all()
def test_birth_immutable_and_future_rejected_before_payload():
 g=TypedHypothesisGrid([-2,-2],retention_s=60);p=packet();g.seed(p,1.5);p['possible_xy'][:]=99
 np.testing.assert_equal(g.birth['possible_xy'],[[0,0],[.1,0]])
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload')
   return 2.
 with pytest.raises(ValueError):TypedHypothesisGrid([-2,-2],retention_s=60).seed(Bomb(),1.5)
def test_disallowed_measured_geometry_rejected():
 g=TypedHypothesisGrid([-2,-2],retention_s=60);p=packet();p['measured_geometry']=True
 with pytest.raises(ValueError):g.seed(p,1.5)
def test_later_seed_does_not_refresh_birth():
 g=TypedHypothesisGrid([-2,-2],retention_s=60);g.seed(packet(),1.5)
 with pytest.raises(ValueError):g.seed(packet(2.),2.)
def test_measured_bootstrap_deadline_stops_without_false_execution():
 from typed_hypothesis_grid import require_bootstrap_progress,BootstrapNotExercised
 from static_seed_belief import SafeStartBootstrap
 g=SafeStartBootstrap()
 for i in range(81):
  g.choose(i*.1*(1-2.23517418e-8),0.,None,np.ones(10),None,i)
  if i<80:require_bootstrap_progress(g,current_frame=i)
  else:
   with pytest.raises(BootstrapNotExercised):require_bootstrap_progress(g,current_frame=i)
 assert g.blocked and not g.handed_off
