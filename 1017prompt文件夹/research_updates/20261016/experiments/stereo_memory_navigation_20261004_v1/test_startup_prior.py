import numpy as np
import pytest
from startup_prior import ImmutableStartupPrior

def make():return ImmutableStartupPrior([[1,1],[1,2]],6.,source='task_assumption',trusted=True)

def test_explicit_nonobservation_trust_required():
 for source,trust in [('rgb',True),('task_assumption',False)]:
  with pytest.raises(ValueError):ImmutableStartupPrior([[1,1]],6.,source=source,trusted=trust)

def test_cold_start_and_clock():
 x=make();labels=np.full((4,4),-1,dtype=np.int8)
 assert x.evaluate(labels,[[1,1]],5.9)['unknown_cells']==1
 assert x.evaluate(labels,[[1,1]],6.)['prior_support_cells']==1
 with pytest.raises(ValueError):x.evaluate(labels,[[1,1]],float('nan'))

def test_prior_never_writes_observed_and_hazard_wins():
 x=make();labels=np.full((4,4),-1,dtype=np.int8);old=labels.copy();labels[1,2]=1
 r=x.evaluate(labels,[[1,1],[1,2]],7.)
 assert r['prior_support_cells']==1 and r['occupied_cells']==1 and not r['necessary_support_passed']
 assert labels[1,1]==old[1,1]==-1 and r['certified_free'] is False and r['action_authorized'] is False

def test_region_is_immutable_and_does_not_follow_robot():
 cells=np.array([[1,1],[1,2]]);x=ImmutableStartupPrior(cells,6.,source='task_assumption',trusted=True);cells[:]=2
 r=x.evaluate(np.full((5,5),-1,dtype=np.int8),[[1,1],[2,2]],10.)
 assert r['prior_support_cells']==1 and r['unknown_cells']==1

