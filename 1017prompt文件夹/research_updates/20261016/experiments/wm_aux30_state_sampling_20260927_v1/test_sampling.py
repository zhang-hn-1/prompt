import numpy as np
import pytest

def test_group_requires_actual_reverse_speed_and_angle():
 from sampling import classify_anchor
 assert classify_anchor(.2,-.1,16)==1
 assert classify_anchor(.2,-.1,-16)==2
 assert classify_anchor(.1,-.1,20)==0
 assert classify_anchor(.2,.1,20)==0
 assert classify_anchor(.2,-.1,14)==0
 assert classify_anchor(.15,-.05001,15)==1
 assert classify_anchor(.15,-.05,15)==0

def test_exact_budget_and_group_exposure():
 from sampling import balanced_schedule
 groups={0:[0,1,2,3,4],1:[10,11],2:[20,21]};out=balanced_schedule(groups,100,2,7)
 assert out.shape==(100,2)
 a=out.ravel();assert sum(a<10)==180 and sum((a>=10)&(a<20))==10 and sum(a>=20)==10
 assert set(a)=={0,1,2,3,4,10,11,20,21}
 assert np.count_nonzero(a==10)==5 and np.count_nonzero(a==11)==5

def test_reproducible_without_global_rng():
 from sampling import balanced_schedule
 groups={0:[0,1,2],1:[3,4],2:[5,6]};a=balanced_schedule(groups,100,2,3072);np.random.seed(0)
 assert np.array_equal(a,balanced_schedule(groups,100,2,3072))
 assert not np.array_equal(a,balanced_schedule(groups,100,2,1))

@pytest.mark.parametrize('groups', [{0:[0],1:[],2:[2]},{0:[0],1:[0],2:[2]},{0:[0],1:[1],2:[2,2]}])
def test_invalid_or_overlapping_groups_rejected(groups):
 from sampling import balanced_schedule
 with pytest.raises(ValueError):balanced_schedule(groups,100,2,1)
