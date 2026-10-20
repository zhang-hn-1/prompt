import numpy as np
import pytest

def test_equal_update_budget_and_complete_epoch_coverage():
 from training_scope import batch_schedule
 for size,epochs in [(1848,30),(2772,20)]:
  batches=list(batch_schedule(list(range(size)),epochs,2,3072))
  assert len(batches)==27720
  assert np.array_equal(np.bincount(np.concatenate(batches)),np.full(size,epochs))
  assert len([s for s in range(1,len(batches)+1) if s%924==0])==30

def test_schedule_does_not_use_global_rng_or_unlisted_ids():
 from training_scope import batch_schedule
 one=list(batch_schedule([4,8,12,16],2,2,9));np.random.seed(777)
 two=list(batch_schedule([4,8,12,16],2,2,9))
 assert all(np.array_equal(a,b) for a,b in zip(one,two))
 assert set(np.concatenate(one))=={4,8,12,16}

def test_only_whole_disjoint_sequences_can_expand_training():
 from training_scope import expanded_partition
 old=[{'episode':s} for s in ['fit','fit','val','test']];new=[{'episode':'new'},{'episode':'new'}]
 assert expanded_partition(old,[[0,1],[2],[3]],new)==[[0,1,4,5],[2],[3]]
 with pytest.raises(ValueError):expanded_partition(old,[[0,1],[2],[3]],[{'episode':'test'}])
 with pytest.raises(ValueError):expanded_partition(old,[[0],[1,2],[3]],new)

def test_invalid_budget_rejected():
 from training_scope import batch_schedule
 with pytest.raises(ValueError):list(batch_schedule([0,0],1,2,1))
 with pytest.raises(ValueError):list(batch_schedule([0,1,2],1,2,1))
