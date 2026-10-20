"""Meaningful pre-call snapshot; old presealed test bytes retained unchanged."""
import copy,numpy as np
from test_masked_source_pair import inputs
from masked_source_pair import paired_sources

def test_pair_does_not_mutate_inputs_with_pre_call_snapshot():
 p,k,h,c=inputs();p_before=copy.deepcopy(p);k_before=copy.deepcopy(k);h_before=h.copy();frozen_before=copy.deepcopy(c.frozen)
 paired_sources(p,k,h,c,6.)
 np.testing.assert_equal(p,p_before);np.testing.assert_equal(k,k_before);np.testing.assert_equal(h,h_before);np.testing.assert_equal(c.frozen,frozen_before)
