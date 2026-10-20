import numpy as np
import pytest
from primitive_state_key_lineage_v1 import summarize_saved_keys

def test_two_root_latches_plus_unique_prefix_states():
 checks=[dict(samples=[[0,0,0],[.3,0,0]],primitive_free=True),dict(samples=[[0,0,0],[.3,.01,0]],primitive_free=True)]
 r=summarize_saved_keys([0,0,0],checks,[0,0],.1)
 assert r['distinct_free_prefix_keys']==1 and r['root_latch_keys']==2 and r['finite_state_count_including_roots']==3

def test_bad_sample_refused():
 with pytest.raises(ValueError):summarize_saved_keys([0,0,0],[dict(samples=[[0,0,0],[float('nan'),0,0]],primitive_free=True)],[0,0],.1)
