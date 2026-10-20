import importlib.util
import numpy as np

def test_early_end_uses_both_real_states_at_common_time():
 assert importlib.util.find_spec('matched_progress_v1') is not None, 'common-time progress evaluator missing'
 from matched_progress_v1 import matched_progress
 a=[dict(time_s=t,position=[x,0,0]) for t,x in [(0,0),(1,1),(2,2)]]
 b=[dict(time_s=t,position=[x,0,0]) for t,x in [(0,0),(1,.5),(2,1),(3,8)]]
 r=matched_progress(a,b,2.,[0,0],[1,0])
 assert r['axis_gain_m']==1.
 assert r['candidate_sample_time_s']==r['base_sample_time_s']==2.
