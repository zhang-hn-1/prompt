import numpy as np
from pacing_reference_metrics_v1 import measure

def test_projected_index_is_irrelevant_and_parking_short_fails_endpoint():
 s=[dict(time_s=i*.1,position=[min(i*.1,.4),.1,0],velocity=[0,0,0],path_index=999) for i in range(11)]
 r=measure(s,[[0,0],[1,0]],0)
 assert np.isclose(r['real_arc_final_m'],.4) and np.isclose(r['cross_track_RMS_m'],.1)
 assert r['first_endpoint_passage_from_install_s'] is None and r['endpoint_final_m']>.2

def test_endpoint_passage_not_terminal_acceptance_if_overshoot():
 s=[dict(time_s=i*.1,position=[i*.1,0,0],velocity=[1,0,0]) for i in range(16)]
 r=measure(s,[[0,0],[1,0]],0)
 assert r['first_endpoint_passage_from_install_s'] is not None and r['endpoint_final_m']>.2 and r['terminal_stop']['last_count']==0
