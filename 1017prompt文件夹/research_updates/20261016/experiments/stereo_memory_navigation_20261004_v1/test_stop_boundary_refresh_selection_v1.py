import json
from pathlib import Path
import numpy as np
from audit_stop_boundary_reference_refresh_v1 import select_stop_boundary
D=Path(__file__).parent/'reverse_main_navigation_e1_b0_run_v1'
def test_actual_qualification_not_time_trigger():
 rows=json.loads((D/'policy_records.json').read_text());speeds=[float(np.linalg.norm(np.load(D/'history'/f'frame_{i:03d}.npz')['proprio_history'][-1,:2])) for i in range(len(rows))]
 assert select_stop_boundary(rows,speeds)==145
 # Current rebound blocks the otherwise four-cycle pending confirmation.
 speeds[145]=.05
 assert select_stop_boundary(rows[:146],speeds[:146]) is None

def test_pending_without_four_count_never_qualifies():
 rows=[{'tracker_state_after_act':dict(active_direction='forward',pending_direction='reverse',stopped_cycles=3,braking_state='direction_change')},{'tracker_state_after_act':dict(active_direction='reverse',pending_direction=None,stopped_cycles=0,braking_state='tracking')}]
 assert select_stop_boundary(rows,[0.,0.]) is None
