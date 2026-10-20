import json,numpy as np
from pathlib import Path

def test_real_frozen_reference_outgoing_edge_endpoint_can_have_next_direction():
 from forward_course_score_evidence_v4 import verify_geometry
 d=Path(__file__).resolve().parent/'forward_course_geometry_e1_b0_run_v1'
 rows=json.loads((d/'policy_records.json').read_text());states=json.loads((d/'states.json').read_text());history=[]
 for r in rows[245:250]:
  prop=np.load(d/'history'/f"frame_{r['frame']:03d}.npz")['proprio_history'][-1]
  history.append((r['time_s'],prop[:2],float(prop[12])))
 assert rows[249]['current_reference_waypoints_after_act'][5]['drive_direction']=='forward'
 assert rows[249]['forward_course_geometry']['composition_input']['reference_direction']=='reverse'
 verify_geometry(rows[249],states[249],history,enabled=False)
