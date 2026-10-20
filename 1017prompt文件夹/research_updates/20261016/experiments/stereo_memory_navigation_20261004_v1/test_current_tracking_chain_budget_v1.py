import json
from pathlib import Path
def test_readonly_single_run_active_protocol_has_no_stale_pair_claim():
 r=json.loads((Path(__file__).resolve().parent/'current_tracking_chain_e1_protocol_v1.json').read_text())
 assert r['maximum_physical_sequences']==r['new_physical_budget']==r['physical_run_budget']==1
 assert r['arms']==['b0'] and list(r['condition_protocols'])==['known_e1']
 assert r['design'].endswith('current_tracking_chain_telemetry_scope_v1.json')
 assert r['only_change']=='readonly_current_tracker_record_serialization_no_control_change'
 assert r['not_performance_AB'] and 'only_change_evidence' not in r
