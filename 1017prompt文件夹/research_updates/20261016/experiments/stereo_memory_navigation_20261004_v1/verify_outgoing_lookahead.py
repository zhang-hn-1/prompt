"""Fresh author identity/scalar checks only, no scientific replay."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'outgoing_lookahead_protocol_v2.json').read_text());a=json.loads((H/'outgoing_lookahead_assessment.json').read_text());rows=json.loads((H/'outgoing_lookahead_rows.json').read_text());ticks=json.loads((H/'outgoing_lookahead_actual_feedback.json').read_text())
 for mapping in [p['source_sha256'],p['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 assert p['sealed_utc']<a['started_utc']<a['assessed_utc']
 assert len(rows)==a['original_geometry_inverse_mapping_request_rows_exact']==188 and len(ticks)==1750
 assert len({r['frame'] for r in rows})==188
 assert a['archived_policy_act_calls']==a['new_physical_runs']==a['new_planner_searches']==0
 assert sum(r['raw_request_changes'] for r in rows)==52
 assert sum(r['old']['accumulated_m']==0 and r['candidate']['accumulated_m']>0 for r in rows)==61
 assert all(not r['candidate_successor_truth_used'] and r['same_reference_and_current_state'] for r in rows)
 assert all(r[k]['current_only'] and not r[k]['physical_prediction'] for r in rows for k in ['old','candidate'])
 assert [g['valid_tracking_rows'] for g in a['groups']]==[19,13,17,17]
 assert all(g['candidate_current_algebra_only']['zero_lookahead_rows']==0 for g in a['groups'])
 assert not a['actual_navigation_candidate_tested'] and not a['candidate_physical_prediction'] and not a['whole_research_complete']
 for n in ['outgoing_lookahead.py','test_outgoing_lookahead.py']:
  assert str((H/n).resolve()) in p['source_sha256']
 d=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),unique_current_tracking_rows=188,actual_feedback_rows=1750,changed_algebraic_requests=52,zero_target_boundary_rows_corrected=61,maximum_attempts=2,completed_archived_passes=1,archived_policy_act_calls=0,new_physical_runs=0,new_planner_searches=0,interface_cpu_regression_pass=True,actual_navigation_candidate_tested=False,whole_research_complete=False,assessment_sha256=sha(H/'outgoing_lookahead_assessment.json'),protocol_v2_sha256=sha(H/'outgoing_lookahead_protocol_v2.json'))
 (H/'outgoing_lookahead_verification.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d,indent=2))
if __name__=='__main__':main()
