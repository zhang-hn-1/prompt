"""Final identity/archive/document closure only; does not rerun science."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'time_cost_protocol.json').read_text());ident=json.loads((H/'time_cost_final_identity.json').read_text());sync=json.loads((H/'time_cost_document_sync.json').read_text());r=json.loads((H/'time_cost_assessment.json').read_text());v=json.loads((H/'time_cost_verification.json').read_text())
for group in ['source_sha256','input_sha256']:
 for f,d in p[group].items():assert sha(f)==d,f
for f,d in ident['artifact_sha256'].items():assert sha(f)==d,f
for f,d in sync['documents'].items():assert sha(f)==d,f
for current,old in p['current_document_archives'].items():assert Path(current).read_bytes().startswith(Path(old).read_bytes()),current
report=Path(ident['report']);review=json.loads((H/'time_cost_review.json').read_text());assert sha(report)==ident['final_report_sha256'];assert hashlib.sha256(report.read_bytes()[:ident['reviewed_report_body_bytes']]).hexdigest()==ident['reviewed_report_body_sha256'];assert review['reviewed_body_sha256']==ident['reviewed_report_body_sha256']
assert sha(H/'time_cost_assessment.json')==v['assessment_sha256'];assert p['sealed_utc']<r['started_utc']<r['search_started_utc']<r['search_completed_utc']<r['completed_utc']<v['verified_utc']
assert r['target_candidate_hybrid_searches']==1 and r['target_original_hybrid_searches']==0 and r['shared_grid_route_constructions']==1
assert r['arms'][1]['status']=='node_budget_exhausted' and r['arms'][1]['expanded_nodes']==3000 and not r['cpu_geometry_eligibility_pass']
assert all(r[k]==0 for k in ['new_physical_sequences','policy_act_calls','model_fits'])
assert ident['stage_complete'] and not ident['whole_research_complete'] and ident['automation']=='ACTIVE';assert not ident['review_fresh_context']
assert json.loads((H/'time_cost_tasks.json').read_text())['Task3'].startswith('complete')
status=dict(checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),PASS=True,stage_complete=True,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),artifact_checks=len(ident['artifact_sha256']),current_document_checks=len(sync['documents']),new_target_searches=1,original_target_replays=0,physical_sequences=0,CPU_candidate='REJECT',whole_research_complete=False,automation='ACTIVE',review_context='retained; fresh dispatch limit rejected')
(H/'time_cost_final_check.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status,indent=2))
