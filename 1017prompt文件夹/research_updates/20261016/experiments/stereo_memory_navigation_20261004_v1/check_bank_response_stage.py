"""Fresh closure identity and documentary validation; no science rerun."""
from pathlib import Path
import json,hashlib,datetime,re
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
load=lambda n:json.loads((H/n).read_text())
p=load('bank_response_protocol.json'); ident=load('bank_response_final_identity.json'); sync=load('bank_response_document_sync.json'); r=load('bank_response_assessment.json'); v=load('bank_response_verification.json'); status=load('bank_response_status.json'); pkg=load('bank_response_review_package.json'); review=load('bank_response_review.json')
for group in ['source_sha256','input_sha256']:
 for f,d in p[group].items():assert sha(f)==d,f
for f,d in ident['artifact_sha256'].items():assert sha(f)==d,f
manifest=load('bank_response_run/artifact_manifest_post_close.json')['artifact_sha256']
for f,d in manifest.items():assert sha(f)==d,f
for f,d in sync['documents'].items():assert sha(f)==d,f
for current,old in p['current_document_archives'].items():assert Path(current).read_bytes().startswith(Path(old).read_bytes()),current
body=Path(pkg['report']).read_bytes()[:pkg['reviewed_body_bytes']]
assert hashlib.sha256(body).hexdigest()==pkg['reviewed_body_sha256']==review['reviewed_body_sha256']
assert sha(pkg['report'])==ident['final_report_sha256']
assert sha(H/'bank_response_assessment.json')==v['assessment_sha256']
assert sha(H/'bank_response_protocol.json')==r['protocol_sha256']
assert p['sealed_utc']<status['start_utc']<status['end_utc']<r['measured_utc']<v['verified_utc']
assert status['status']=='finished' and status['exit_code']==0 and status['maximum_sequences']==1
assert r['actual_new_sequences']==1 and r['historical_windows_remeasured']==1
assert all(r[k]==0 for k in ['new_control_candidates','new_planner_searches','new_model_fits'])
assert r['bounded_case_consistency_pass'] and all(r['gates'].values())
assert not any(r[k] for k in ['historical_matrix_or_refit_repeated','causal_gain_effect_isolated','full_bank_mapping_admitted','physical_dynamic_model_admitted','navigation_admitted','whole_research_complete'])
assert v['actual_requests']==320 and v['physics_samples']==1601 and v['captures']==321 and v['rgb_samples']==65 and v['feedback_execution_ticks_exact']==1600 and v['postclose_artifacts']==len(manifest)==717
assert v['physical_contacts']==0 and v['contact_samples']==1921
assert v['measured_feedback_reconstruction_only'] and v['not_recursive_physical_prediction']
assert len(v['decoded_videos'])==2 and all(x['decoded_frames']==321 and x['fps']==10 for x in v['decoded_videos'])
assert ident['stage_complete'] and not ident['whole_research_complete'] and ident['automation']=='ACTIVE' and ident['review_fresh_context']
assert all(x.startswith('complete') for x in load('bank_response_tasks.json').values())
assert review['fresh_context'] and review['critical']==review['important']==0
assert len(review['declined_to_judge'])==len(review['author_scope_rulings'])
scheduler=Path('/home/an/.codex/automations/automation/automation.toml')
values=re.findall(r'^status\s*=\s*"([^"]+)"\s*$',scheduler.read_text(),re.M)
assert values==['ACTIVE'],values
result=dict(checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),PASS=True,stage_complete=True,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),actual_run_artifact_checks=len(manifest),stage_artifact_checks=len(ident['artifact_sha256']),current_document_checks=len(sync['documents']),actual_new_sequences=1,planner_searches=0,model_fits=0,control_candidates=0,bounded_single_case='PASS; not full mapping or navigation',review_context='fresh second person',whole_research_complete=False,automation='ACTIVE')
(H/'bank_response_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
