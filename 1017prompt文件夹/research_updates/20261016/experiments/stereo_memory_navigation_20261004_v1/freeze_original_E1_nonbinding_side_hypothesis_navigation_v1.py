from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
review=H/'original_E1_nonbinding_side_hypothesis_first_review_v1.md';assert 'Critical: **0**' in review.read_text() and 'Important: **0**' in review.read_text()
native=json.loads((H/'original_E1_nonbinding_side_hypothesis_native_identity_preflight_v1.json').read_text());assert native['decision']=='PASS_NATIVE_IMPORT_CLASS_IDENTITY_AND_RESET_ONLY' and native['driver_sha256']==sha(H/'run_original_E1_nonbinding_side_hypothesis_navigation_v1.py')
cp=json.loads((H/'E1_nonbinding_side_hypothesis_shadow_CPU_v1/assessment.json').read_text());assert cp['decision']=='PASS_NECESSARY_INTERFACE_ONLY'
review_identity=json.loads((H/'original_E1_nonbinding_side_hypothesis_review_source_identity_v1.json').read_text())
for f,s in review_identity.items():assert sha(f)==s,f
closure=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),unresolved_critical=0,unresolved_important=0,first_review_sha256=sha(review),driver_sha256=sha(H/'run_original_E1_nonbinding_side_hypothesis_navigation_v1.py'),review_source_identity=review_identity)
(H/'original_E1_nonbinding_side_hypothesis_navigation_author_review_closed_v1.json').write_text(json.dumps(closure,indent=2)+'\n')
p=json.loads((H/'original_E1_cusp_coast_acquisition_navigation_protocol_v1.json').read_text())
for group in ['source_sha256','input_sha256']:
 for f,s in p[group].items():assert sha(f)==s,f
p.update(schema='original_E1_nonbinding_side_hypothesis_navigation_v1',utc=closure['utc'],only_control_change='nonbinding_partial_route_side_hypothesis_exclusion',comparison_baseline='closed_cusp_consumer_same_policy_hard_side_grid',previous_cusp_nonintervention_preserved=True,actual_startup_attempt_number=1)
for n in ['nonbinding_side_hypothesis_v1.py','test_nonbinding_side_hypothesis_v1.py','qualify_E1_nonbinding_side_hypothesis_shadow_v1.py','run_original_E1_nonbinding_side_hypothesis_navigation_v1.py','run_original_E1_nonbinding_side_hypothesis_navigation_stage_v1.py','preflight_E1_nonbinding_side_hypothesis_v1.py','freeze_original_E1_nonbinding_side_hypothesis_navigation_v1.py']:
 f=H/n;p['source_sha256'][str(f)]=sha(f)
for n in ['E1_nonbinding_side_hypothesis_shadow_CPU_v1/assessment.json','original_E1_nonbinding_side_hypothesis_first_review_v1.md','original_E1_nonbinding_side_hypothesis_navigation_author_review_closed_v1.json','original_E1_nonbinding_side_hypothesis_native_identity_preflight_v1.json','original_E1_nonbinding_side_hypothesis_review_source_identity_v1.json','original_E1_cusp_coast_acquisition_control_progress_assessment_v1.json','original_E1_cusp_coast_acquisition_intervention_assessment_v1.json','original_E1_cusp_coast_acquisition_sparse_memory_navigation_assessment_v1/assessment.json','original_E1_cusp_coast_acquisition_sparse_membership_v1/assessment.json','nonbinding_side_GREEN_v1.log']:
 f=H/n;p['input_sha256'][str(f)]=sha(f)
f=H.parent.parent/'docs/superpowers/plans/2026-10-08-E1-nonbinding-side-hypothesis.md';p['input_sha256'][str(f)]=sha(f)
out=H/'original_E1_nonbinding_side_hypothesis_navigation_protocol_v1.json';assert not out.exists();out.write_text(json.dumps(p,indent=2)+'\n');print('Frozen',len(p['source_sha256']),len(p['input_sha256']),sha(out))
