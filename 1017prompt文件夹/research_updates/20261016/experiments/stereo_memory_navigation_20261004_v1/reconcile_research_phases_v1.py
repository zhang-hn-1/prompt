"""Export measured phases without merging distinct pose/task conditions."""
from pathlib import Path
import hashlib,json,datetime
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
source={};checked={}
def read(name):
 p=H/name;source[str(p)]=sha(p);return json.loads(p.read_text())
def verify_hashes(mapping):
 for p,expected in mapping.items():
  actual=sha(p);assert actual==expected,(p,expected,actual);checked[p]=actual
base=read('classical_baseline_registry_v3.json');rolling=read('rolling_endpoint_candidate_registry_v2.json');learning=read('learned_memory_risk_holdout_assessment_v1.json');design=read('learned_memory_risk_qualifier_design_v1.json');review=read('learned_memory_risk_offline_closed_review_v1.json');geometry=read('runtime_geometric_curvature_review_v2.json');read('research_phase_reconciliation_scope_v1.json')
records=[]
for layer,rows in base['layers'].items():
 for r in rows:
  verify_hashes(r['input_sha256']);m=r['recomputed_metrics']
  records.append(dict(variant='frozen_classical',pose_layer=layer,run=r['run'],outcome=m['outcome'],success=m['success'],goal_remaining_m=m['goal_remaining_m'],net_axis_progress_m=m['along_start_goal_axis_m'],arrival_time_s=m['arrival_time_s'],contact_time_s=m['contact_time_s'],budget_s=m['budget_s'],strict_cold_start=False))
verify_hashes({rolling['original_baseline_registry']:rolling['original_baseline_registry_sha256']})
for condition,r in rolling['records'].items():
 verify_hashes({r['assessment']:r['assessment_sha256']});m=r['results']
 records.append(dict(variant='rolling_endpoint',pose_layer='known_pose' if condition.startswith('known') else 'estimated_pose',run=condition,outcome=m['outcome'],success=m['success'],goal_remaining_m=m['goal_remaining_m'],net_axis_progress_m=m['along_start_goal_axis_m'],arrival_time_s=m['arrival_time_s'],contact_time_s=m['contact_time_s'],budget_s=m['budget_s'],strict_cold_start=False))
assert len(records)==12 and all(r['budget_s']==35 for r in records)
counts={}
for variant in ('frozen_classical','rolling_endpoint'):
 counts[variant]={}
 for layer in ('known_pose','estimated_pose'):
  rows=[r for r in records if r['variant']==variant and r['pose_layer']==layer];assert len(rows)==3
  counts[variant][layer]=dict(arrived=sum(r['success'] for r in rows),conditions=len(rows))
assert counts['frozen_classical']==base['success_counts'];assert counts['rolling_endpoint']['known_pose']['arrived']==rolling['known_actual_arrivals'];assert counts['rolling_endpoint']['estimated_pose']['arrived']==rolling['native_actual_arrivals']
assert learning['results']==review['personally_checked']['results_exact']
assert not learning['candidate_accepted'] and learning['classification']=='unassessed_label_coverage'
assert geometry['severity_counts']==dict(Critical=0,Important=0,Minor=0)
allowed=all(r['passed'] for r in learning['results'].values())
assert not allowed
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),records=records,success_counts=counts,learning=dict(target=design['question'],classification=learning['classification'],model_converged=learning['model_converged'],results=learning['results'],runtime_trial_permitted_by_existing_design=allowed,remaining_fit_budget=0,remaining_extra_collection_budget=0,not_a_rejection_of_all_world_models=True),geometry_diagnostic_closed=True,source_sha256=source,verified_referenced_file_sha256=checked,personally_run='CPU reconciliation and listed referenced hashes only; original episodes/fit and full historical manifests not rerun',not_pooled_success_rate=True,strict_cold_start_navigation_unestablished=True,robust_native_navigation_unestablished=True,whole_mission_complete=False)
out=H/'research_phase_reconciliation_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(success_counts=counts,verified_referenced_files=len(checked),learning_runtime_allowed=allowed),ensure_ascii=False))
