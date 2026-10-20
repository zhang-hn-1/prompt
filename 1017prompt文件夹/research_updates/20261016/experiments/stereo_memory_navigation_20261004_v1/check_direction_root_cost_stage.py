"""Final identity/doc/gate check, no searches or scientific reruns."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def read(n):return json.loads((H/n).read_text())
def main():
 p=read('direction_root_cost_protocol.json');f=read('direction_root_cost_final_identity.json');s=read('direction_root_cost_document_sync.json');r=read('direction_root_cost_review.json');v=read('direction_root_cost_verification.json');a=read('direction_root_cost_assessment.json');st=read('direction_root_cost_status.json')
 checks={}
 for name,d in [('sealed_sources',p['source_sha256']),('old_inputs',p['input_sha256']),('final_stage_artifacts',f['final_artifact_sha256']),('synchronized_documents',s['document_sha256'])]:
  for path,digest in d.items():assert sha(path)==digest,(name,path)
  checks[name]=len(d)
 assert sha(H/'direction_root_cost_final_identity.json')==s['final_identity_sha256']
 assert sha(H/'direction_root_cost_verification.json')==s['author_verification_sha256']==f['author_verification_sha256']
 assert sha(H/'direction_root_cost_review.json')==s['review_sha256']==f['review_sha256']
 assert sha(H/'close_direction_root_cost_stage.py')==s['closure_script_sha256']
 assert sha(H/'direction_root_cost_assessment.json')==v['assessment_sha256']
 report=W/'reports/benchmark_direction_root_cost_audit_20261005.md';assert sha(report)==f['final_report_sha256']
 assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest()==r['reviewed_report_sha256']==f['reviewed_report_body_sha256']
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 for doc in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
  assert doc.read_bytes().startswith((H/('direction_root_cost_pre_close_'+doc.name)).read_bytes())
  assert doc.read_text().count('## 06:44UTC current-direction root cost — final stage closure')==1
 assert st['status']=='finished' and st['exit_code']==0 and st['actual_grid_searches']==a['actual_grid_searches']==f['actual_grid_searches']==1 and st['actual_hybrid_searches']==a['actual_hybrid_searches']==f['actual_hybrid_searches']==2
 assert p['sealed_utc']<st['started_utc']<a['saved_utc']<=st['ended_utc']<v['verified_utc']<r['saved_utc']<f['closed_utc']<s['synchronized_utc']
 assert v['interface_gate_pass'] and v['nominal_behavior_eligibility_pass'] and not v['new_physical_candidate_tested'] and not v['braking_switch_elimination_demonstrated']
 assert v['geometry']['baseline']['total_direction_changes_including_initial']==v['geometry']['candidate']['total_direction_changes_including_initial']==2
 assert all(not d['whole_research_complete'] for d in [p,v,a,r,f,s]) and f['automation']==s['automation']=='ACTIVE' and f['new_simulations']==0
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,reviewed_report_prefix_exact=True,author_document_append_only=True,review_count=1,scope_rulings=len(r['declined_scope_rulings']),actual_archived_experiment_finished_exit_zero=True,actual_grid_searches=1,actual_hybrid_searches=2,new_simulations=0,interface_gate_pass=True,nominal_behavior_eligibility_pass=True,braking_switch_elimination_demonstrated=False,physical_navigation_candidate_tested=False,stage_complete=True,whole_research_complete=False,automation='ACTIVE',final_identity_sha256=sha(H/'direction_root_cost_final_identity.json'),document_sync_sha256=sha(H/'direction_root_cost_document_sync.json'),scope='Fresh closure hashes/documents/gates only, no further searches or scientific reruns.')
 (H/'direction_root_cost_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
