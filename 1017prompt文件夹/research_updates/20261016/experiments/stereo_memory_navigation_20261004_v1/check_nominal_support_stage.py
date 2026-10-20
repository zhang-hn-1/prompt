"""Fresh author administrative hashes and append checks; no search replay."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def main():
 p=read('nominal_support_protocol.json');a=read('nominal_support_assessment.json');v=read('nominal_support_verification.json');r=read('nominal_support_review.json');f=read('nominal_support_final_identity.json');s=read('nominal_support_document_sync.json')
 checks={}
 for n,m in [('sealed_sources',p['source_sha256']),('old_actual_inputs',p['input_sha256']),('stage_artifacts',f['final_artifact_sha256']),('synchronized_documents',s['document_sha256']),('definition_sources',read('nominal_support_definition_audit.json')['source_sha256'])]:
  for path,d in m.items():assert sha(path)==d,(n,path)
  checks[n]=len(m)
 assert sha(H/'nominal_support_final_identity.json')==s['final_identity_sha256']
 assert sha(H/'close_nominal_support_stage.py')==s['closure_script_sha256']
 assert sha(H/'nominal_support_review.json')==f['review_sha256']
 assert sha(H/'nominal_support_verification.json')==f['verification_sha256']
 report=W/'reports/benchmark_nominal_support_and_planner_consistency_20261005.md'
 assert sha(report)==f['final_report_sha256']
 assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest()==r['reviewed_report_sha256']==f['reviewed_report_body_sha256']
 for path in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
  assert path.read_bytes().startswith((H/('nominal_support_pre_close_'+path.name)).read_bytes())
  assert path.read_text().count('## 13:47UTC nominal support CPU comparison — final stage closure')==1
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert p['sealed_utc']<a['started_utc']<a['completed_utc']<v['verified_utc']<r['saved_utc']<f['closed_utc']<s['synchronized_utc']
 assert all(not d['whole_research_complete'] for d in [p,a,v,r,f,s])
 assert f['automation']==s['automation']=='ACTIVE' and f['completed_cpu_comparisons']==1
 assert f['hybrid_searches']==2 and f['shared_grid_routes']==1 and f['candidate_count']==1
 assert f['new_physical_sequences']==f['policy_act_calls']==f['fits']==0
 assert not f['cpu_geometry_eligibility_pass'] and not f['navigation_admitted'] and not f['physical_model_admitted']
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,reviewed_prefix_exact=True,author_documents_append_only=True,review_count=1,review_context=r['review_context'],scope_rulings=len(r['declined_scope_rulings']),completed_cpu_comparisons=1,hybrid_searches=2,shared_grid_routes=1,new_physical_sequences=0,cpu_geometry_eligibility_pass=False,stage_complete=True,whole_research_complete=False,automation='ACTIVE',final_identity_sha256=sha(H/'nominal_support_final_identity.json'),document_sync_sha256=sha(H/'nominal_support_document_sync.json'))
 (H/'nominal_support_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
