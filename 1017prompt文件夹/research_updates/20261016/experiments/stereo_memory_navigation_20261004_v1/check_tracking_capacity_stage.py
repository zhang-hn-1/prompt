"""Fresh administrative author verification; no scientific replay."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def main():
 p=read('tracking_capacity_protocol.json');a=read('tracking_capacity_assessment.json');v=read('tracking_capacity_verification.json');r=read('tracking_capacity_review.json');f=read('tracking_capacity_final_identity.json');s=read('tracking_capacity_document_sync.json')
 checks={}
 for n,m in [('sealed_sources',p['source_sha256']),('old_actual_inputs',p['input_sha256']),('stage_artifacts',f['final_artifact_sha256']),('synchronized_documents',s['document_sha256'])]:
  for path,d in m.items():assert sha(path)==d,(n,path)
  checks[n]=len(m)
 assert sha(H/'tracking_capacity_final_identity.json')==s['final_identity_sha256']
 assert sha(H/'close_tracking_capacity_stage.py')==s['closure_script_sha256']
 assert sha(H/'tracking_capacity_review.json')==f['review_sha256']
 assert sha(H/'tracking_capacity_verification.json')==f['verification_sha256']
 report=W/'reports/benchmark_same_direction_capacity_and_guard_source_20261005.md'
 assert sha(report)==f['final_report_sha256']
 assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest()==r['reviewed_report_sha256']==f['reviewed_report_body_sha256']
 for path in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
  assert path.read_bytes().startswith((H/('tracking_capacity_pre_close_'+path.name)).read_bytes())
  assert path.read_text().count('## 12:46UTC tracking capacity and guard-source audit — final stage closure')==1
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert p['sealed_utc']<a['started_utc']<a['completed_utc']<v['verified_utc']<r['saved_utc']<f['closed_utc']<s['synchronized_utc']
 assert all(not d['whole_research_complete'] for d in [p,a,v,r,f,s])
 assert f['automation']==s['automation']=='ACTIVE' and f['completed_cpu_audits']==1
 assert f['new_physical_sequences']==f['planner_searches']==f['policy_act_calls']==f['fits']==0
 assert f['old_navigation_gate_unchanged_rejected'] and f['control_candidate'] is None
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,reviewed_prefix_exact=True,author_documents_append_only=True,review_count=1,review_context=r['review_context'],scope_rulings=len(r['declined_scope_rulings']),completed_cpu_audits=1,new_physical_sequences=0,control_candidate=None,old_navigation_gate_unchanged_rejected=True,stage_complete=True,whole_research_complete=False,automation='ACTIVE',final_identity_sha256=sha(H/'tracking_capacity_final_identity.json'),document_sync_sha256=sha(H/'tracking_capacity_document_sync.json'))
 (H/'tracking_capacity_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
