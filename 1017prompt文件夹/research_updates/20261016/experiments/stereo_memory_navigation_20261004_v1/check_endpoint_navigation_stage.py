"""Fresh author checksum/append/chronology/gates verification; no science."""
from pathlib import Path
import datetime,hashlib,json
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def main():
 p=read('endpoint_navigation_protocol.json');a=read('endpoint_navigation_assessment.json');v=read('endpoint_navigation_verification.json');r=read('endpoint_navigation_review.json');f=read('endpoint_navigation_final_identity.json');s=read('endpoint_navigation_document_sync.json');st=read('endpoint_navigation_status.json');m=read('endpoint_navigation_run/artifact_manifest_post_close.json')
 checks={}
 for n,mapping in [('sealed_sources',p['source_sha256']),('previous_inputs',p['input_sha256']),('actual_post_close',m['artifact_sha256']),('final_stage_artifacts',f['final_artifact_sha256']),('synchronized_documents',s['document_sha256'])]:
  for path,d in mapping.items():assert sha(path)==d,(n,path)
  checks[n]=len(mapping)
 assert sha(H/'endpoint_navigation_final_identity.json')==s['final_identity_sha256']
 assert sha(H/'close_endpoint_navigation_stage.py')==s['closure_script_sha256']
 assert sha(H/'endpoint_navigation_review.json')==f['review_sha256']
 assert sha(H/'endpoint_navigation_verification.json')==f['verification_sha256']
 report=W/'reports/benchmark_outgoing_endpoint_navigation_20261005.md'
 assert sha(report)==f['final_report_sha256']
 assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest()==r['reviewed_report_sha256']==f['reviewed_report_body_sha256']
 for path in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
  assert path.read_bytes().startswith((H/('endpoint_navigation_pre_close_'+path.name)).read_bytes())
  assert path.read_text().count('## 11:46UTC outgoing endpoint actual35s — final stage closure')==1
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert p['sealed_utc']<st['start_utc']<st['end_utc']<a['assessed_utc']<v['verified_utc']<r['saved_utc']<f['closed_utc']<s['synchronized_utc']
 assert p['maximum_sequences']==st['maximum_sequences']==f['actual_sequences']==1 and p['episode_budget_s']==35
 assert v['interface_gate_pass'] and f['limited_improvement_all_three_pass']==a['limited_improvement_all_three_pass'] and f['navigation_gate_pass']==v['navigation_gate_pass']
 assert read('endpoint_navigation_gpu_launch.json')['failed_attempt_physics_and_sensor_artifacts']==0
 assert read('endpoint_navigation_preinit_status.json')['exit_code']==-11
 assert all(not d['whole_research_complete'] for d in [p,a,v,r,f,s])
 assert f['automation']==s['automation']=='ACTIVE'
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,reviewed_prefix_exact=True,author_documents_append_only=True,review_count=1,scope_rulings=len(r['declined_scope_rulings']),actual_sequences=1,original_budget_s=35,interface_gate_pass=True,limited_improvement_all_three_pass=a['limited_improvement_all_three_pass'],navigation_gate_pass=v['navigation_gate_pass'],stage_complete=True,whole_research_complete=False,automation='ACTIVE',final_identity_sha256=sha(H/'endpoint_navigation_final_identity.json'),document_sync_sha256=sha(H/'endpoint_navigation_document_sync.json'))
 (H/'endpoint_navigation_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
