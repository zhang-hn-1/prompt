"""Fresh author closure checks only; no scientific replay."""
from pathlib import Path
import json,datetime,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();read=lambda n:json.loads((H/n).read_text())
def main():
 p=read('outgoing_lookahead_protocol_v2.json');a=read('outgoing_lookahead_assessment.json');v=read('outgoing_lookahead_verification.json');r=read('outgoing_lookahead_review.json');f=read('outgoing_lookahead_final_identity.json');s=read('outgoing_lookahead_document_sync.json');checks={}
 for n,mp in [('sealed_sources',p['source_sha256']),('old_and_actual_inputs',p['input_sha256']),('final_stage_artifacts',f['final_artifact_sha256']),('synchronized_documents',s['document_sha256'])]:
  for q,d in mp.items():assert sha(q)==d,(n,q)
  checks[n]=len(mp)
 assert sha(H/'outgoing_lookahead_final_identity.json')==s['final_identity_sha256'];assert sha(H/'close_outgoing_lookahead_stage.py')==s['closure_script_sha256']
 assert sha(H/'outgoing_lookahead_assessment.json')==v['assessment_sha256'];assert sha(H/'outgoing_lookahead_protocol_v2.json')==v['protocol_v2_sha256']
 assert sha(H/'outgoing_lookahead_review.json')==f['review_sha256'];assert sha(H/'outgoing_lookahead_verification.json')==f['verification_sha256']
 report=W/'reports/benchmark_outgoing_lookahead_and_heading_chain_20261005.md';assert sha(report)==f['final_report_sha256']
 assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest()==r['reviewed_report_sha256']==f['reviewed_report_body_sha256']
 for q in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
  assert q.read_bytes().startswith((H/('outgoing_lookahead_pre_close_'+q.name)).read_bytes());assert q.read_text().count('## 10:45UTC outgoing-edge lookahead CPU audit — final stage closure')==1
 assert p['sealed_utc']<a['started_utc']<a['assessed_utc']<v['verified_utc']<r['saved_utc']<f['closed_utc']<s['synchronized_utc']
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert a['archived_policy_act_calls']==a['new_physical_runs']==a['new_planner_searches']==0 and a['interface_regression_pass']
 assert not a['actual_navigation_candidate_tested'] and f['total_archive_attempts']==2 and f['completed_archive_passes']==1
 assert all(not z['whole_research_complete'] for z in [p,a,v,r,f,s]);assert f['automation']==s['automation']=='ACTIVE'
 d=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,reviewed_report_prefix_exact=True,author_documents_append_only=True,review_count=1,scope_rulings=len(r['declined_scope_rulings']),total_archive_attempts=2,completed_archive_passes=1,new_physical_runs=0,new_planner_searches=0,archived_policy_act_calls=0,cpu_query_regression_pass=True,actual_navigation_candidate_tested=False,old_navigation_gate_unchanged_rejected=True,stage_complete=True,whole_research_complete=False,automation='ACTIVE',final_identity_sha256=sha(H/'outgoing_lookahead_final_identity.json'),document_sync_sha256=sha(H/'outgoing_lookahead_document_sync.json'))
 (H/'outgoing_lookahead_final_check.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d,indent=2))
if __name__=='__main__':main()
