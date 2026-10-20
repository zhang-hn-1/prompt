"""Append-only administrative closure; no policy call or scientific recomputation."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent;REPORT=W/'reports/benchmark_outgoing_endpoint_navigation_20261005.md';HEADING='## 11:46UTC outgoing endpoint actual35s — final stage closure'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();read=lambda n:json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 p=read('endpoint_navigation_protocol.json');a=read('endpoint_navigation_assessment.json');v=read('endpoint_navigation_verification.json');r=read('endpoint_navigation_review.json');pkg=read('endpoint_navigation_review_package.json');st=read('endpoint_navigation_status.json');motion=read('endpoint_navigation_motion_audit.json')
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert st['status']=='finished' and st['exit_code']==0 and st['maximum_sequences']==1
 assert v['interface_gate_pass'] and a['navigation']['elapsed_s']<=35
 assert motion['all_acceptances_have_five_current_low']
 body=REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0];assert hashlib.sha256(body).hexdigest()==r['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 for mapping in [p['source_sha256'],p['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 manifest=read('endpoint_navigation_run/artifact_manifest_post_close.json')
 for f,d in manifest['artifact_sha256'].items():assert sha(f)==d,f
 for f,d in read('outgoing_lookahead_document_sync.json')['document_sha256'].items():assert sha(H/'endpoint_navigation_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==d,f
 assert read('endpoint_navigation_gpu_launch.json')['failed_attempt_physics_and_sensor_artifacts']==0
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 for f in docs:
  assert HEADING not in f.read_text();(H/('endpoint_navigation_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excluded={'endpoint_navigation_final_identity.json','endpoint_navigation_document_sync.json','endpoint_navigation_final_check.json'}
 files={f for f in H.glob('*endpoint_navigation*') if f.is_file() and f.name not in excluded};files.update([REPORT,W/'docs/superpowers/specs/2026-10-05-endpoint-navigation-design.md',W/'docs/superpowers/plans/2026-10-05-endpoint-navigation.md'])
 ident=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',actual_sequences=1,launch_attempts=2,failed_preacquisition_attempts=1,original_episode_budget_s=35,interface_gate_pass=v['interface_gate_pass'],limited_improvement_all_three_pass=a['limited_improvement_all_three_pass'],navigation_gate_pass=v['navigation_gate_pass'],source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),actual_artifact_checks=len(manifest['artifact_sha256']),reviewed_report_body_sha256=r['reviewed_report_sha256'],final_report_sha256=sha(REPORT),verification_sha256=sha(H/'endpoint_navigation_verification.json'),review_sha256=sha(H/'endpoint_navigation_review.json'),final_artifact_sha256={str(f):sha(f) for f in sorted(files)})
 save('endpoint_navigation_final_identity.json',ident)
 summary=(H/'endpoint_navigation_closure_text.md').read_text();assert HEADING in summary
 for f in docs:
  with f.open('a') as out:out.write('\n\n'+summary+'\n')
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[REPORT,W/'docs/superpowers/plans/2026-10-05-endpoint-navigation.md']},final_identity_sha256=sha(H/'endpoint_navigation_final_identity.json'),closure_script_sha256=sha(__file__))
 save('endpoint_navigation_document_sync.json',sync);print(json.dumps({k:v for k,v in ident.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
