"""Append-only author closure. No policy, planner, or scientific computation."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent; W=H.parent.parent
REPORT=W/'reports/benchmark_same_direction_capacity_and_guard_source_20261005.md'
HEADING='## 12:46UTC tracking capacity and guard-source audit — final stage closure'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 p=read('tracking_capacity_protocol.json');a=read('tracking_capacity_assessment.json');v=read('tracking_capacity_verification.json');r=read('tracking_capacity_review.json');pkg=read('tracking_capacity_review_package.json')
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 body=REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]
 assert hashlib.sha256(body).hexdigest()==r['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 for mapping in [p['source_sha256'],p['input_sha256'],pkg['extra_author_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 assert v['old_navigation_gate_unchanged_rejected'] and len(a['rows'])==98
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 for f in docs:
  assert HEADING not in f.read_text();(H/('tracking_capacity_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excluded={'tracking_capacity_final_identity.json','tracking_capacity_document_sync.json','tracking_capacity_final_check.json'}
 files={f for f in H.glob('*tracking_capacity*') if f.is_file() and f.name not in excluded}
 files.update(H.glob('*tracking_measurement*.py'))
 files.update([REPORT,W/'docs/superpowers/specs/2026-10-05-tracking-capacity-audit-design.md',W/'docs/superpowers/plans/2026-10-05-tracking-capacity-audit.md'])
 ident=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',completed_cpu_audits=1,new_physical_sequences=0,planner_searches=0,policy_act_calls=0,fits=0,control_candidate=None,old_navigation_gate_unchanged_rejected=True,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),review_context=r['review_context'],reviewed_report_body_sha256=r['reviewed_report_sha256'],final_report_sha256=sha(REPORT),verification_sha256=sha(H/'tracking_capacity_verification.json'),review_sha256=sha(H/'tracking_capacity_review.json'),final_artifact_sha256={str(f):sha(f) for f in sorted(files)})
 save('tracking_capacity_final_identity.json',ident)
 summary=(H/'tracking_capacity_closure_text.md').read_text();assert HEADING in summary
 for f in docs:
  with f.open('a') as out:out.write('\n\n'+summary+'\n')
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[REPORT,W/'docs/superpowers/plans/2026-10-05-tracking-capacity-audit.md']},final_identity_sha256=sha(H/'tracking_capacity_final_identity.json'),closure_script_sha256=sha(__file__))
 save('tracking_capacity_document_sync.json',sync)
 print(json.dumps({k:v for k,v in ident.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
