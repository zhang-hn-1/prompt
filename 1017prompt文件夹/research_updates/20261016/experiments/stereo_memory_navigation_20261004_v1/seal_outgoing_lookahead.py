"""Seal one archived current-algebra pass after CPU regression, before audit."""
from pathlib import Path
import hashlib,json,datetime,ast
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 old=json.loads((H/'cusp_navigation_protocol.json').read_text());fin=json.loads((H/'cusp_navigation_final_identity.json').read_text());man=json.loads((H/'cusp_navigation_run/artifact_manifest_post_close.json').read_text());docs=json.loads((H/'cusp_navigation_document_sync.json').read_text())
 sources=dict(old['source_sha256']);inputs=dict(old['input_sha256']);inputs.update(man['artifact_sha256']);inputs.update(fin['final_artifact_sha256'])
 for p,d in docs['document_sha256'].items():
  actual=H/'outgoing_lookahead_pre_stage_ledger.md' if Path(p)==H/'progress.md' else Path(p)
  assert sha(actual)==d,p
 for p,d in list(sources.items())+list(inputs.items()):assert sha(p)==d,p
 for n in ['outgoing_lookahead.py','test_outgoing_lookahead.py','audit_outgoing_lookahead.py','seal_outgoing_lookahead.py']:
  p=H/n;ast.parse(p.read_text());sources[str(p)]=sha(p)
 for p in [W/'docs/superpowers/specs/2026-10-05-outgoing-lookahead-design.md',W/'docs/superpowers/plans/2026-10-05-outgoing-lookahead.md']:sources[str(p)]=sha(p)
 for n in ['outgoing_lookahead_red_scaffold.py','outgoing_lookahead_red.log','outgoing_lookahead_green.log','outgoing_lookahead_suite.log','outgoing_lookahead_prewrite_failure.log','outgoing_lookahead_suite_environment_failure.log','outgoing_lookahead_pre_stage_ledger.md','cusp_navigation_final_identity.json','cusp_navigation_document_sync.json','cusp_navigation_final_check.json']:inputs[str(H/n)]=sha(H/n)
 assert '15 passed' in (H/'outgoing_lookahead_green.log').read_text() and '210 passed' in (H/'outgoing_lookahead_suite.log').read_text()
 d=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),maximum_archived_audit_passes=1,maximum_archived_records=350,valid_tracking_expected=188,archived_act_calls=0,new_physical_runs=0,new_planner_searches=0,sole_candidate='current outgoing-edge endpoint lookahead',old_gates_unchanged=True,actual_navigation_candidate_tested=False,whole_research_complete=False,automation='ACTIVE',source_sha256=sources,input_sha256=inputs)
 assert not (H/'outgoing_lookahead_protocol.json').exists();(H/'outgoing_lookahead_protocol.json').write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');print('SEALED',d['sealed_utc'],len(sources),len(inputs))
if __name__=='__main__':main()
