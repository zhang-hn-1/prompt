from pathlib import Path
import json,hashlib,datetime,ast
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 assert not (H/'tracking_capacity_protocol.json').exists()
 old=json.loads((H/'endpoint_navigation_protocol.json').read_text());closed=json.loads((H/'endpoint_navigation_final_identity.json').read_text());sync=json.loads((H/'endpoint_navigation_document_sync.json').read_text());inputs={**old['input_sha256'],**closed['final_artifact_sha256']}
 manifest=json.loads((H/'endpoint_navigation_run/artifact_manifest_post_close.json').read_text());inputs.update(manifest['artifact_sha256'])
 for f,d in inputs.items():assert sha(f)==d,f
 for f,d in sync['document_sha256'].items():assert sha(H/'tracking_capacity_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==d,f
 for n in ['endpoint_navigation_final_identity.json','endpoint_navigation_document_sync.json','endpoint_navigation_final_check.json']:inputs[str(H/n)]=sha(H/n)
 sources=dict(old['source_sha256'])
 for f,d in sources.items():assert sha(f)==d,f
 for f in [H/'tracking_measurement.py',H/'tracking_measurement_red_scaffold.py',H/'test_tracking_measurement.py',H/'audit_tracking_capacity.py',H/'seal_tracking_capacity.py',W/'docs/superpowers/specs/2026-10-05-tracking-capacity-audit-design.md',W/'docs/superpowers/plans/2026-10-05-tracking-capacity-audit.md']:
  if f.suffix=='.py':ast.parse(f.read_text())
  sources[str(f)]=sha(f)
 for name in ['tracking_capacity_red.log','tracking_capacity_green.log']:inputs[str(H/name)]=sha(H/name)
 assert '4 failed, 2 passed' in (H/'tracking_capacity_red.log').read_text() and '6 passed' in (H/'tracking_capacity_green.log').read_text()
 p=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,maximum_completed_audits=1,tracking_frames=[237,335],window_blocks=10,geometry_min_arc_m=.05,geometry_min_half_chord_m=.01,guard_evidence_frames=[330,335],original_clearance_m=.6,current_guard_uses_version3_before335replacement=True,policy_act_calls=0,new_physical_sequences=0,planner_searches=0,control_candidate=None,old_navigation_gate_unchanged_rejected=True,whole_research_complete=False,automation='ACTIVE')
 (H/'tracking_capacity_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print('SEALED',p['sealed_utc'],len(sources),len(inputs))
if __name__=='__main__':main()
