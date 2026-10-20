"""Preseal one candidate/current snapshot and original acceptance; no search."""
from pathlib import Path
import datetime,json,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
if __name__=='__main__':
 assert not (H/'nominal_support_protocol.json').exists() and not (H/'nominal_support_assessment.json').exists()
 assert '11 passed' in (H/'nominal_support_green.log').read_text() and '227 passed' in (H/'nominal_support_full_suite.log').read_text()
 old=json.loads((H/'tracking_capacity_protocol.json').read_text());final=json.loads((H/'tracking_capacity_final_identity.json').read_text());sync=json.loads((H/'tracking_capacity_document_sync.json').read_text())
 sources=old['source_sha256'].copy()
 for name in ['nominal_support.py','test_nominal_support.py','nominal_support_red_scaffold.py','prepare_nominal_support.py','audit_nominal_support.py','seal_nominal_support.py']:sources[str(H/name)]=sha(H/name)
 for path in [W/'docs/superpowers/specs/2026-10-05-nominal-support-design.md',W/'docs/superpowers/plans/2026-10-05-nominal-support.md']:sources[str(path)]=sha(path)
 inputs={**old['input_sha256'],**final['final_artifact_sha256']}
 for path,d in sync['document_sha256'].items():
  actual=H/'nominal_support_pre_stage_ledger.md' if Path(path)==H/'progress.md' else Path(path)
  assert sha(actual)==d;inputs[str(actual)]=d
 for name in ['nominal_support_current175.json','nominal_support_execution175.json','nominal_support_red.log','nominal_support_green.log','nominal_support_full_suite.log','nominal_support_prewrite_failure.log','tracking_capacity_final_identity.json','tracking_capacity_document_sync.json','tracking_capacity_final_check.json']:inputs[str(H/name)]=sha(H/name)
 for group in [sources,inputs]:
  for path,d in group.items():assert sha(path)==d,path
 protocol=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,consumed_runtime_evidence=['nominal_support_current175.json','endpoint_navigation_run/maps/frame_175.npz','endpoint_navigation_run/local_plans/frame_175.npz'],comparison_reference_not_physical_label=True,current_frame=175,maximum_completed_comparisons=1,maximum_hybrid_searches=2,shared_grid_route_constructions=1,candidate_count=1,policy_act_calls=0,new_physical_sequences=0,fits=0,candidate='planner-only V1 curvature clipped to intersection of BOTH BankMap low/high directional supports',request_frozen=dict(node_budget=3000,primitive_length_m=.3,spacing_m=.05,reverse_cost_multiplier=1.25,direction_change_cost_m=.5,local_target_arc_m=2.4,clearance_m=.6),acceptance=dict(original_reference_exact_atol=1e-10,local_goal_m=.2,max_extra_path_and_objective_m=.3,max_extra_direction_changes=0,nominal_unsupported_primitives=0,map_clamped_queries=0,raster_occupied_primitives=0,strictly_reduce_selected_unsupported_count=True),physical_model_admitted=False,navigation_admitted=False,whole_research_complete=False,automation='ACTIVE')
 (H/'nominal_support_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');print('SEALED',protocol['sealed_utc'],len(sources),len(inputs))
