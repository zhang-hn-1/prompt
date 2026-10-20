from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parents[1];sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (H/'time_cost_protocol.json').exists()
old=json.loads((H/'nominal_support_protocol.json').read_text());aliases=json.loads((H/'nominal_support_closure_v2_scope.json').read_text())['archival_input_aliases']
sources=dict(old['source_sha256']);inputs={aliases.get(f,f):d for f,d in old['input_sha256'].items()}
for mapping in [sources,inputs]:
 for f,d in mapping.items():assert sha(f)==d,f
for name in ['nominal_time_cost.py','time_cost_planning.py','audit_time_cost.py','plot_time_cost.py','seal_time_cost.py','test_time_cost.py','time_cost_red_scaffold.py']:
 sources[str(H/name)]=sha(H/name)
old_dir=H.parent/'oracle_feasible_route_20260929_v9/hybrid'
for name in ['time_planning.py','hybrid_planning.py','run_navigation_worker.py']:
 sources[str(old_dir/name)]=sha(old_dir/name)
for path in [W/'docs/superpowers/specs/2026-10-05-time-cost-design.md',W/'docs/superpowers/plans/2026-10-05-time-cost.md',H/'time_cost_red.log',H/'time_cost_green.log',H/'time_cost_suite.log',H/'time_cost_planner.diff',H/'nominal_support_assessment.json',H/'nominal_support_final_identity_v2.json',H/'nominal_support_document_sync_v2.json',W/'reports/benchmark_nominal_support_and_planner_consistency_20261005.md']:
 inputs[str(path)]=sha(path)
pre_docs={}
for path in [W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']:
 copy=H/('time_cost_pre_seal_'+path.name);assert not copy.exists();copy.write_bytes(path.read_bytes());inputs[str(copy)]=sha(copy);pre_docs[str(path)]=str(copy)
p=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,current_document_archives=pre_docs,prior_archival_aliases=aliases,consumed_runtime_evidence=['nominal_support_current175.json','endpoint_navigation_run/maps/frame_175.npz'],known_reference_comparator=['nominal_support_assessment.json','endpoint_navigation_run/local_plans/frame_175.npz'],future_physical_labels_used=False,current_frame=175,max_new_target_hybrid_searches=1,new_original_target_searches=0,shared_grid_route_constructions=1,candidate_count=1,new_physical_sequences=0,policy_act_calls=0,model_fits=0,acceptance=dict(goal_m=.2,max_extra_path_and_original_objective_m=.3,max_extra_internal_switches=0,unsupported_common_domain_primitives=0,raster_occupied_or_outside_primitives=0,strict_domain_improvement=True,weighted_time_proxy_min_gain_s=.1),candidate='ONLY normalized V1-speed base edge cost; original geometry/heuristic/types/root/switch/reverse unchanged',fixture_searches_separate_from_target=True,whole_research_complete=False,automation='ACTIVE')
(H/'time_cost_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print(json.dumps({k:v for k,v in p.items() if k not in ['source_sha256','input_sha256']},indent=2));print('sealed',len(sources),len(inputs))
