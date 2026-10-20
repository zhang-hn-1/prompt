from pathlib import Path
import hashlib,json,datetime,ast,inspect
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 assert not (H/'endpoint_navigation_protocol.json').exists() and not (H/'endpoint_navigation_run').exists()
 prior=json.loads((H/'outgoing_lookahead_protocol_v2.json').read_text());closed=json.loads((H/'outgoing_lookahead_final_identity.json').read_text());sync=json.loads((H/'outgoing_lookahead_document_sync.json').read_text())
 inputs={**prior['input_sha256'],**closed['final_artifact_sha256']}
 for f,s in inputs.items():assert sha(f)==s,f
 for f,s in sync['document_sha256'].items():assert sha(H/'endpoint_navigation_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 for name in ['outgoing_lookahead_final_identity.json','outgoing_lookahead_document_sync.json','outgoing_lookahead_final_check.json']:inputs[str(H/name)]=sha(H/name)
 sources=dict(prior['source_sha256'])
 for f,s in sources.items():assert sha(f)==s,f
 newfiles=[H/n for n in ['run_endpoint_navigation.py','run_endpoint_navigation_stage.py','assess_endpoint_navigation.py','verify_endpoint_navigation.py','audit_endpoint_navigation_motion.py','plot_endpoint_navigation.py','seal_endpoint_navigation.py','endpoint_navigation_runtime_diff.patch']]+[W/'docs/superpowers/specs/2026-10-05-endpoint-navigation-design.md',W/'docs/superpowers/plans/2026-10-05-endpoint-navigation.md']
 for f in newfiles:
  if f.suffix=='.py':ast.parse(f.read_text())
  sources[str(f)]=sha(f)
 assert '15 passed' in (H/'endpoint_navigation_regression.log').read_text();inputs[str(H/'endpoint_navigation_regression.log')]=sha(H/'endpoint_navigation_regression.log')
 from outgoing_lookahead import EndpointLookaheadPolicy
 from cusp_projection import CuspProjectionPolicy
 from root_direction_consumer import RootDirectionPolicy
 from depth_local_curve import P
 from benchmark.policies import OraclePathPolicy
 from training.action_protocol import load_action_protocol
 policy=EndpointLookaheadPolicy(load_action_protocol(P/'configs/data_contract.yaml'))
 assert EndpointLookaheadPolicy.act is RootDirectionPolicy.act and EndpointLookaheadPolicy.plan_boundary is RootDirectionPolicy.plan_boundary
 assert policy.tracker.act.__func__ is OraclePathPolicy.act and policy.tracker._handle_direction_change.__func__ is OraclePathPolicy._handle_direction_change
 identity=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),outer_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(policy).__mro__ if c is not object],inner_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(policy.tracker).__mro__ if c is not object],act_and_root_planning_inherited=True,inner_act_handler_original=True,sensor_privileged=True,global_reference_online=False)
 (H/'endpoint_navigation_identity_audit.json').write_text(json.dumps(identity,indent=2)+'\n');sources[str(H/'endpoint_navigation_identity_audit.json')]=sha(H/'endpoint_navigation_identity_audit.json')
 old=json.loads((H/'cusp_navigation_protocol.json').read_text());p={k:v for k,v in old.items() if k not in ['source_sha256','input_sha256','sealed_utc','candidate','sole_behavior_change','baseline']}
 p.update(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,candidate='EndpointLookaheadPolicy',sole_behavior_change='lookahead consumes current outgoing edge to its endpoint before checking next outgoing direction',baseline=str(H/'cusp_navigation_run'),stronger_baseline=str(H/'guarded_reference_run'),additional_rejected_comparator=str(H/'root_direction_run'),maximum_sequences=1,maximum_new_control_candidates=1,baseline_rerun=False,whole_research_complete=False,automation='ACTIVE')
 assert p['acceptance']==json.loads((H/'guarded_reference_protocol.json').read_text())['acceptance']
 (H/'endpoint_navigation_protocol.json').write_text(json.dumps(p,indent=2)+'\n')
 print(json.dumps(dict(sealed_utc=p['sealed_utc'],sources=len(sources),prior_inputs=len(inputs),maximum_sequences=1,old_gates_exact=True),indent=2))
if __name__=='__main__':main()
