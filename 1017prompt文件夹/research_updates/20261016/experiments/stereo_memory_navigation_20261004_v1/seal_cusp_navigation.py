"""Preserve prior evidence and preregister one real original35s projection fix."""
from pathlib import Path
import json, datetime, hashlib, ast, inspect
H=Path(__file__).resolve().parent; W=H.parent.parent
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()

def main():
 assert not (H/'cusp_navigation_protocol.json').exists() and not (H/'cusp_navigation_run').exists()
 prior=json.loads((H/'cusp_projection_protocol.json').read_text());closed=json.loads((H/'cusp_projection_final_identity.json').read_text());sync=json.loads((H/'cusp_projection_document_sync.json').read_text())
 inputs={**prior['input_sha256'],**closed['final_artifact_sha256']}
 for f,s in inputs.items():assert sha(f)==s,f
 for f,s in sync['document_sha256'].items():assert sha(H/'cusp_navigation_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 for n in ['cusp_projection_final_identity.json','cusp_projection_document_sync.json','cusp_projection_final_check.json']:inputs[str(H/n)]=sha(H/n)
 sources=dict(prior['source_sha256'])
 for f,s in sources.items():assert sha(f)==s,f
 files=[H/n for n in ['run_cusp_navigation.py','run_cusp_navigation_stage.py','assess_cusp_navigation.py','verify_cusp_navigation.py','plot_cusp_navigation.py','seal_cusp_navigation.py','cusp_navigation_runtime_diff.patch']]+[W/'docs/superpowers/specs/2026-10-05-cusp-navigation-design.md',W/'docs/superpowers/plans/2026-10-05-cusp-navigation.md']
 for f in files:
  if f.suffix=='.py':ast.parse(f.read_text())
  sources[str(f)]=sha(f)
 from cusp_projection import CuspProjectionPolicy
 from root_direction_consumer import RootDirectionPolicy
 from depth_local_curve import P
 from benchmark.policies import OraclePathPolicy
 from training.action_protocol import load_action_protocol
 obj=CuspProjectionPolicy(load_action_protocol(P/'configs/data_contract.yaml'))
 assert CuspProjectionPolicy.act is RootDirectionPolicy.act and CuspProjectionPolicy.plan_boundary is RootDirectionPolicy.plan_boundary
 assert obj.tracker.act.__func__ is OraclePathPolicy.act and obj.tracker._handle_direction_change.__func__ is OraclePathPolicy._handle_direction_change
 identity=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),outer_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(obj).__mro__ if c is not object],inner_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(obj.tracker).__mro__ if c is not object],act_and_root_planning_inherited=True,inner_act_and_direction_handler_original=True,episode_reset_only_constructs_new_projection_tracker=True,sensor_privileged=True,global_reference_online=False,root_comparator_rejected=True)
 (H/'cusp_navigation_identity_audit.json').write_text(json.dumps(identity,indent=2)+'\n');sources[str(H/'cusp_navigation_identity_audit.json')]=sha(H/'cusp_navigation_identity_audit.json')
 old=json.loads((H/'root_direction_protocol.json').read_text());p={k:v for k,v in old.items() if k not in ['source_sha256','input_sha256','sealed_utc','candidate','sole_behavior_change','baseline']}
 p.update(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,candidate='unaccepted current cusp projection hold with original qualification',sole_behavior_change='current opposite waypoint cannot be skipped before accepted direction changes',baseline=str(H/'root_direction_run'),stronger_baseline=str(H/'guarded_reference_run'),same_numeric_acceptance_for_both_comparators=True,rejected_root_not_an_admitted_baseline=True,maximum_new_control_candidates=1,baseline_rerun=False,whole_research_complete=False,automation='ACTIVE')
 assert p['acceptance']==json.loads((H/'guarded_reference_protocol.json').read_text())['acceptance']
 (H/'cusp_navigation_protocol.json').write_text(json.dumps(p,indent=2)+'\n')
 print(json.dumps(dict(sealed_utc=p['sealed_utc'],sources=len(sources),prior_inputs=len(inputs),maximum_sequences=1,old_acceptance_unchanged=True),indent=2))

if __name__=='__main__':main()
