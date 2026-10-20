"""Immutable preregistration and source identity before one physical candidate."""
from pathlib import Path
import json,hashlib,datetime,ast,inspect
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 assert not (H/'root_direction_protocol.json').exists() and not (H/'root_direction_run').exists()
 prior=json.loads((H/'direction_root_cost_protocol.json').read_text());closed=json.loads((H/'direction_root_cost_final_identity.json').read_text());sync=json.loads((H/'direction_root_cost_document_sync.json').read_text())
 inputs={**prior['input_sha256'],**closed['final_artifact_sha256']}
 for f,s in inputs.items():assert sha(f)==s,f
 for f,s in sync['document_sha256'].items():assert sha(H/'root_direction_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 for f in ['direction_root_cost_final_identity.json','direction_root_cost_document_sync.json','direction_root_cost_final_check.json']:inputs[str(H/f)]=sha(H/f)
 sources=dict(prior['source_sha256'])
 for f,s in sources.items():assert sha(f)==s,f
 additions=[H/f for f in ['root_direction_consumer.py','test_root_direction_consumer.py','run_root_direction.py','run_root_direction_stage.py','assess_root_direction.py','verify_root_direction.py','root_direction_runtime_diff.patch','root_direction_local_curve_diff.patch','seal_root_direction.py']]+[W/'docs/superpowers/specs/2026-10-05-root-direction-online-design.md',W/'docs/superpowers/plans/2026-10-05-root-direction-online.md']
 for f in additions:
  if f.suffix=='.py':ast.parse(f.read_text())
  sources[str(f)]=sha(f)
 from root_direction_consumer import RootDirectionPolicy
 from guarded_reference import GuardedReferencePolicy
 from depth_local_curve import P
 from training.action_protocol import load_action_protocol
 p=RootDirectionPolicy(load_action_protocol(P/'configs/data_contract.yaml'))
 assert RootDirectionPolicy.act is GuardedReferencePolicy.act and RootDirectionPolicy.reset is GuardedReferencePolicy.reset
 identity=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),outer_class=str(type(p)),outer_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(p).__mro__ if c is not object],inner_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(p.tracker).__mro__ if c is not object],act_exact_guarded_baseline=True,reset_exact_guarded_baseline=True,initial_uninstalled_latch_cost_input=None,sensor_privileged=True,global_reference_online=False,original_tracker_unchanged=True,original_objective_summary_not_consumed=True)
 (H/'root_direction_identity_audit.json').write_text(json.dumps(identity,indent=2)+'\n');sources[str(H/'root_direction_identity_audit.json')]=sha(H/'root_direction_identity_audit.json')
 old=json.loads((H/'guarded_reference_protocol.json').read_text())
 protocol={k:v for k,v in old.items() if k not in ['source_sha256','input_sha256','sealed_utc','candidate','sole_behavior_change','baseline']}
 protocol.update(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,candidate='current accepted direction root cost at original guarded rebuild',sole_behavior_change='initial direction consumed by frozen independent root cost planner',baseline=str(H/'guarded_reference_run'),initial_direction_rules=dict(before_first_install=None,at_rebuild='pre-planning accepted tracker latch only',on_retention=None,both_roots_allowed=True,opposite_root_cost_m=.5,reverse_multiplier=1.25),maximum_new_control_candidates=1,baseline_rerun=False,whole_research_complete=False,automation='ACTIVE')
 (H/'root_direction_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
 print(json.dumps(dict(sealed_utc=protocol['sealed_utc'],sources=len(sources),prior_inputs=len(inputs),maximum_sequences=1,old_acceptance_unchanged=protocol['acceptance']==old['acceptance']),indent=2))
if __name__=='__main__':main()
