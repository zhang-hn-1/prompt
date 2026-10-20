from pathlib import Path
import json,hashlib,datetime,ast,inspect
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 assert not (H/'guarded_reference_protocol.json').exists()
 old=json.loads((H/'depth_local_curve_protocol.json').read_text())
 prior=json.loads((H/'reference_continuity_protocol.json').read_text());closed=json.loads((H/'reference_continuity_final_identity.json').read_text());sync=json.loads((H/'reference_continuity_document_sync.json').read_text())
 inputs={**prior['input_sha256'],**closed['final_artifact_sha256']}
 for f,s in inputs.items():assert sha(f)==s,f
 for f,s in sync['document_sha256'].items():
  assert sha(H/'guarded_reference_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 for f in ['reference_continuity_final_identity.json','reference_continuity_document_sync.json','reference_continuity_final_check.json']:inputs[str(H/f)]=sha(H/f)
 sources=dict(prior['source_sha256'])
 additions=[H/f for f in ['guarded_reference.py','test_guarded_reference.py','run_guarded_reference.py','run_guarded_reference_stage.py','guarded_reference_runtime_diff.patch','seal_guarded_reference.py']]+[W/'docs/superpowers/specs/2026-10-05-guarded-reference-retention-design.md',W/'docs/superpowers/plans/2026-10-05-guarded-reference-retention.md']
 for f,s in sources.items():assert sha(f)==s,f
 for f in additions:
  if f.suffix=='.py':ast.parse(f.read_text())
  sources[str(f)]=sha(f)
 from guarded_reference import GuardedReferencePolicy
 from depth_local_curve import P
 from training.action_protocol import load_action_protocol
 p=GuardedReferencePolicy(load_action_protocol(P/'configs/data_contract.yaml'))
 identity=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),outer_class=str(type(p)),outer_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(p).__mro__ if c is not object],inner_mro=[dict(name=c.__qualname__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(p.tracker).__mro__ if c is not object],sensor_privileged=True,global_reference_online=False,controller_state_reset_on_retention=False,original_tracker_unchanged=True)
 (H/'guarded_reference_identity_audit.json').write_text(json.dumps(identity,indent=2)+'\n');sources[str(H/'guarded_reference_identity_audit.json')]=sha(H/'guarded_reference_identity_audit.json')
 protocol={k:v for k,v in old.items() if k not in ['source_sha256','sealed_utc','acceptance']}
 protocol.update(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,input_sha256=inputs,candidate='conditional installed-reference retention at original0.5s boundary',sole_behavior_change='reference replacement timing',guard=dict(observed_hazard_clearance_m=.6,spacing_m=.05,remaining_arc_min_m=.5,projection_distance_max_m=.5,both_continuous_and_original_raster=True,checks_only_at_planning_boundary=True,unknown_certified_free=False),baseline=str(H/'depth_local_curve_run'),acceptance=dict(interface='exact causal replay; physical/request/full execution prefix to first action divergence; actual applied-action change',navigation='actual original35s goalentry<=.2m and no contact',limited_improvement=dict(reference_progress_gain_min_m=.05,goal_distance_decrease_min_m=.05,lateral_rms_nonregression_tolerance_m=1e-5,lateral_peak_nonregression_tolerance_m=1e-5,no_contact=True),descriptive_checkpoints=dict(reference_arcs_m=[2.,4.,6.,8.],euclidean_radius_m=.3,offline_only=True)),whole_sequence_sealed_before_acquisition=True,fresh_environment_generalization=False,maximum_new_control_candidates=1,baseline_rerun=False)
 (H/'guarded_reference_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
 print(json.dumps(dict(sealed_utc=protocol['sealed_utc'],sources=len(sources),prior_inputs=len(inputs),candidate=protocol['candidate'],maximum_sequences=1,old_gates_changed=False),indent=2))
if __name__=='__main__':main()
