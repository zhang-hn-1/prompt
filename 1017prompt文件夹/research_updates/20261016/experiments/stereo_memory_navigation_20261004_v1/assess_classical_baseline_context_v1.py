"""Independent stored raw execution/row/context consistency and scope, no policy act replay."""
from pathlib import Path
import json,hashlib
import numpy as np
from baseline35_metrics_v1 import assess
from request_context_layer_check_v1 import layers_match
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent
D=H/'classical_baseline_context_e0_run_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if __name__=='__main__':
 manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
 for file,digest in manifest.items():assert sha(Path(file))==digest,file
 envelope=json.loads((H/'classical_baseline_repro_protocol_v1.json').read_text());contract=json.loads((D/'reproduction_contract.json').read_text());assert contract['envelope']==envelope and contract['request_context_mode']=='on'
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];identity=json.loads((D/'identity.json').read_text());status=json.loads((H/'classical_baseline_context_e0_status_v1.json').read_text())
 with np.load(D/'requests.npz') as f:requests=f['requests'].copy()
 with np.load(D/'physics.npz') as f:physics_clock=f['simulation_time_s'].copy();physics_count=len(physics_clock)
 baseline_identity=json.loads((H/'native_pose_baseline35_e0_run_v1/identity.json').read_text());config=json.loads((H/'native_pose_baseline35_e0_protocol_v1.json').read_text())
 assert episode['episode_id']==config['episode_id']=='golden_e0_0001'
 assert identity['policy_MRO'][1:]==baseline_identity['policy_MRO'] and 'current_request_context_v1' in identity['policy_MRO'][0]
 assert identity['pid_config']==config['pid_config'] and identity['inner_config']==config['inner_config']
 assert identity['camera_rig']==baseline_identity['camera_rig']
 assert contract['source_protocol_sha256']==sha(H/'native_pose_baseline35_e0_protocol_v1.json')
 assert identity['known_GT_pose_control'] is False and identity['source_GT_pose_registration'] is False
 contexts=[];prefix=0
 for i,row in enumerate(rows):
  assert np.array_equal(requests[i],np.asarray(row['request'],np.float32))
  ctx=row.get('current_request_context')
  if row['prefix_observer']:
   assert ctx is None;prefix+=1;continue
  assert row['planner_act_called'] and ctx is not None
  assert ctx['frame']==i==row['frame'] and ctx['time_s']==row['time_s']
  assert np.array_equal(np.asarray(ctx['issued_request']),np.asarray(row['request']))
  assert layers_match(ctx,row['actual_tracker_emission_audit_after_act'],protocol)
  assert ctx['applied_target'] is None and not ctx['scope'].startswith('future')
  if not ctx['geometry_computed_this_call']:assert ctx['requested_curvature_1_m'] is None
  if contexts and not ctx['records_reset_during_act']:assert ctx['emission_count']==contexts[-1]['emission_count']+1
  contexts.append(ctx)
 count=len(rows);last=float(states[-1]['time_s']);available=all(c['available'] for c in contacts)
 expected_contacts=count*5+count+1
 ticks={round(c['simulation_time_s']/.02) for c in contacts}
 coverage=len(contacts)==expected_contacts and available and ticks==set(range(count*5+1)) and all(abs(c['simulation_time_s']-.02*round(c['simulation_time_s']/.02))<1e-5 for c in contacts)
 installations=sum(bool(r.get('local_plan_created')) for r in rows)
 clocks_complete=len(states)==351 and np.allclose(np.array([s['time_s'] for s in states]),np.arange(351)*.1,rtol=0,atol=1e-5) and physics_count==1751 and np.allclose(physics_clock,np.arange(1751)*.02,rtol=0,atol=1e-5)
 gates=dict(normal_exit=status['status']=='finished' and status['child_returncode']==0,complete_35=count==350 and requests.shape==(350,10) and clocks_complete and abs(last-35.)<1e-5,complete_available_contact_zero=coverage and not any(c['contact'] for c in contacts),contexts_every_planner_call=len(contexts)==sum(bool(r['planner_act_called']) for r in rows),normal_tracking_exercised=any(c['normal_tracking_this_call'] for c in contexts),at_least_two_installs=installations>=2)
 normalized=[dict(time_s=c['simulation_time_s'],available=c['available'],collision=c['contact']) for c in contacts]
 metrics=assess(states,normalized,episode['start_position_world_m'][:2],episode['goal_position_world_m'][:2])
 out=dict(gates=gates,interface_reproduction_pass=all(gates.values()),actual_online_contexts=len(contexts),initial_prefix_rows_without_tracker_context=prefix,reference_installations=installations,immutable_per_call_clock_request_context_checked=True,raw_protocol_layers_independently_checked_against_actual_tracker_emission_and_official_normalization=True,manifest_items_verified=len(manifest),control_candidate=None,not_on_off_physics_causal_AB=True,old_baseline_results_not_modified=True,not_navigation_improvement=True,observed_conditional_navigation_result=metrics,whole_mission_complete=False,physical_runs_added=1,input_sha256={str(D/n):sha(D/n) for n in ['states.json','physics.npz','policy_records.json','requests.npz','physics_contacts.json','identity.json','artifact_manifest_post_close.json','reproduction_contract.json']},assessor_sha256=sha(Path(__file__)))
 p=H/'classical_baseline_context_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
