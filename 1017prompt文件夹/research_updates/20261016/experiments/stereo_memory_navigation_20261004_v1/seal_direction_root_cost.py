"""Seal one current-state archived problem before any actual scene search."""
from pathlib import Path
import datetime,json,hashlib,difflib,math
import numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent;O=H/'guarded_reference_run'
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 assert not (H/'direction_root_cost_protocol.json').exists()
 p=json.loads((H/'guarded_reference_protocol.json').read_text());f=json.loads((H/'guarded_reference_final_identity.json').read_text());s=json.loads((H/'guarded_reference_document_sync.json').read_text())
 for d in [p['source_sha256'],p['input_sha256'],f['final_artifact_sha256']]:
  for path,digest in d.items():assert sha(path)==digest,path
 for path,digest in s['document_sha256'].items():assert sha(H/'direction_root_cost_pre_stage_ledger.md' if Path(path)==H/'progress.md' else path)==digest,path
 i=175;record=json.loads((O/'policy_records.json').read_text())[i];state=record['local']['reference_decision']['state_before'];accepted=state['active_direction'];assert accepted=='forward' and state['pending_direction'] is None
 raw=np.load(O/'maps'/f'frame_{i:03d}.npz');inp=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz');a=np.load(O/'episode_arrays.npz')
 np.testing.assert_equal(inp['position_world_xy'],a['positions_world_xy'][i]);assert float(raw['time_s'])==record['local']['time_s']
 payload={k:raw[k] for k in raw.files}
 payload.update(frame=i,current_time_s=float(raw['time_s']),position_world_xy=inp['position_world_xy'],yaw_world_rad=inp['yaw_world_rad'],pointgoal_base_xy=inp['pointgoal_base_xy'],accepted_direction=accepted)
 np.savez_compressed(H/'direction_root_cost_snapshot.npz',**payload)
 e=np.load(O/'local_plans'/f'frame_{i:03d}.npz');np.savez_compressed(H/'direction_root_cost_expected_baseline.npz',**{k:e[k] for k in e.files})
 xyv=a['linear_velocity_world_m_s'][i,:2];yaw=float(inp['yaw_world_rad']);course=math.atan2(xyv[1],xyv[0]);main=float(a['joint_velocity_rad_s'][i,0]);applied=a['applied_action'][i]
 provenance=dict(frame=i,time_s=float(raw['time_s']),accepted_direction=accepted,accepted_state_before=state,accepted_previous_record=int(a['policy_active_drive_direction'][i-1]),current_emitted_request_after_planning=record['request'],current_desired_after_planning=record['diagnostics'].get('desired_drive_direction'),current_applied_at_capture=applied.tolist(),current_main_joint_velocity_rad_s=main,current_xy_velocity_world_m_s=xyv.tolist(),current_xy_course_rad=course,current_longitudinal_velocity_relative_root_yaw_m_s=float(xyv@np.array([math.cos(yaw),math.sin(yaw)])),mapping_privileged=True,no_future_label=True,expected_saved_meta={k:v for k,v in record['local']['plan'].items() if k not in ['route_xy','seconds']},expected_saved_route=record['local']['plan']['route_xy'],source_files={str(O/n):sha(O/n) for n in [f'maps/frame_{i:03d}.npz',f'policy_inputs/frame_{i:03d}.npz',f'local_plans/frame_{i:03d}.npz','policy_records.json','episode_arrays.npz']},scope='Only current accepted_direction enters intervention; current after-planning request/desired and measured/applied values are report-only and absent from snapshot model fields.')
 (H/'direction_root_cost_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
 sources=dict(p['source_sha256'])
 for n in ['direction_root_planning.py','direction_root_cost.py','test_direction_root_cost.py','seal_direction_root_cost.py','run_direction_root_cost_audit.py']:
  file=H/n;sources[str(file)]=sha(file)
 for file in [W/'docs/superpowers/specs/2026-10-05-direction-root-cost-design.md',W/'docs/superpowers/plans/2026-10-05-direction-root-cost.md',H/'direction_root_cost_source_diff.patch']:sources[str(file)]=sha(file)
 inputs=dict(p['input_sha256']);inputs.update(f['final_artifact_sha256'])
 for n in ['guarded_reference_final_identity.json','guarded_reference_document_sync.json','guarded_reference_final_check.json','direction_root_cost_snapshot.npz','direction_root_cost_expected_baseline.npz','direction_root_cost_provenance.json']:
  inputs[str(H/n)]=sha(H/n)
 protocol=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),frame=175,maximum_actual_grid_searches=1,maximum_actual_hybrid_searches=2,maximum_new_simulations=0,initial_direction=accepted,direction_change_cost_m=.5,reverse_cost_multiplier=1.25,node_budget=3000,primitive_length_m=.3,collision_sample_spacing_m=.05,goal_tolerance_m=.2,source_sha256=sources,input_sha256=inputs,acceptance=dict(interface='old exact baseline reference/target/nodes/current provenance/immutability/budget/identity',nominal_eligibility='both success, changed first primitive direction, same sampled nominal raster and endpoint tolerance',navigation='not tested/no physical label'),formal_matrix=False,whole_research_complete=False,automation='ACTIVE')
 (H/'direction_root_cost_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');print(json.dumps({k:v for k,v in protocol.items() if k not in ['source_sha256','input_sha256']},indent=2));print('sources',len(sources),'inputs',len(inputs))
if __name__=='__main__':main()
