"""Check the scored V2 comparison against published raw states and clocks."""
import json,hashlib
from pathlib import Path
import numpy as np
h=Path(__file__).resolve().parent
hard=['cluttered-0_e0_0002','cluttered-0_e0_0003','cluttered-0_e0_0011','hard_0_e1_0001','hard_0_e1_0002','hard_0_e1_0003','hard_0_e1_0004']
golden=['golden_e0_0001','golden_e1_0001','golden_e2_0001']
for path in ['hybrid_hard_status.json','physical_baseline_status.json','hybrid_golden_status.json']:
 records=json.loads((h/path).read_text());assert all(r['returncode']==0 for r in records),(path,records)
for path in ['hybrid_hard_identity_after.json','physical_baseline_identity_after.json','hybrid_golden_identity_after.json']:
 assert all(json.loads((h/path).read_text()).values()),path
out={'protocol':'ball_pointgoal_v1_2_contact_35s','hard_0':{},'golden':{},'summary':{}}
for case in hard:
 result={}
 for label,prefix in [('baseline','navigation_physical_baseline_'),('candidate','navigation_hybrid_hard_')]:
  d=h/f'{prefix}{case}'/f'episode_{case}'
  m=json.loads((d/'metrics.json').read_text());env=json.loads((d/'environment.json').read_text());z=np.load(d/'states.npz');a=np.load(d/'actions.npz');contact=json.loads((d/'collision_audit.json').read_text())
  assert m['evaluation_protocol_id']==out['protocol'] and env['evaluation_protocol_id']==out['protocol']
  manifest=json.loads((d/'manifest.json').read_text());ep=next(x for x in manifest['episodes'] if x['episode_id']==case)
  assert env['scene_sha256']==ep['scene_sha256'] and env['occupancy_sha256']==ep['occupancy_sha256']
  assert m['episode_budget_s']==35.0 and len(z['position_world_xy'])==m['action_blocks']+1
  assert float(z['simulation_time_s'][-1]) <= 35.0+1e-5
  assert np.array_equal(z['collision'],z['physical_contact'])
  assert all(sample['available'] for sample in contact['samples'])
  assert m['collision']==float(m['termination_reason']=='collision')
  assert m['success']==float(m['termination_reason']=='success')
  assert len(a['action_blocks'])==m['action_blocks']
  result[label]=dict(termination=m['termination_reason'],time_s=float(z['simulation_time_s'][-1]),final_distance_m=m['final_distance_m'],path_length_m=m['path_length_m'],minimum_map_clearance_m=m['minimum_clearance_m'],physical_contact_count=int(z['physical_contact'].sum()),map_nonpositive_count=m['map_nonpositive_clearance_samples'],initial_position_xy=z['position_world_xy'][0].tolist(),initial_yaw_rad=float(z['yaw_world_rad'][0]),initial_joint_positions_rad=z['joint_position_rad'][0].tolist(),initial_joint_velocities_rad_s=z['joint_velocity_rad_s'][0].tolist(),initial_linear_velocity_world_m_s=z['linear_velocity_world_m_s'][0].tolist(),initial_angular_velocity_world_rad_s=z['angular_velocity_world_rad_s'][0].tolist(),initial_applied_action=z['applied_action'][0].tolist(),reference_hash=hashlib.sha256(z['kinodynamic_reference_pose'].tobytes()).hexdigest(),action_blocks=m['action_blocks'])
 assert result['baseline']['initial_position_xy']==result['candidate']['initial_position_xy']
 assert result['baseline']['initial_yaw_rad']==result['candidate']['initial_yaw_rad']
 assert result['baseline']['initial_joint_positions_rad']==result['candidate']['initial_joint_positions_rad']
 for key in ('initial_joint_velocities_rad_s','initial_linear_velocity_world_m_s','initial_angular_velocity_world_rad_s','initial_applied_action'):
  assert result['baseline'][key]==result['candidate'][key],(case,key)
 result['candidate']['route_decision']=json.loads((h/f'navigation_hybrid_hard_{case}'/'feedback/reference_decision.json').read_text())
 if result['candidate']['route_decision']['selected_source']=='frozen_measured_time_with_reserve':assert result['candidate']['reference_hash']==result['baseline']['reference_hash']
 out['hard_0'][case]=result
for case in golden:
 d=h/f'navigation_hybrid_golden_{case}'/f'episode_{case}';m=json.loads((d/'metrics.json').read_text());env=json.loads((d/'environment.json').read_text());z=np.load(d/'states.npz');a=np.load(d/'actions.npz');contact=json.loads((d/'collision_audit.json').read_text())
 assert m['evaluation_protocol_id']==out['protocol'] and env['evaluation_protocol_id']==out['protocol'] and m['episode_budget_s']==35.
 assert len(z['position_world_xy'])==m['action_blocks']+1 and len(a['action_blocks'])==m['action_blocks']
 assert float(z['simulation_time_s'][-1])<=35.0+1e-5
 assert np.array_equal(z['collision'],z['physical_contact'])
 manifest=json.loads((d/'manifest.json').read_text());ep=next(x for x in manifest['episodes'] if x['episode_id']==case)
 assert env['scene_sha256']==ep['scene_sha256'] and env['occupancy_sha256']==ep['occupancy_sha256']
 assert all(sample['available'] for sample in contact['samples'])
 out['golden'][case]=dict(termination=m['termination_reason'],time_s=float(z['simulation_time_s'][-1]),final_distance_m=m['final_distance_m'],physical_contact_count=int(z['physical_contact'].sum()),route_decision=json.loads((h/f'navigation_hybrid_golden_{case}'/'feedback/reference_decision.json').read_text())['selected_source'])
for label in ['baseline','candidate']:
 results=[v[label] for v in out['hard_0'].values()]
 out['summary'][label]=dict(success=sum(r['termination']=='success' for r in results),collision=sum(r['termination']=='collision' for r in results),timeout=sum(r['termination']=='timeout' for r in results),runtime_error=sum(r['termination']=='runtime_error' for r in results),episodes=len(results))
assert all(r['termination']=='success' for r in out['golden'].values())
(h/'full_comparison.json').write_text(json.dumps(out,indent=2))
print(out['summary'])
for case,v in out['hard_0'].items():print(case,v['baseline']['termination'],round(v['baseline']['final_distance_m'],3),'->',v['candidate']['termination'],round(v['candidate']['final_distance_m'],3),v['candidate']['route_decision']['selected_source'])
