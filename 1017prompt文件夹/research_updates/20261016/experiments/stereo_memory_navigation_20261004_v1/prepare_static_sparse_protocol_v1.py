from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent
p=json.loads((H/'original_E1_direct_memory_navigation_protocol_v1.json').read_text())
p.update(schema='original_E1_static_sparse_memory_knownGTpose_TaskH_54.6s_v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),static_scene_assumption=True,only_control_change='hazard_cell_lifetime_past_measured_static_sparse_until_reset',maximum_actual_requests=546,simulation_budget_s=54.6,wall_budget_s=600,no_retry=True,comparison='verified_B0_completed54.6prefix_only_not_full60',local_goal_remaining_gate_m=4.054687998,local_axis_progress_gate_m=2.566441774,navigation_goal_threshold_m=.2)
for f in ['static_sparse_point_risk_grid_v1.py','policy_row_journal_v1.py','test_static_sparse_point_risk_grid_v1.py','test_policy_row_journal_v1.py','test_original_E1_static_sparse_runner_v1.py','qualify_static_sparse_memory_and_journal_v1.py','run_original_E1_static_sparse_memory_navigation_v1.py','run_original_E1_static_sparse_memory_navigation_stage_v1.py','prepare_static_sparse_protocol_v1.py']:
 path=H/f;p['source_sha256'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
p['input_sha256'][str(H/'static_sparse_memory_journal_CPU_v1/assessment.json')]=hashlib.sha256((H/'static_sparse_memory_journal_CPU_v1/assessment.json').read_bytes()).hexdigest()
(H/'original_E1_static_sparse_memory_navigation_protocol_v1.json').write_text(json.dumps(p,indent=2)+'\n')
print('protocol identities frozen',len(p['source_sha256']),len(p['input_sha256']))
