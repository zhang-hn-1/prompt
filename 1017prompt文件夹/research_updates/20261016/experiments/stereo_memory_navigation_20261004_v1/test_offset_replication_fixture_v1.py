from pathlib import Path
import ast
H=Path(__file__).resolve().parent

def test_frozen_control_diff_only_fixture_and_names():
 a=(H/'run_flat_ground_pose_D3_v1.py').read_text();b=(H/'run_offset_replication_D4_v1.py').read_text()
 b=b.replace('offset_replication_protocol_v1.json','flat_ground_pose_protocol_v1.json').replace('offset_replication_D4_run_v1','flat_ground_pose_D3_run_v1').replace('newstart=np.array([-2.2,.15,.45]);newgoal=np.array([2.2,.15,.45])','newstart=np.array([-2.2,0.,.45]);newgoal=np.array([2.2,0.,.45])').replace('local_goal_m=4.4,initial_goal_lateral_offset_m=.15,frozen_D3_control_stack=True,','local_goal_m=4.4,')
 assert a==b

def test_goal_metric_matches_offset_not_original():
 s=(H/'assess_offset_replication_actual_v1.py').read_text()
 assert 'xy[-1]-[2.2,.15]' in s
 assert 'xy[-1]-[2.2,0.]' not in s

def test_independent_evidence_names_and_variant():
 s=(H/'verify_offset_replication_sensor_observation_v1.py').read_text()
 assert "H/'offset_replication_errors_posthoc_v1.npz'" in s
 assert "H/'flat_ground_pose_errors_posthoc_v1.npz'" not in s
 assert "for arm in ['D4']:" in (H/'verify_offset_replication_causal_v1.py').read_text()
 for f in H.glob('*offset_replication*v1.py'):ast.parse(f.read_text())

def test_explicit_collision_and_budget_fail_even_with_arrival():
 from offset_replication_acceptance import safety_checks
 good=dict(actual_success=True,contact_frames=0,collision_frames=0,elapsed_s=59.9)
 assert all(safety_checks(good).values())
 assert not safety_checks(dict(good,collision_frames=1))['no_collision']
 assert not safety_checks(dict(good,elapsed_s=60.1))['within_task_budget']
 assert not safety_checks(dict(good,contact_frames=1))['no_contact']
