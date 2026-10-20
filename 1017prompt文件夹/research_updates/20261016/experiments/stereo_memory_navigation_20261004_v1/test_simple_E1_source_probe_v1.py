import ast
from pathlib import Path
H=Path(__file__).resolve().parent

def test_prescribed_source_only_chain():
    p=H/'run_simple_E1_partial_source_probe_v1.py'
    assert p.exists()
    s=p.read_text();ast.parse(s)
    assert 'request=commands[block].copy()' in s
    assert 'policy.act(' not in s and 'observer.request(' not in s
    assert 'bind_fixed_appearance(' not in s
    assert 'golden_e1_0001' in s
    assert "out/'known_poses'" in s
    assert 'current_GT_pose_registration=True' in s
    assert 'len(clocks)==141 and len(source_rows)==29' in s
    assert 'if current.collision:' in s
    assert 'goal_position_world_m[:2]))<=.2' not in s

def test_original_episode_identity_and_incomplete_stop_not_success():
    s=(H/'run_simple_E1_partial_source_probe_v1.py').read_text()
    assert 'export_task_identity(out/' not in s
    assert "actual_task='original_E1_prescribed_source_probe_14s'" in s
    assert s.index("assert len(actual_requests)==140") < s.index("complete=True")

def test_last_action_collision_cannot_complete():
    s=(H/'run_simple_E1_partial_source_probe_v1.py').read_text()
    assert "assert termination=='source_interface_budget_complete'" in s
    assert s.index("assert termination=='source_interface_budget_complete'") < s.index('complete=True')
