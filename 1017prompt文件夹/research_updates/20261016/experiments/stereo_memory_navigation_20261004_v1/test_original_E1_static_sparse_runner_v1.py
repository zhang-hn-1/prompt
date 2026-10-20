from pathlib import Path
import ast
H=Path(__file__).resolve().parent

def test_only_lifetime_change_and_546_scope():
 s=(H/'run_original_E1_static_sparse_memory_navigation_v1.py').read_text();ast.parse(s)
 assert 'for block in range(546):' in s and 'budget_s=54.6' in s
 assert 'policy.grid=StaticSparsePointRiskGrid(' in s and 'static_scene_assumption=True' in s
 assert 'raw_point_memory=TentativePointMemory()' in s
 assert s.count('request=policy.act(observation,local_goal)')==1
 assert 'journal.planned(row)' in s and 'journal.executed(row)' in s
 assert s.index('journal.planned(row)')<s.index('current=runtime.execute(request)')<s.index('journal.executed(row)')
 assert "policy_rows.append(row)" not in s
 assert 'known_GT_pose=True,native_pose_used=False' in s
 assert 'bind_fixed_appearance(' not in s
 assert 'if current.collision:' in s and "termination='success'" in s
