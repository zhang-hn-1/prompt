from pathlib import Path
import ast
H=Path(__file__).resolve().parent

def test_direct_navigation_keeps_knownpose_partial_scope():
 p=H/'run_original_E1_direct_memory_navigation_v1.py';assert p.exists();s=p.read_text();ast.parse(s)
 assert 'for block in range(600)' in s
 assert s.count('request=policy.act(observation,local_goal)')==1
 assert 'observer.request(' not in s and 'request=commands[block]' not in s
 assert 'known_GT_pose=True,native_pose_used=False' in s
 assert 'raw_point_memory=TentativePointMemory()' in s
 assert 'bind_fixed_appearance(' not in s
 assert 'if current.collision:' in s
 assert 'budget_s=60,true_goal_xy_threshold_m=.2' in s

def test_action_latency_and_memory_provenance_saved():
 s=(H/'run_original_E1_direct_memory_navigation_v1.py').read_text()
 assert 'end_to_end_wall_s=time.monotonic()-action_wall_begin' in s
 assert 'policy_wall_s=policy_wall' in s
 assert "out/'raw_point_memory_snapshots'/f'frame_{block:03d}.json'" in s
