from pathlib import Path
import ast
H=Path(__file__).resolve().parent

def test_single_candidate_act_and_readonly_shadow_stops_at_fork():
 s=(H/'run_original_E1_route_side_navigation_v1.py').read_text();ast.parse(s)
 assert s.count('request=policy.act(observation,local_goal)')==1
 assert s.count('shadow.observe(request,observation,local_goal')==1
 assert 'policy.grid=RouteSideCommitmentGrid(' in s and 'baseline.grid=StaticSparsePointRiskGrid(' in s
 assert 'for block in range(1200)' in s and 'if block==545:' in s
 assert "termination='REJECT_LOCAL54_6_PROGRESS_STOP';break" in s
 assert 'goal_remaining<=4.182900500 and axis_progress>=2.427377623' in s
 assert 'known_GT_pose=True,native_pose_used=False' in s
 assert "raw_point_memory=TentativePointMemory()" in s
 assert 'bind_fixed_appearance(' not in s
