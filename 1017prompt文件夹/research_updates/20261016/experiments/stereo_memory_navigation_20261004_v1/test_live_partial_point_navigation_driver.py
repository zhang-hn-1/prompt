from pathlib import Path
import ast
H=Path(__file__).resolve().parent

def test_driver_has_no_future_archive_actor_and_exact_single_reset():
 s=(H/'run_live_partial_point_navigation_v1.py').read_text();ast.parse(s)
 assert s.count('runtime.reset(episode)')==1
 assert 'range(600)' in s and 'if block<140:' in s
 assert 'PartialPointRiskGrid' in s and 'TentativePointMemory' in s
 assert 'native_flat_pose_navigation_R1_run_v1' not in s
 assert 'policy.grid.seed(' not in s
 assert 'policy.set_source_packet(risk_source)' in s

def test_current_native_inputs_and_point_source_exported_before_truth():
 s=(H/'run_live_partial_point_navigation_v1.py').read_text()
 assert s.index('raw_point_memory.update(raw_record,t)') < s.index("left_depth_m=read_camera_depth(self._left_camera)")
 assert "velocity_world_m_s=(R0@result['velocity']).tolist()" in s
 assert 'navigation_observation.build(' in s
 assert 'policy.set_current_packet(current_packet)' in s

def test_full_plan_and_end_to_end_timing_export():
 s=(H/'run_live_partial_point_navigation_v1.py').read_text();assert 'dataclasses.asdict(w) for w in policy.plan.waypoints' in s;assert 'end_to_end_wall_s=time.monotonic()-action_wall_begin' in s;assert 'execute_capture_NN_wall_s=' in s
