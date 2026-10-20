import ast,json
from pathlib import Path
from types import SimpleNamespace

H=Path(__file__).parent
def test_active_driver_task_and_termination_identify_current_sampling_original35():
 tree=ast.parse((H/'run_dense_current_live_e1_v4.py').read_text())
 calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='dict']
 context={'episode':SimpleNamespace(to_dict=lambda:{}),'a':SimpleNamespace(arm='candidate'),'len':len,'actual_requests':[],'clocks':[{'episode_time_s':35.}],'termination':'timeout35'}
 task=next(n for n in calls if any(k.arg=='actual_task' for k in n.keywords))
 result=eval(compile(ast.Expression(task),'task','eval'),context)
 assert result['not_original35'] is False
 assert result['only_source_change']=='live_current_packet_stride2_vs_dense_same_SGBM'
 assert result['only_control_change']=='none_relative_to_identical_B0_pipeline'
 term=next(n for n in calls if any(k.arg=='termination' for k in n.keywords))
 assert eval(compile(ast.Expression(term),'termination','eval'),context)['not_original35'] is False

def test_active_protocol_does_not_reuse_old_baseline_comparison_scope():
 p=json.loads((H/'dense_current_live_e1_candidate_protocol_v1.json').read_text())
 assert p['candidate_stage']=='live_current_source_sampling_development_comparison'
 assert p['front_view_existing_behavior_unchanged'] is True
 assert p['new_method_not_present'] is False
 assert p['only_control_change']=='none_relative_to_identical_B0_pipeline'
