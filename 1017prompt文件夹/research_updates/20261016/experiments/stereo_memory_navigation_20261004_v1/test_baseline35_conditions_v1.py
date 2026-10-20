from baseline35_conditions_v1 import build_config
import pytest,json
from pathlib import Path
H=Path(__file__).resolve().parent
@pytest.mark.parametrize('name,occ',[('e0','cluttered_hard_0_bodyband.npz'),('e2','e2_corridor_bodyband.npz')])
def test_real_episode_and_35second_budget_without_depth_accuracy_gate(name,occ):
 c=build_config(name);assert c['episode_id']==f'golden_{name}_0001';assert Path(c['occupancy']).name==occ;assert c['requests_max']==350 and c['sim_budget_s']==35.;assert c['source_accuracy_is_diagnostic_only'];assert 'source_gate' not in c and 'local_gate' not in c;assert c['knownpose_task_prior_baseline_not_strict_visual'];assert c['no_controller_change']
def test_disallows_unplanned_repeat_of_e1():
 with pytest.raises(ValueError):build_config('e1')
