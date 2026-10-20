import pytest

def metric(**changes):
 d=dict(success=True,raw_contacts_zero=True,along_start_goal_axis_m=2.,cross_rms_m=.2,cross_peak_m=.3,wall_median_s=1.,wall_p90_s=2.)
 d.update(changes);return d

def test_success_requires_real_arrival_and_progress():
 from dense_current_live_gates_v1 import gates
 b=metric(success=False,along_start_goal_axis_m=1.)
 assert all(gates(b,metric(),changed=True,contract=True).values())
 assert not gates(b,metric(success=False),changed=True,contract=True)['arrival']
 assert not gates(b,metric(along_start_goal_axis_m=1.),changed=True,contract=True)['net_progress_improves']

@pytest.mark.parametrize('field,value,gate',[('raw_contacts_zero',False,'no_contact'),('wall_p90_s',2.50001,'latency'),('cross_rms_m',.30001,'cross_rms'),('cross_peak_m',.37501,'cross_peak')])
def test_frozen_gate_failure_is_not_rescued(field,value,gate):
 from dense_current_live_gates_v1 import gates
 assert not gates(metric(along_start_goal_axis_m=1.),metric(**{field:value}),changed=True,contract=True)[gate]

def test_interface_or_contract_failure_not_navigation_acceptance():
 from dense_current_live_gates_v1 import gates
 assert not gates(metric(),metric(along_start_goal_axis_m=3.),changed=False,contract=True)['actual_action_effect']
 assert not gates(metric(),metric(along_start_goal_axis_m=3.),changed=True,contract=False)['runtime_source_contract']
