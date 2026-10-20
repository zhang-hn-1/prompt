import copy,json
from pathlib import Path
import pytest
from suffix_completion_semantics_v1 import suffix_completion_eligibility
H=Path(__file__).resolve().parent
def fixture():
 d=H/'forward_outer_yaw_e1_b0_run_v1';rows=json.loads((d/'policy_records.json').read_text());return rows[120],rows[119],json.loads((d/"local_plans/frame_119.json").read_text())
def test_actual_first_boundary_eligible_but_not_physical_proof():
 current,previous,prior_plan=fixture();r=suffix_completion_eligibility(current,previous,prior_plan);assert r['eligible'];assert r['endpoint_distance_m']>.2;assert not r['physical_safety_certified'];assert r['remaining_arc_m']<.5
@pytest.mark.parametrize('why',['hazard','pending','stale'])
def test_negative_real_boundary(why):
 current,previous,prior_plan=fixture();current=copy.deepcopy(current)
 if why=='hazard':current['reference_decision']['guard']['reject_reasons'].append('continuous_observed_hazard_clearance')
 elif why=='pending':current['reference_decision']['state_before']['pending_direction']='reverse'
 else:current['reference_decision']['measured_time_s']-=.1
 assert not suffix_completion_eligibility(current,previous,prior_plan)['eligible']

@pytest.mark.parametrize('where',['current_boundary','past_plan'])
def test_nan_and_future_clock_rejected(where):
 current,previous,prior_plan=fixture()
 if where=='current_boundary':current['reference_decision']['measured_time_s']=float('nan')
 else:prior_plan['time_s']=current['time_s']+1
 assert not suffix_completion_eligibility(current,previous,prior_plan)['eligible']
