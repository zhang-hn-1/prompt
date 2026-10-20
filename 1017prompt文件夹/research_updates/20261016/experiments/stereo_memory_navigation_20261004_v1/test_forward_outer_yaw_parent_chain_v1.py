from pathlib import Path
import copy,json
import pytest
from assess_forward_outer_yaw_e1_v1 import verify_forward_parent_record
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
@pytest.mark.parametrize('component',['encoded','correction'])
def test_edited_removed_term_cannot_pass_unchanged_raw_and_issued(component):
 row=json.loads((H/'current_tracking_chain_e1_b0_run_v1/policy_records.json').read_text())[70];protocol=load_action_protocol(P/'configs/data_contract.yaml');verify_forward_parent_record(row['current_tracking_chain'],row['request'],protocol)
 bad=copy.deepcopy(row['current_tracking_chain'])
 if component=='encoded':bad['calibrated_bank_mapping']['encoded_aux_rad']+=.01
 else:bad['yaw_rate_feedback_auxiliary_rad']+=.01
 with pytest.raises(AssertionError):verify_forward_parent_record(bad,row['request'],protocol)
