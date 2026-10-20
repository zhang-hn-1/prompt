import copy,json
from pathlib import Path
import pytest
from assess_current_tracking_chain_e1_v1 import validate_request_chain,validate_complete_timeout_clock
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
def test_tampered_raw_does_not_pass_disconnected_mapping_and_issued_aux():
 protocol=load_action_protocol(P/'configs/data_contract.yaml');row=json.loads((H/'current_tracking_chain_e1_b0_run_v1/policy_records.json').read_text())[70];validate_request_chain(row['current_tracking_chain'],row['request'],protocol)
 bad=copy.deepcopy(row['current_tracking_chain']);bad['policy_raw_auxiliary_target_rad']+=.01
 with pytest.raises(AssertionError):validate_request_chain(bad,row['request'],protocol)
def test_short_timeout_clock_not_complete_even_when_relative_counts_consistent():
 import numpy as np
 t=np.arange(351)*.1;validate_complete_timeout_clock(t,350)
 with pytest.raises(AssertionError):validate_complete_timeout_clock(t[:101],100)
 with pytest.raises(AssertionError):validate_complete_timeout_clock(t-.1,350)
