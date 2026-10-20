import numpy as np,pytest
from safe_start_motion_v1 import SafeStartMotion

def test_program_zeros_then_two_seconds_F_then_stop():
 s=SafeStartMotion([0,0]);a=[s.choose(i,i*.1,[0,0],.1) for i in range(80)]
 assert all(np.array_equal(x,[0,0]) for x in a[:20]+a[40:]);assert all(np.array_equal(x,[.5,0]) for x in a[20:40])
def test_prior_boundary_stop_latches_without_changing_command():
 s=SafeStartMotion([0,0]);assert s.choose(20,2,[.36,0],0).tolist()==[0,0];assert s.aborted
 assert s.choose(21,2.1,[.1,0],0).tolist()==[0,0]
def test_five_continuous_lows_and_rebound_reset():
 s=SafeStartMotion([0,0]);[s.choose(i,i*.1,[0,0],.01) for i in range(40,44)];assert not s.settled
 s.choose(44,4.4,[0,0],.06);assert s.low_count==0
 [s.choose(i,i*.1,[0,0],.01) for i in range(45,50)];assert s.settled
 s.choose(50,5,[0,0],.07);assert not s.settled
def test_future_clock_mismatch_and_nonincreasing_rejected():
 s=SafeStartMotion([0,0]);s.choose(0,0,[0,0],0)
 with pytest.raises(ValueError):s.choose(1,2,[0,0],0)
 with pytest.raises(ValueError):s.choose(0,0,[0,0],0)
def test_nonfinite_input_rejected():
 s=SafeStartMotion([0,0])
 with pytest.raises(ValueError):s.choose(0,0,[np.nan,0],0)
 with pytest.raises(ValueError):SafeStartMotion([np.inf,0])
def test_stop_qualification_not_before_drive_ends():
 s=SafeStartMotion([0,0]);[s.choose(i,i*.1,[0,0],0) for i in range(40)];assert s.low_count==0 and not s.settled

def test_runtime_block_encoding_five_equal_controls():
 from safe_start_motion_v1 import encode_request
 from training.action_protocol import load_action_protocol
 from pathlib import Path
 p=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 a=encode_request([.5,0]);assert a.shape==(10,) and np.array_equal(a.reshape(5,2),np.tile([.5,0],(5,1)))
 assert np.array_equal(p.clip_block(a),a)

def test_final_capture_rebound_or_prior_exit_must_reject():
 from safe_start_motion_v1 import assess_complete_states
 rows=[dict(time_s=i*.1,position=[i*.002,0,0],velocity=[0,0,0]) for i in range(81)]
 assert assess_complete_states(rows)['passes']
 rows[-1]['velocity']=[.06,0,0];assert not assess_complete_states(rows)['final_five_XY_low']
 rows[-1]['velocity']=[0,0,0];rows[-1]['position']=[.71,0,0];assert not assess_complete_states(rows)['inside_center_prior']

def test_incomplete_final_state_sequence_is_not_pass():
 from safe_start_motion_v1 import assess_complete_states
 with pytest.raises(ValueError):assess_complete_states([])
