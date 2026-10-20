from pathlib import Path
import json,sys,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
p=np.load(D/'physics.npz');rows=json.loads((D/'policy_records.json').read_text());shadow=json.loads((D/'B0_shadow_until_fork.json').read_text());f=shadow[-1]['frame'];assert shadow[-1]['first_request_fork']
clone=ExecutionClone.from_snapshot(json.loads((D/'execution/frame_001.json').read_text()));N=len(rows)
for tick in range(5,N*5):
 a,sm=clone.step(p['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(p['root_quaternion_world_wxyz'][tick]),aux_position_rad=p['joint_position_rad'][tick,1],aux_velocity_rad_s=p['joint_velocity_rad_s'][tick,1]);np.testing.assert_array_equal(a,p['applied_action'][tick+1]);np.testing.assert_array_equal(sm,p['smoothed_target'][tick+1])
 if (tick+1)%5==0:assert clone.snapshot()==json.loads((D/'execution'/f'frame_{(tick+1)//5:03d}.json').read_text())
tick=f*5;kw=dict(bank_rad=bank_from_quaternion(p['root_quaternion_world_wxyz'][tick]),aux_position_rad=p['joint_position_rad'][tick,1],aux_velocity_rad_s=p['joint_velocity_rad_s'][tick,1]);snapshot=json.loads((D/'execution'/f'frame_{f:03d}.json').read_text());c=ExecutionClone.from_snapshot(snapshot);b=ExecutionClone.from_snapshot(snapshot);ca,cs=c.step(np.asarray(shadow[-1]['candidate_request']).reshape(5,2)[0],**kw);ba,bs=b.step(np.asarray(shadow[-1]['B0_request']).reshape(5,2)[0],**kw);np.testing.assert_array_equal(ca,p['applied_action'][tick+1]);np.testing.assert_array_equal(cs,p['smoothed_target'][tick+1]);out=dict(measured_feedback_reconstruction_ticks=(N-1)*5,snapshots_exact=N-1,first_fork=f,first_tick_B0_applied=ba.tolist(),first_tick_candidate_applied=ca.tolist(),first_tick_applied_different=bool(not np.array_equal(ba,ca)),current_state_only_first_tick_comparison=True,no_future_B0_truth_or_physical_prediction=True);(D/'execution_reconstruction_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
