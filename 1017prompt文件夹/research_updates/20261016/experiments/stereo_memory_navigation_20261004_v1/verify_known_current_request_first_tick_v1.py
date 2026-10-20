from pathlib import Path
import json,sys,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
rows=json.loads((D/'policy_records.json').read_text());ph=np.load(D/'physics.npz');out=[]
for i in [221,371]:
 snapshot=json.loads((D/'execution'/f'frame_{i:03d}.json').read_text());command=np.asarray(rows[i]['request']).reshape(5,2)[0];tick=i*5;current=dict(bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1]);clone=ExecutionClone.from_snapshot(snapshot);a,sm=clone.step(command,**current);out.append(dict(frame=i,current_given_request=command.tolist(),current_feedback={k:float(v) for k,v in current.items()},first_tick_applied=a.tolist(),first_tick_smoothed=sm.tolist(),phase='AFTER_CURRENT_ACT_BEFORE_DISPATCH',not_recursive_physical_prediction=True,future_feedback_used=False))
f=D/'known_current_request_first_tick_predictions_v1.json';f.write_text(json.dumps(out,indent=2)+'\n');seal={'producer_output_sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'scope':'predicted execution target sealed before future dispatch label comparison; current request given now, not predicted'};(H/'known_current_request_first_tick_prelabel_seal_v1.json').write_text(json.dumps(seal,indent=2)+'\n')
for r in out:
 i=r['frame'];np.testing.assert_array_equal(r['first_tick_applied'],ph['applied_action'][i*5+1]);np.testing.assert_array_equal(r['first_tick_smoothed'],ph['smoothed_target'][i*5+1])
(D/'known_current_request_first_tick_verification_v1.json').write_text(json.dumps(dict(two_first_ticks_exact=True,prelabel_output_seal=seal,current_request_known_only_after_act=True,no_future_actual_request_plan=True,not_online_or_physical_prediction=True,formal35_NOTRUN=True),indent=2)+'\n');print('Two first ticks exact; no future feedback input; no physical prediction')
