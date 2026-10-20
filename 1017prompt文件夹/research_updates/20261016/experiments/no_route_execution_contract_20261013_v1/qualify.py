import bootstrap
from bootstrap import D,W
import json,numpy as np,hashlib
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
R=W/'experiments/replanning_reserve_20261013_v1';results={}
for arm in ['baseline_e1','candidate_e1']:
 r=R/(arm+'_run');ph=np.load(r/'physics.npz');rows=json.loads((r/'policy_records.json').read_text());clone=ExecutionClone.from_snapshot(json.loads((r/'initial_execution_after_reset.json').read_text()));err=0.;ticks=0
 for tick in range(len(ph['raw_command_target'])-1):
  applied,smooth=clone.step(ph['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1]);np.testing.assert_array_equal(applied,ph['applied_action'][tick+1]);np.testing.assert_array_equal(smooth,ph['smoothed_target'][tick+1]);ticks+=1
 zero=np.array([not any(row['request']) and row.get('plan',{}).get('status')=='observed_obstacle_blocks_route' for row in rows]);idx=np.flatnonzero((ph['physics_step_in_block']>0)&np.isin(ph['action_block_index'],np.flatnonzero(zero)))
 if len(idx):
  assert np.all(ph['raw_command_target'][idx]==0);xy=ph['position_world_xy'];travel=float(np.linalg.norm(xy[idx]-xy[idx-1],axis=1).sum());main=ph['applied_action'][idx,0];aux=ph['applied_action'][idx,1];j=np.flatnonzero(main!=0);last=None if not len(j) else float(ph['simulation_time_s'][idx[j[-1]]]);data=dict(blocks=int(zero.sum()),ticks=len(idx),nonzero_applied_main_ticks=int((main!=0).sum()),last_nonzero_main_time_s=last,nonzero_aux_ticks=int((aux!=0).sum()),aux_min_max_rad=[float(aux.min()),float(aux.max())],actual_tick_travel_m=travel)
 else:data=None
 results[arm]=dict(full_execution_exact_ticks=ticks,no_route=data,input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [r/'physics.npz',r/'policy_records.json',r/'initial_execution_after_reset.json']})
(D/'qualification.json').write_text(json.dumps(dict(result='mistranslation_hypothesis_rejected_before_physics',arms=results,physical=0,fit=0,recovery=0,limits='Recorded actual measured feedback replay, not predicted future dynamics. Quaternion here is privileged runtime feedback used by installed lowlevel controller, not claimed deployable IMU estimator. Zero desired bank retains active PID compensation; no braking guarantee or proof of cause.'),indent=2)+'\n');print(results)
