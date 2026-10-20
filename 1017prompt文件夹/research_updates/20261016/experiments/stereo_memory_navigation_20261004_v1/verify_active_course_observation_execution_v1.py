from pathlib import Path
import json,datetime,hashlib,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent;O=H/'active_course_observation_run_v1';status=json.loads((H/'active_course_observation_status_v1.json').read_text());assert status['stage']=='finished_actual_probe_only' and status['exit_code']==0
es=json.loads((O/'execution_states.json').read_text());ph=np.load(O/'physics.npz');rows=json.loads((O/'policy_records.json').read_text());N=len(rows);assert len(es)==N+1 and es[0] is None;clone=ExecutionClone.from_snapshot(es[1]);ticks=0
for tick in range(5,5*N):
 applied,smooth=clone.step(ph['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1]);np.testing.assert_equal(applied,ph['applied_action'][tick+1]);np.testing.assert_equal(smooth,ph['smoothed_target'][tick+1])
 if (tick+1)%5==0:assert clone.snapshot()==es[(tick+1)//5]
 ticks+=1
for i,r in enumerate(rows):np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],np.asarray(r['local']['request']).reshape(5,2))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_request_blocks_exact=N,actual_dispatch_ticks_exact=N*5,measured_feedback_clone_ticks_exact=ticks,execution_snapshots_exact=N-1,initial5_not_cloned=True,not_recursive_physical_prediction=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
f=H/'active_course_observation_execution_verification_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
