from pathlib import Path
import sys,json,math,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');H=W/'experiments/oracle_feasible_route_20260929_v9/hybrid';sys.path[:0]=[str(P),str(H)]
from bank_mapping import BankMap
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
model=BankMap(json.loads((H/'bank_mapping.json').read_text()));protocol=load_action_protocol(DEFAULT_CONTRACT.resolve());R=D/'candidate_e1_run';rows=json.loads((R/'policy_records.json').read_text());native={round(n['time_s'],7):n for n in json.loads((R/'native_predictions.json').read_text())};imu={round(n['time_s'],7):n for n in json.loads((R/'native_imu_reads.json').read_text())};requests=np.load(R/'requests.npz')['requests'];bad=[];count=0;changed=0
for row in rows:
 a=row.get('tangent_field')
 if not a or row['tracker_state_after_act']['braking_state']!='tracking':continue
 i=row['frame'];t=round(row['time_s'],7);n=native[t];rot=np.asarray(n['rotation_world']);yaw=float(np.arctan2(rot[1,0],rot[0,0]));pos=np.asarray(a['position']);v=np.asarray(n['velocity_world_m_s'][:2],np.float32);speed=float(np.linalg.norm(v.astype(float)));current_speed=float(np.linalg.norm(v));gyro=float(np.float32((rot@np.asarray(imu[t]['gyro_body_rad_s']))[2]));main=float(row['uncapped_request'][0]);sign=np.sign(main)
 def auxiliary(target):
  delta=np.asarray(target)-pos;body_y=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1];k=float(2*body_y/np.dot(delta,delta));bank=model.query(main,speed,k)['encoded_aux_rad'];correction=float(np.clip(-sign*.15*(k*sign*max(current_speed,.2)-gyro),math.radians(-5),math.radians(5)));return np.float32(np.clip(bank+correction,protocol.minimum[1],protocol.maximum[1]))
 new=auxiliary(a['new_target']);old=auxiliary(a['original_target'])
 if not np.allclose(requests[i,1::2],new,rtol=0,atol=1e-7):bad.append(i)
 count+=1;changed+=int(new!=old)
r=dict(actual_normal_tracking_requests_checked=count,errors=bad,same_candidate_current_state_legacy_target_aux_differences=changed,pass_gate=bool(count and not bad),scope='new target to original geometriccurvature/BankMap/yaw correction/protocol actual issued requests; oldtarget counterfactual same candidate state only, not another physical trajectory; native world gyro privileged estimate source disclosed')
(D/'request_effect.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
