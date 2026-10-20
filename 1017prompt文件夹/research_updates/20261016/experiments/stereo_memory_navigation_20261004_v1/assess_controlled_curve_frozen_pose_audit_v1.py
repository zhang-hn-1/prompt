from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;D=H/'controlled_curve_frozen_pose_audit_v1'
c=json.loads((D/'prediction_completion.json').read_text())
for group in ('source_sha256','prediction_sha256'):
 for f,s in c[group].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
out={}
for name in ('tracking_tube_D1_run_v1',):
 p=np.load(D/(name+'_predictions.npz'));ph=np.load(H/name/'physics.npz');s=np.load(D/(name+'_sensors.npz'))
 t=ph['simulation_time_s'];assert np.array_equal(t,p['time_s']) and np.array_equal(t,s['time_s'])
 R=Rotation.from_quat(ph['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();relative=R[0].T[None]@R
 truth=(ph['root_position_world_m']-ph['root_position_world_m'][0])@R[0]
 e=p['position']-truth;world_e=e@R[0].T;delta=relative.transpose(0,2,1)@p['rotation']
 ae=np.degrees(np.arccos(np.clip((np.trace(delta,axis1=1,axis2=2)-1)/2,-1,1)))
 actual_up=np.einsum('nji,j->ni',R,np.array([0.,0.,1.]));pred_up=np.einsum('nji,j->ni',p['rotation'],s['initial_normal'])
 up=np.degrees(np.arccos(np.clip(np.sum(actual_up*pred_up,axis=1),-1,1)))
 est_world=R[0][None]@p['rotation'];yaw=np.arctan2(est_world[:,1,0],est_world[:,0,0])-np.arctan2(R[:,1,0],R[:,0,0]);yaw=np.degrees(np.abs((yaw+np.pi)%(2*np.pi)-np.pi))
 v=t>=1.;norm=np.linalg.norm(e,axis=1);rms=float(np.sqrt(np.mean(norm[v]**2)));peak=float(norm[v].max());ap=float(np.percentile(ae[v],90))
 out[name]=dict(samples=len(t),gate_samples=int(v.sum()),duration_s=float(t[-1]),rms3D_m=rms,peak3D_m=peak,attitude_p90_deg=ap,passed=rms<=.05 and peak<=.1 and ap<=2.,world_xy_rms_m=float(np.sqrt(np.mean(np.sum(world_e[v,:2]**2,axis=1)))),world_z_rms_m=float(np.sqrt(np.mean(world_e[v,2]**2))),up_error_p90_deg=float(np.percentile(up[v],90)),yaw_error_p90_deg=float(np.percentile(yaw[v],90)),initial_z_m=float(ph['root_position_world_m'][0,2]),final_z_m=float(ph['root_position_world_m'][-1,2]))
 np.savez_compressed(D/(name+'_errors.npz'),time_s=t,error_world_m=world_e,attitude_error_deg=ae,up_error_deg=up,yaw_error_deg=yaw,gate_valid=v)
result=dict(historical_not_fresh=True,post_prediction_scoring=True,old_gates_unchanged=True,sequences=out,both_pass=all(x['passed'] for x in out.values()))
(D/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
