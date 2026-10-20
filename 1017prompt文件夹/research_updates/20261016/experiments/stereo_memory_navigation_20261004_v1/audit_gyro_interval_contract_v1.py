from pathlib import Path
import numpy as np,json,datetime,hashlib
from scipy.spatial.transform import Rotation
from rolling_odometry import exponential
H=Path(__file__).resolve().parent;D=H/'RGB_memory_frozen_pose_stress_v1';S=np.load(D/'ideal_sensor_rows.npz');t=S['time_s'];g=S['gyro_body_rad_s'];out=D/'gyro_interval_contract_v1';assert not out.exists();out.mkdir()
rotations={}
for name in ['previous_held','endpoint','midpoint']:
 R=S['initial_rotation_world'].copy();rows=[R.copy()]
 for i in range(1,len(t)):
  w=g[i-1] if name=='previous_held' else g[i] if name=='endpoint' else .5*(g[i-1]+g[i]);R=R@exponential(w*(t[i]-t[i-1]));rows.append(R.copy())
 rotations[name]=np.asarray(rows)
np.savez_compressed(out/'all_rotations_before_labels.npz',time_s=t,**rotations);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sensor_sha256=sha(D/'ideal_sensor_rows.npz'),output_sha256=sha(out/'all_rotations_before_labels.npz'),code_sha256=sha(__file__),gyro_only_after_initial_gauge=True,truth_not_read_for_predictions=True,ideal_sensor_emulator_privileged=True,no_model_fit_or_weight_scan=True);(out/'prediction_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
# currentactualorientation labels appear only after predictions/seal.
p=np.load(H/'online_RGB_memory_fresh_run_v1/physics.npz');q=p['root_quaternion_world_wxyz'];actual=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();relative=np.transpose(actual[:-1],(0,2,1))@actual[1:];results={}
for name,R in rotations.items():
 error=np.rad2deg(Rotation.from_matrix(np.transpose(actual,(0,2,1))@R).magnitude());w=g[:-1] if name=='previous_held' else g[1:] if name=='endpoint' else .5*(g[:-1]+g[1:]);delta=np.array([exponential(v*dt) for v,dt in zip(w,np.diff(t))]);perstep=np.rad2deg(Rotation.from_matrix(np.transpose(relative,(0,2,1))@delta).magnitude());results[name]=dict(integrated_RMS_deg=float(np.sqrt(np.mean(error*error))),integrated_peak_deg=float(error.max()),event190_error_deg=float(error[950]),step_RMS_deg=float(np.sqrt(np.mean(perstep*perstep))),step_peak_deg=float(perstep.max()))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),results=results,oldmodel_not_changed_or_readmitted=True,not_native_sensor_validation=True,development_not_fresh=True)
(out/'assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
