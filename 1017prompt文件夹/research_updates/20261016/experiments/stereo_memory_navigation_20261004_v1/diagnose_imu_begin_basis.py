from pathlib import Path
import json,datetime,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;z=np.load(H/'native_imu_run/physics.npz');rows=json.loads((H/'native_imu_run/native_imu_reads.json').read_text());R=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();g=np.array([r['gyro_body_rad_s'] for r in rows]);omega=z['angular_velocity_world_rad_s'];v=z['simulation_time_s'][1:]>=1;result={}
for tag,rots,om in [('currentbasis_currentomega',R[1:],omega[1:]),('beginbasis_currentomega',R[:-1],omega[1:]),('currentbasis_previousomega',R[1:],omega[:-1])]:
 e=np.linalg.norm(g[1:]-(np.transpose(rots,(0,2,1))@om[...,None])[...,0],axis=1)[v];result[tag]=dict(samples=len(e),rms=float(np.sqrt(np.mean(e*e))),p90=float(np.percentile(e,90)),peak=float(e.max()))
result.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),role='post-rejection basis diagnostics only, no predictor replay or old gate change',new_pose_score=False,raw_inputs_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [H/'native_imu_run/physics.npz',H/'native_imu_run/native_imu_reads.json']})
(H/'imu_begin_basis_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
