"""Historical sensor-only replay; not an online run or native IMU claim."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row
H=Path(__file__).resolve().parent;D=H/'controlled_curve_frozen_pose_audit_v1';names=['tracking_tube_D1_run_v1'];sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();source={str(H/n/'physics.npz'):sha(H/n/'physics.npz') for n in names}
for p in [H/'rolling_odometry.py',H/'rolling_shadow.py',H/'run_controlled_curve_frozen_pose_audit_v1.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-controlled-curve-frozen-pose-audit.md']:source[str(p)]=sha(p)
(D/'protocol.json').write_text(json.dumps(dict(utc=now(),source_sha256=source,sequences=names,maximum_sequences=1,historical_not_fresh=True,old_frozen_model=True,gate=dict(rms3D_m=.05,peak3D_m=.1,attitude_p90_deg=2,evaluate_t_min_s=1.)),indent=2)+'\n')
for name in names:
 ph=np.load(H/name/'physics.npz');times=ph['simulation_time_s'];model=OnlineRollingShadow();sensors=[];initial=None
 for i,t in enumerate(times):
  sensor,normal=emulate_sensor_row({k:ph[k][i:i+1] for k in ('simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s')})
  assert float(sensor['time_s'])==float(t) and set(sensor)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'}
  if i==0:initial=normal
  model.consume(sensor,initial if i==0 else None);sensors.append(sensor)
 np.savez_compressed(D/(name+'_sensors.npz'),time_s=times,gyro_body_rad_s=np.array([s['gyro_body_rad_s'] for s in sensors]),main_encoder_rad_s=np.array([s['main_encoder_rad_s'] for s in sensors]),initial_normal=initial)
 np.savez_compressed(D/(name+'_predictions.npz'),time_s=times,position=np.array([r['position'] for r in model.records]),rotation=np.array([r['rotation'] for r in model.records]))
 print('FROZEN_DOMAIN_PREDICTED',name,len(times),flush=True)
(D/'prediction_completion.json').write_text(json.dumps(dict(utc=now(),position_truth_not_read=True,source_sha256=source,prediction_sha256={str(f):sha(f) for f in D.glob('*_predictions.npz')}),indent=2)+'\n')
