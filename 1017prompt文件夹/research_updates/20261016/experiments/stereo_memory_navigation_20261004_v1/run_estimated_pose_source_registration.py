from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from rolling_pose_bridge import RollingPoseBridge
from dense_source_points import point_packet
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;old=H/'cad_complete_baseline_current_run';D=H/'estimated_pose_source_registration';D.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
# Single initial gauge only; no further position or quaternion lookup.
cal=np.load(old/'offline_truth/frame_000.npz');q=cal['root_quaternion_world_wxyz'];bridge=RollingPoseBridge(cal['root_position_world_m'],Rotation.from_quat(q[[1,2,3,0]]).as_matrix());cal.close()
p=np.load(H/'frozen_rolling_prior_domain/cad_complete_baseline_current_run_predictions.npz');s=np.load(H/'frozen_rolling_prior_domain/cad_complete_baseline_current_run_sensors.npz')
from rolling_shadow import OnlineRollingShadow
replay=OnlineRollingShadow()
for i,t in enumerate(s['time_s']):
 replay.consume(dict(time_s=float(t),gyro_body_rad_s=s['gyro_body_rad_s'][i],main_encoder_rad_s=float(s['main_encoder_rad_s'][i])),s['initial_normal'] if i==0 else None)
 np.testing.assert_array_equal(replay.records[-1]['position'],p['position'][i]);np.testing.assert_array_equal(replay.records[-1]['rotation'],p['rotation'][i]);bridge.consume(replay.records[-1])
inputs=[old/'offline_truth/frame_000.npz',H/'frozen_rolling_prior_domain/cad_complete_baseline_current_run_predictions.npz',H/'frozen_rolling_prior_domain/cad_complete_baseline_current_run_sensors.npz',H/'rolling_pose_bridge.py',H/'rolling_shadow.py',H/'rolling_odometry.py',H/'dense_source_points.py',H/'run_estimated_pose_source_registration.py',H.parent.parent/'docs/superpowers/specs/2026-10-07-estimated-pose-source-registration-design.md']
inputs+=sorted((old/'source_predictions').glob('*.npz'))+sorted((old/'masks').glob('*.npz'))
(D/'protocol.json').write_text(json.dumps(dict(source_sha256={str(f):sha(f) for f in inputs},maximum_sources=71,historical_not_fresh=True,initial_calibration_privileged=True,dynamic_truth_not_delivered=True),indent=2)+'\n')
for f in sorted((old/'source_predictions').glob('*.npz')):
 prediction=dict(np.load(f));i=int(prediction['frame']);t=float(prediction['time_s']);mask=np.load(old/'masks'/f'frame_{i:03d}.npz');pose=bridge.at(t)
 known={k:pose[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']}
 pred={k:prediction[k] for k in ['depth_m','confidence','frame','time_s']};pred['disparity_native_px']=restore_native(prediction['disparity_px'])
 packet=point_packet(pred,known,mask['left'],mask['right']);packet['pose_estimated']=True;packet['initial_gauge_privileged']=True;packet['pose_sigma_m']=0.;packet['pose_sigma_assumption_not_certified']=True
 np.savez_compressed(D/f'source_{i:03d}.npz',**packet)
(D/'prediction_completion.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all_packets_saved_before_comparison=True,outputs_sha256={str(f):sha(f) for f in D.glob('source_*.npz')},sources=71,not_online=True),indent=2)+'\n');print('ESTIMATED_SOURCE_REGISTRATION_COMPLETE',len(list(D.glob('source_*.npz'))))
