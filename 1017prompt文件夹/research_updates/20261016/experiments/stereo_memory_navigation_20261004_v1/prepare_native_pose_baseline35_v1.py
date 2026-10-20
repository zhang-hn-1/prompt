from pathlib import Path
import json,hashlib,ast
H=Path(__file__).resolve().parent
src=(H/'run_sgbm_baseline35_remaining_v1.py').read_text()
src=src.replace('from stereo_geometry import SGBMStereo','from native_pose_consumer_v1 import current_native_pose\nfrom stereo_geometry import SGBMStereo')
src=src.replace("choices=['e0','e1']","choices=['e0']").replace("a.name='sgbm_baseline35_'+a.condition+'_coverage'","a.name='native_pose_baseline35_e0'")
src=src.replace("('sgbm_baseline35_'+a.condition+'_coverage_protocol_v1.json')","'native_pose_baseline35_e0_protocol_v1.json'")
src=src.replace("('sgbm_baseline35_'+a.condition+'_coverage_run_v1')","'native_pose_baseline35_e0_run_v1'")
src=src.replace('frozen_classical_SGBM_binocular_memory_rollPID_knownpose_TaskH_35s','frozen_classical_SGBM_binocular_memory_rollPID_nativeIMUencoder_TaskH_35s')
src=src.replace("only_source_change='frozen_SGBM_instead_of_frozen_FastFoundationStereo'","only_source_change='none_SGBM_frozen',only_pose_change='native_gyro_encoder_current_for_registration_and_navigation_initial_gauge_privileged'")
src=src.replace('current_GT_pose_registration=True','current_GT_pose_registration=False').replace('source_GT_pose_registration=True','source_GT_pose_registration=False').replace('known_GT_pose_control=True','known_GT_pose_control=False')
old="source_poses={eye+'_camera_usd_world':get_cam_pose(stage,path) for eye,path in [('left',LEFT_CAMERA_PATH),('right',RIGHT_CAMERA_PATH)]}\n                rootT=np.eye(4);rootT[:3,3]=position"
new="native_pose=current_native_pose(native,float(step.simulation_time_s))\n                source_poses={eye+'_camera_usd_world':native_pose[eye+'_camera_usd_world'] for eye in ('left','right')}\n                rootT=np.eye(4);rootT[:3,3]=native_pose['root_position_world_m']"
assert old in src;src=src.replace(old,new)
old="R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix()\n            pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']))\n            gyro=R.T@np.asarray(runtime._robot.get_angular_velocity())"
new="pose=current_native_pose(imu_predictions[-1],t)\n            packet_now=imu_packets[-1];assert packet_now is not None and abs(float(packet_now['time_s'])-t)<1e-8\n            gyro=np.asarray(packet_now['gyro_body_rad_s'])"
assert old in src;src=src.replace(old,new)
src=src.replace('known_GT_pose=True,native_pose_used=False','known_GT_pose=False,native_pose_used=True')
src=src.replace('predictions_used_by_control=False','predictions_used_by_control=True')
src=src.replace("role='ONE fixed classical matcher trial; same initial handoff planner tracker PID speed; knownpose TaskH not strict visual'","role='ONE pose-source trial; registration and navigation current native IMU encoder; initial privileged gauge and ideal lowlevel IMU; not pure RGB' ")
src=src.replace('# Explicit current GT pose upperbound, never GT depth for source selection.','# Native current pose only for registration; GT depth stays post-output offline diagnostic.')
ast.parse(src);(H/'run_native_pose_baseline35_e0_v1.py').write_text(src)
c=json.loads((H/'sgbm_baseline35_e0_coverage_protocol_v1.json').read_text())
c['source_sha256'].pop(str(H/'run_sgbm_baseline35_remaining_v1.py'),None)
for f in [H/'run_native_pose_baseline35_e0_v1.py',H/'native_pose_consumer_v1.py',H/'imu_gravity_rolling.py',H/'imu_interval_packet.py',H/'estimated_navigation_observation.py']:
 c['source_sha256'][str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
c.update(role='one native pose consumer substitution at both registration/navigation; no source/controller change',maximum_actual_sequences=1,scope='E0 known initial gauge flatground safe-start TaskH; not pureRGB or originalstrictbenchmark',comparison='sgbm_baseline35_e0_coverage_run_v1',stop='first physical contact, true XY goal .2m, 35s, unavailable exact current native pose or runtime failure',acceptance='no contact and arrival within35; separately record truthful progress vs same frozen knownpose E0; pose error diagnostic cannot qualify navigation',future_truth_forbidden=True,parameter_scan=False)
(H/'native_pose_baseline35_e0_protocol_v1.json').write_text(json.dumps(c,indent=2)+'\n')
print('prepared',len(c['source_sha256']))
