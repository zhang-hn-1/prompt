"""Post-close sensor/estimated-pose/source/actor reconstruction, then truth scoring."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row
from flat_ground_pose_bridge import FlatGroundPoseBridge
from dense_source_points import point_packet
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
st=json.loads((H/'flat_ground_pose_stage_status_v1.json').read_text());assert st['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and st['returncode']==0
seal=D/'artifact_manifest_post_close.json';assert sha(seal)==st['post_close_manifest_sha256'];manifest=json.loads(seal.read_text());protocol=json.loads((H/'flat_ground_pose_protocol_v1.json').read_text())
for rows in [protocol['source_sha256'],manifest['artifact_sha256']]:
 for f,s in rows.items():assert sha(f)==s,f
inputs=json.loads((D/'ideal_rolling_sensors.json').read_text());pred=json.loads((D/'rolling_predictions.json').read_text());init=json.loads((D/'initial_gauge.json').read_text());ph=np.load(D/'physics.npz');times=ph['simulation_time_s'];assert len(inputs)==len(pred)==len(times)
replay=OnlineRollingShadow();bridge=FlatGroundPoseBridge(init['position'],init['rotation']);positions=[];rotations=[]
for i,(sensor,row) in enumerate(zip(inputs,pred)):
 expected,normal=emulate_sensor_row({k:ph[k][i:i+1] for k in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']})
 assert set(sensor)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'}
 for k in expected:np.testing.assert_equal(expected[k],sensor[k])
 replay.consume(sensor,normal if i==0 else None)
 for k in row:np.testing.assert_equal(row[k],replay.records[-1][k])
 bridge.consume(row);estimate=bridge.at(sensor['time_s']);positions.append(estimate['root_position_world_m']);rotations.append(estimate['root_rotation_world'])
clocks=json.loads((D/'camera_clocks.json').read_text());policy=json.loads((D/'policy_records.json').read_text());task=json.loads((D/'task_episode.json').read_text())['actual_episode'];goalworld=np.asarray(task['goal_position_world_m']);prev=[];posefields=sourcefields=obsfields=0
for clock in clocks:
 i=clock['frame'];t=clock['time_s'];estimate=bridge.at(t);saved=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz')
 for k,v in estimate.items():np.testing.assert_equal(saved[k],v);posefields+=1
 assert bool(saved['flat_ground_height_prior']) and not bool(saved['height_prior_is_observation']) and not bool(saved['full_3d_certified'])
 if i%5==0:
  p=dict(np.load(D/'source_predictions'/f'frame_{i:03d}.npz'));mask=np.load(D/'masks'/f'frame_{i:03d}.npz');p0={k:p[k] for k in ['depth_m','confidence','frame','time_s']};p0['disparity_native_px']=restore_native(p['disparity_px']);pose={k:estimate[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']};packet=point_packet(p0,pose,mask['left'],mask['right']);save=np.load(D/'sensor_packets'/f'source_{i:03d}.npz')
  for k,v in packet.items():np.testing.assert_equal(save[k],v);sourcefields+=1
 if i<len(policy):
  op=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz');history=np.load(D/'history'/f'frame_{i:03d}.npz');R=estimate['root_rotation_world'];sensor=inputs[int(round(t/.02))];q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
  if prev and np.dot(q,prev[-1][1])<0:q=-q
  row=np.r_[estimate['velocity_world_m_s'],R@np.asarray(sensor['gyro_body_rad_s']),-R.T@np.array([0,0,1]),history['proprio_history'][-1,9:17]];prev.append((row,q));prev=prev[-5:];prop=np.asarray([prev[0][0]]*(5-len(prev))+[x[0] for x in prev]);quat=np.asarray([prev[0][1]]*(5-len(prev))+[x[1] for x in prev]);yaw=estimate['yaw_world_rad'];c,s=np.cos(yaw),np.sin(yaw);goal=np.array([[c,s],[-s,c]])@(goalworld[:2]-estimate['root_position_world_m'][:2])
  for k,v in dict(proprio_history=prop.astype(np.float32),quaternion_history=quat.astype(np.float32),position_world_xy=estimate['root_position_world_m'][:2],yaw_world_rad=yaw,pointgoal_base_xy=goal,action_history=history['action_history']).items():np.testing.assert_equal(op[k],v);obsfields+=1
  l=policy[i]['local'];assert l['pose_estimated'] and not l['pose_privileged'] and l['initial_gauge_privileged'] and not l['height_prior_is_observation']
# Scoring labels after complete estimator/source/actor reconstruction. Sensor emulation above is a declared privileged boundary.
positions=np.asarray(positions);rotations=np.asarray(rotations);truth=ph['root_position_world_m'];Rgt=Rotation.from_quat(ph['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();e=positions-truth;norm=np.linalg.norm(e,axis=1);dr=Rgt.transpose(0,2,1)@rotations;ae=np.degrees(np.arccos(np.clip((np.trace(dr,axis1=1,axis2=2)-1)/2,-1,1)));valid=times>=1.;rms=float(np.sqrt((norm[valid]**2).mean()));peak=float(norm[valid].max());ap=float(np.percentile(ae[valid],90));posepass=rms<=.05 and peak<=.1 and ap<=2.
# Reconstruct a truncated estimator using only available sensor rows / initial gauge.
prefixn=min(501,len(inputs));prefix=OnlineRollingShadow();pb=FlatGroundPoseBridge(init['position'],init['rotation'])
for i in range(prefixn):
 _,n0=emulate_sensor_row({k:ph[k][0:1] for k in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']}) if i==0 else (None,None)
 prefix.consume(inputs[i],n0 if i==0 else None);pb.consume(prefix.records[-1]);np.testing.assert_equal(pb.at(inputs[i]['time_s'])['root_position_world_m'],positions[i])
try:pb.at(float(inputs[prefixn]['time_s']) if prefixn<len(inputs) else float(inputs[-1]['time_s'])+.02)
except ValueError:pass
else:raise AssertionError('future exposure accepted')
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pose_gate=posepass,rms3D_m=rms,peak3D_m=peak,attitude_p90_deg=ap,gate_samples=int(valid.sum()),pose_fields_exact=posefields,source_fields_exact=sourcefields,actor_observation_fields_exact=obsfields,sensor_samples_exact=len(inputs),rolling_prediction_samples_exact=len(pred),prefix_sensor_samples_exact=prefixn,future_pose_query_rejected=True,source_identities=len(protocol['source_sha256']),sealed_artifacts=len(manifest['artifact_sha256']),initial_gauge_privileged=True,ideal_bodygyro_emulator_privileged=True,not_native_IMU_or_pure_RGB=True,height_prior_not_observation=True,full_3d_certified=False,old_rolling_reject_preserved=True,no_recursive_physics_prediction=True,whole_research_complete=False,code_sha256=sha(__file__))
file=H/'flat_ground_pose_sensor_observation_verification_v1.json';assert not file.exists();file.write_text(json.dumps(out,indent=2)+'\n');np.savez_compressed(H/'flat_ground_pose_errors_posthoc_v1.npz',time_s=times,estimated_world_position=positions,error3D_m=norm,attitude_error_deg=ae,valid=valid);print(json.dumps(out,indent=2))
