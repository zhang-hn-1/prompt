"""Audit estimatedpose control using sensor replay; GT only in final scoring."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row
from rolling_pose_bridge import RollingPoseBridge
from dense_source_points import point_packet
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;D=H/'estimated_pose_local_current_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'estimated_pose_local_navigation_status.json').read_text());assert status['status']=='finished';protocol=json.loads((H/'estimated_pose_local_navigation_protocol.json').read_text());manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())
for rows in [protocol['source_sha256'],manifest['artifact_sha256']]:
 for f,s in rows.items():assert sha(f)==s,f
inputs=json.loads((D/'ideal_sensor_inputs.json').read_text());pred=json.loads((D/'online_rolling_predictions.json').read_text());live=json.loads((D/'rolling_live_identity.json').read_text());init=json.loads((D/'initial_gauge.json').read_text());replay=OnlineRollingShadow();bridge=RollingPoseBridge(init['position'],init['rotation']);ph=np.load(D/'physics.npz');times=ph['simulation_time_s'];assert len(inputs)==len(pred)==len(times)
for i,(sensor,row) in enumerate(zip(inputs,pred)):
 expected,n=emulate_sensor_row({k:ph[k][i:i+1] for k in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']})
 for k in expected:np.testing.assert_array_equal(expected[k],sensor[k])
 replay.consume(sensor,live['initial_normal'] if i==0 else None)
 for k in row:np.testing.assert_array_equal(row[k],replay.records[-1][k])
 bridge.consume(row)
clocks=json.loads((D/'camera_clocks.json').read_text());policy=json.loads((D/'policy_records.json').read_text());task=json.loads((D/'task_episode.json').read_text())['actual_episode'];worldgoal=np.array(task['goal_position_world_m']);prev=[];pose_fields=0;source_fields=0;observation_fields=0
for clock in clocks:
 i=clock['frame'];t=clock['time_s'];estimate=bridge.at(t);saved=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz')
 for k,v in estimate.items():np.testing.assert_array_equal(saved[k],v);pose_fields+=1
 if i%5==0:
  p=dict(np.load(D/'source_predictions'/f'frame_{i:03d}.npz'));mask=np.load(D/'masks'/f'frame_{i:03d}.npz');p0={k:p[k] for k in ['depth_m','confidence','frame','time_s']};p0['disparity_native_px']=restore_native(p['disparity_px']);pose={k:estimate[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']};packet=point_packet(p0,pose,mask['left'],mask['right']);sensitivity=.1+np.linalg.norm(packet['world']-estimate['root_position_world_m'],axis=1)*np.sin(np.deg2rad(2.));packet['sigma_m']+=sensitivity
  save=np.load(D/'sensor_packets'/f'source_{i:03d}.npz')
  for k,v in packet.items():np.testing.assert_array_equal(save[k],v);source_fields+=1
  assert bool(save['pose_estimated']) and bool(save['initial_gauge_privileged']) and bool(save['pose_sensitivity_not_certified'])
 if i<len(policy):
  op=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz');history=np.load(D/'history'/f'frame_{i:03d}.npz');R=estimate['root_rotation_world'];sensor=inputs[int(round(t/.02))];q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
  if prev and np.dot(q,prev[-1][1])<0:q=-q
  row=np.r_[estimate['velocity_world_m_s'],R@np.array(sensor['gyro_body_rad_s']),-R.T@np.array([0,0,1]),history['proprio_history'][-1,9:17]];prev.append((row,q));prev=prev[-5:];proprio=np.array([prev[0][0]]*(5-len(prev))+[r[0] for r in prev]);quat=np.array([prev[0][1]]*(5-len(prev))+[r[1] for r in prev]);yaw=estimate['yaw_world_rad'];c,s=np.cos(yaw),np.sin(yaw);goal=np.array([[c,s],[-s,c]])@(worldgoal[:2]-estimate['root_position_world_m'][:2])
  np.testing.assert_array_equal(op['proprio_history'],proprio.astype(np.float32));np.testing.assert_array_equal(op['quaternion_history'],quat.astype(np.float32));np.testing.assert_array_equal(op['position_world_xy'],estimate['root_position_world_m'][:2]);np.testing.assert_array_equal(op['yaw_world_rad'],yaw);np.testing.assert_array_equal(op['pointgoal_base_xy'],goal);np.testing.assert_array_equal(op['action_history'],history['action_history']);observation_fields+=6
  assert policy[i]['local']['source_layer']=='RGB_depth_idealgyro_encoder_estimatedpose' and not policy[i]['local']['pose_privileged']
# Labels first decoded after all control/source reconstruction is complete.
Rgt=Rotation.from_quat(ph['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();truth=(ph['root_position_world_m']-ph['root_position_world_m'][0])@Rgt[0];pp=np.array([r['position'] for r in pred]);Rp=np.array([r['rotation'] for r in pred]);e=pp-truth;norm=np.linalg.norm(e,axis=1);dr=(Rgt[0].T[None]@Rgt).transpose(0,2,1)@Rp;ae=np.degrees(np.arccos(np.clip((np.trace(dr,axis1=1,axis2=2)-1)/2,-1,1)));v=times>=1;rms=float(np.sqrt(np.mean(norm[v]**2)));peak=float(norm[v].max());ap=float(np.percentile(ae[v],90));metrics=json.loads((D/'metrics.json').read_text());position=ph['root_position_world_m'][:,:2];start=np.array(task['start_position_world_m'][:2]);delta=worldgoal[:2]-start;unit=delta/np.linalg.norm(delta);relative=position-start;progress=relative@unit;lateral=np.abs(relative[:,0]*unit[1]-relative[:,1]*unit[0]);movement=times>=6.-1e-6;cross_rms=float(np.sqrt(np.mean(lateral[movement]**2)));cross_peak=float(lateral[movement].max());taskpass=metrics['success']==1 and metrics['collision']==0 and metrics['runtime_error']==0 and metrics['elapsed_s']<=35 and metrics['final_distance_m']<=.2;geompass=cross_rms<=.12 and cross_peak<=.30
assert len(policy)==metrics['action_blocks'] and len(clocks)==len(policy)+1 and len(times)==len(policy)*5+1
out=dict(author_only=True,source_verified=len(protocol['source_sha256']),artifacts_verified=len(manifest['artifact_sha256']),pose_fields_verified=pose_fields,source_fields_verified=source_fields,observation_fields_verified=observation_fields,sensor_prediction_exact=True,task_success_gate=taskpass,geometric_tracking_gate=geompass,cross_rms_m=cross_rms,cross_peak_m=cross_peak,actual_goal_line_progress_m=float(progress[-1]),rms3D_m=rms,peak3D_m=peak,attitude_p90_deg=ap,gate_samples=int(v.sum()),pose_gate=rms<=.05 and peak<=.1 and ap<=2,navigation_metrics={k:v for k,v in metrics.items() if not isinstance(v,list)},initial_gauge_privileged=True,ideal_gyro_encoder_not_native_IMU=True,low_level_PID_measured_roll_privileged=True,source_pose_sensitivity_not_certified=True,not_pure_RGB=True,not_formal_E1=True,whole_research_complete=False)
(H/'estimated_pose_local_assessment.json').write_text(json.dumps(out,indent=2)+'\n');np.savez_compressed(D/'errors_posthoc.npz',time_s=times,position_world_xy=position,predicted_gauge=pp,truth_gauge=truth,error3D_m=norm,attitude_error_deg=ae,goal_line_progress_m=progress,lateral_error_m=lateral);print(json.dumps(out,indent=2))
