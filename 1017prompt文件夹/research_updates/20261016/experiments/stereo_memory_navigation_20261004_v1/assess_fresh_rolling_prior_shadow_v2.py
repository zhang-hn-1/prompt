"""Post-close fresh whole-sequence scoring; no controller or predictor feedback."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row
H=Path(__file__).resolve().parent;D=H/'fresh_rolling_prior_shadow_current_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'fresh_rolling_prior_shadow_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0
protocol=json.loads((H/'fresh_rolling_prior_shadow_protocol.json').read_text());manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())
for key,rows in [('source_sha256',protocol['source_sha256']),('artifact_sha256',manifest['artifact_sha256'])]:
 for f,s in rows.items():assert sha(f)==s,f
pred=json.loads((D/'online_rolling_predictions.json').read_text());sensors=json.loads((D/'ideal_sensor_inputs.json').read_text());live=json.loads((D/'rolling_live_identity.json').read_text());ph=np.load(D/'physics.npz');t=ph['simulation_time_s'];assert len(pred)==len(sensors)==len(t)
assert not live['controller_consumes_estimate'] and live['prediction_live'] and live['initial_yaw_deg']==20
# Rebuild after online output was sealed: this only checks causal interface equality.
replay=OnlineRollingShadow()
for i,(row,sensor) in enumerate(zip(pred,sensors)):
 expected,normal=emulate_sensor_row({k:ph[k][i:i+1] for k in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']})
 assert set(sensor)==set(expected)
 for k in sensor:np.testing.assert_array_equal(sensor[k],expected[k])
 assert row['time_s']==float(t[i]);replay.consume(sensor,live['initial_normal'] if i==0 else None)
 for k,v in row.items():np.testing.assert_array_equal(v,replay.records[-1][k])
R=Rotation.from_quat(ph['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();Rp=np.array([r['rotation'] for r in pred]);pp=np.array([r['position'] for r in pred]);p=ph['root_position_world_m'];truth=(p-p[0])@R[0];e=pp-truth;ew=e@R[0].T;err=np.linalg.norm(e,axis=1);dr=(R[0].T[None]@R).transpose(0,2,1)@Rp;ae=np.degrees(np.arccos(np.clip((np.trace(dr,axis1=1,axis2=2)-1)/2,-1,1)));v=t>=1.
rms=float(np.sqrt(np.mean(err[v]**2)));peak=float(err[v].max());ap=float(np.percentile(ae[v],90));metrics=json.loads((D/'metrics.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());policies=json.loads((D/'policy_records.json').read_text());contacts=json.loads((D/'contacts.json').read_text())
assert len(policies)==metrics['action_blocks'] and len(clocks)==len(policies)+1 and len(t)==5*len(policies)+1
assert len(contacts)==len(clocks)
result=dict(online_whole_sequence=True,fresh_no_fit=True,available_duration_s=float(t[-1]),whole_original_episode_terminated=True,coverage_after_termination_not_available=True,source_verified=len(protocol['source_sha256']),artifacts_verified=len(manifest['artifact_sha256']),samples=len(t),gate_samples=int(v.sum()),sensors_and_live_replay_exact=True,controller_consumes_estimate=False,ideal_sensor_not_native_IMU=True,rms3D_m=rms,peak3D_m=peak,attitude_p90_deg=ap,world_xy_rms_m=float(np.sqrt(np.mean(np.sum(ew[v,:2]**2,axis=1)))),world_z_rms_m=float(np.sqrt(np.mean(ew[v,2]**2))),pose_domain_gate_pass=rms<=.05 and peak<=.1 and ap<=2,original_gate=dict(rms3D_m=.05,peak3D_m=.1,attitude_p90_deg=2),navigation_metrics=metrics,contacts=sum(bool(x['contact']) for x in contacts),captures=len(clocks),policy_calls=len(policies),future_suffix_not_read='tested on actual hook and frozen estimator; raw live no future stream delivered',whole_research_complete=False)
np.savez_compressed(D/'pose_errors_posthoc.npz',time_s=t,error_world_m=ew,attitude_error_deg=ae,gate_valid=v,truth_gauge=truth,predicted_gauge=pp)
(H/'fresh_rolling_prior_shadow_assessment_v2.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
