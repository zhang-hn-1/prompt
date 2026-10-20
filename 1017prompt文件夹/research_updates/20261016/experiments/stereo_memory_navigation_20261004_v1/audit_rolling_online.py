"""Actual post-exit ideal-sensor shadow and complete frozen B0 comparison."""
from pathlib import Path
import datetime,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    protocol=json.loads((H/'rolling_online_protocol.json').read_text());status=json.loads((H/'rolling_online_status.json').read_text());assert status['exit_code']==0
    out=H/'rolling_online_run';rows=json.loads((out/'online_rolling_predictions.json').read_text());sensor=json.loads((out/'ideal_sensor_inputs.json').read_text());started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    for file,value in protocol['source_sha256'].items():assert sha(file)==value,file
    for file,value in protocol['baseline_artifacts'].items():assert sha(file)==value,file
    manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())
    for file,value in manifest['artifact_sha256'].items():assert sha(file)==value,file
    z=np.load(out/'physics.npz');old=np.load(H/'oracle_rgb_shadow_run/physics.npz');q=z['root_quaternion_world_wxyz'];R=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();t=z['simulation_time_s']
    assert len(rows)==len(sensor)==len(t);assert np.array_equal([r['time_s'] for r in rows],t)
    assert all(set(r)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'} for r in sensor)
    # Actual online estimator consumed only white-listed fields; no replay execution here.
    pe=np.linalg.norm(np.array([r['position'] for r in rows])-(z['root_position_world_m']-z['root_position_world_m'][0])@R[0],axis=1)
    relative=R[0].T[None]@R;estimate=np.array([r['rotation'] for r in rows]);delta=np.transpose(relative,(0,2,1))@estimate
    ae=np.degrees(np.arccos(np.clip((np.trace(delta,axis1=1,axis2=2)-1)/2,-1,1)));valid=t>=1.
    rms=float(np.sqrt(np.mean(pe[valid]**2)));peak=float(pe[valid].max());p90=float(np.percentile(ae[valid],90));gates=dict(rms=rms<=.05,peak=peak<=.1,attitude=p90<=2)
    equality={k:bool(old[k].shape==z[k].shape and np.array_equal(old[k],z[k],equal_nan=True)) for k in old.files if not k.startswith('raw_simulation_time') and k!='raw_physics_step_index'}
    raw_delta=None
    if old['raw_simulation_time_s'].shape==z['raw_simulation_time_s'].shape:raw_delta=np.unique(z['raw_simulation_time_s']-old['raw_simulation_time_s']).tolist()
    errors={k:(float(np.max(abs(old[k]-z[k]))) if old[k].shape==z[k].shape else None) for k in equality if not equality[k]}
    metrics=json.loads((out/'metrics.json').read_text());contacts=json.loads((out/'physics_contacts.json').read_text());identity=json.loads((out/'rolling_live_identity.json').read_text())
    result=dict(checked_utc=started,actual_run_status=status,actual_online_rows=len(rows),sensor_rows=len(sensor),physics_rows=len(t),evaluated_samples=int(valid.sum()),ideal_pose_gates=gates,ideal_pose_pass=all(gates.values()),translation_rms_m=rms,translation_peak_m=peak,attitude_p90_deg=p90,
        baseline_exact_nonraw=equality,baseline_exact_nonraw_all=all(equality.values()),baseline_mismatch_maximum=errors,raw_time_delta=raw_delta,metrics=metrics,contact_records=len(contacts),live_sensor_identity=identity,source_count=len(protocol['source_sha256']),postclose_artifact_count=len(manifest['artifact_sha256']),
        physical_episodes=1,formal_e0e1e2_matrix_run=False,rgb_depth_gate_pass=False,estimated_pose_consumed_by_controller=False,real_imu_noise_bias_tested=False,fresh_environment=False,whole_research_complete=False)
    np.savez_compressed(H/'rolling_online_measurement.npz',time_s=t,position_error_m=pe,attitude_error_deg=ae,predicted_position_gauge=np.array([r['position'] for r in rows]),actual_position_gauge=(z['root_position_world_m']-z['root_position_world_m'][0])@R[0])
    (H/'rolling_online_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ONLINE_SHADOW_ASSESSMENT',len(rows),rms,peak,p90,all(equality.values()),errors,flush=True)
if __name__=='__main__':main()
