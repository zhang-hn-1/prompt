"""Current complexB0 native interval pose and exact unchanged control, no replay."""
from pathlib import Path
import datetime,json,hashlib
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'imu_complex_online_protocol.json').read_text());s=json.loads((H/'imu_complex_online_status.json').read_text());assert s['status']=='finished' and s['exit_code']==0;out=H/'imu_complex_online_run';m=json.loads((out/'artifact_manifest_post_close.json').read_text());checks=0
 for obj,key in [(p,'source_sha256'),(p,'baseline_artifacts'),(m,'artifact_sha256')]:
  for f,v in obj[key].items():assert sha(f)==v,f;checks+=1
 z=np.load(out/'physics.npz');old=np.load(H/'native_imu_run/physics.npz');t=z['simulation_time_s'];reads=json.loads((out/'native_imu_reads.json').read_text());packets=json.loads((out/'native_imu_packets.json').read_text());pred=json.loads((out/'native_imu_predictions.json').read_text());assert len(reads)==len(packets)==len(pred)==len(t);assert np.array_equal([r['time_s'] for r in pred],t)
 for packet in packets:
  if packet is not None:assert set(packet)=={'time_s','gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s'} and all(np.isfinite(np.asarray(v)).all() for v in packet.values())
 assert all(not r['orientation_read'] for r in reads)
 availability=np.array([r['available'] for r in pred]);qualified=np.array([r['qualified'] for r in reads]);assert np.array_equal(availability,qualified);interface=all(r['qualified'] or (r['time_s']<=.04 and r['reason']=='invalid') for r in reads)
 equality={k:bool(old[k].shape==z[k].shape and np.array_equal(old[k],z[k],equal_nan=True)) for k in old.files if k not in ['raw_simulation_time_s','raw_physics_step_index']};assert len(equality)==17
 R=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();truth=(z['root_position_world_m']-z['root_position_world_m'][0])@R[0];trueR=R[0].T[None]@R;valid=t>=1.;ev=valid&availability;complete=bool(availability[valid].all());pe=np.full(len(t),np.nan);ae=np.full(len(t),np.nan);position=np.full((len(t),3),np.nan)
 if availability.any():
  est=np.array([r['rotation'] for r in pred if r['available']]);position[availability]=np.array([r['position'] for r in pred if r['available']]);pe[availability]=np.linalg.norm(position[availability]-truth[availability],axis=1);ae[availability]=np.degrees(Rotation.from_matrix(np.transpose(trueR[availability],(0,2,1))@est).magnitude())
 rms=float(np.sqrt(np.mean(pe[ev]**2))) if ev.any() else None;peak=float(pe[ev].max()) if ev.any() else None;p90=float(np.percentile(ae[ev],90)) if ev.any() else None;gates=dict(complete=complete,rms=rms is not None and rms<=.05,peak=peak is not None and peak<=.1,attitude=p90 is not None and p90<=2.)
 intervals=np.array([r['end_raw_s']-r['begin_raw_s'] for r in reads]);stamps=np.array([abs(r['sensor_begin_stamp_s']-r['begin_raw_s']) for r in reads]);assert sha(H/'imu_gravity_rolling.py')==p['candidate_model_sha256'] and sha(H/'imu_interval_packet.py')==p['packet_source_sha256']
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_status=s,integrity_checks=checks,source_count=len(p['source_sha256']),postclose_artifacts=len(m['artifact_sha256']),physics_rows=len(t),evaluated_denominator=int(valid.sum()),evaluated_available=int(ev.sum()),interval_qualification_pass=interface,unqualified_reasons=dict(Counter(r['reason'] for r in reads if not r['qualified'])),translation_rms_m=rms,translation_peak_m=peak,attitude_p90_deg=p90,pose_gates=gates,pose_pass=bool(interface and all(gates.values()) and all(equality.values())),baseline_exact_nonraw=equality,baseline_exact_nonraw_all=all(equality.values()),raw_time_delta=np.unique(z['raw_simulation_time_s']-old['raw_simulation_time_s']).tolist() if len(t)==len(old['simulation_time_s']) else None,interval_dt_error_peak_s=float(np.max(abs(intervals-.02))),begin_stamp_error_peak_s=float(stamps.max()),metrics=json.loads((out/'metrics.json').read_text()),new_model_fit=False,same_frozen_model=True,model_control_consumption=False,rgb_source_gate=False,fresh_validation=False,formal_e0e1e2=False,whole_research_complete=False)
 np.savez_compressed(H/'imu_complex_online_measurement.npz',time_s=t,available=availability,position_error_m=pe,attitude_error_deg=ae,predicted_position=position,actual_position=truth)
 (H/'imu_complex_online_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:result[k] for k in ['physics_rows','evaluated_denominator','evaluated_available','interval_qualification_pass','translation_rms_m','translation_peak_m','attitude_p90_deg','pose_pass','baseline_exact_nonraw_all','raw_time_delta']},indent=2))
if __name__=='__main__':main()
