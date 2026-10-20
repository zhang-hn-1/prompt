from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
from imu_gravity_rolling import GravityRolling
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'original_E1_tube_viability_sparse_memory_navigation_pose_assessment_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
config=json.loads((H/'original_E1_tube_viability_navigation_protocol_v1.json').read_text());status=json.loads((H/'original_E1_tube_viability_navigation_stage_status_v1.json').read_text());assert status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and status['exit_code']==0 and not status['wall_timeout']
for group in ['source_sha256','input_sha256']:
 for f,s in config[group].items():assert sha(f)==s,f
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f
rows=json.loads((D/'native_imu_packets.json').read_text());reads=json.loads((D/'native_imu_reads.json').read_text());live=json.loads((D/'native_predictions.json').read_text());gauge=json.loads((D/'initial_gauge.json').read_text());identity=json.loads((D/'native_imu_identity.json').read_text());assert not identity['orientation_field_read'] and len(rows)==len(reads)==len(live)==len(np.load(D/'physics.npz')['simulation_time_s'])
qualified=all(r['qualified'] for r in reads);available=all(r is not None for r in rows)
assert qualified==available
if not available:
 (O/'assessment.json').write_text(json.dumps(dict(decision='REJECT_NATIVE_PACKET_AVAILABILITY',qualified=qualified,not_navigation=True,whole_research_complete=False),indent=2)+'\n');raise SystemExit('packet contract failed, no fallback')
def predict(packets):
 model=GravityRolling(identity['initial_normal']);p0=np.asarray(gauge['position']);R0=np.asarray(gauge['rotation']);P=[];R=[]
 for packet in packets:
  a=model.update(packet);p=p0+R0@a['position'];p[2]=.4000000059604645;P.append(p);R.append(R0@a['rotation'])
 return np.asarray(P),np.asarray(R)
p,R=predict(rows);np.testing.assert_array_equal(p,[r['position_world'] for r in live]);np.testing.assert_array_equal(R,[r['rotation_world'] for r in live]);assert all(r['live_before_next_action'] for r in live)
short=predict(rows[:501]);poison=json.loads(json.dumps(rows))
for row in poison[501:]:row['gyro_body_rad_s']=[999,888,777];row['linear_acceleration_body_m_s2']=[111,222,333];row['main_encoder_rad_s']=999
modified=predict(poison)
for full,prefix,replacement in zip((p,R),short,modified):np.testing.assert_array_equal(full[:501],prefix);np.testing.assert_array_equal(full[:501],replacement[:501])
np.savez_compressed(O/'predictions_pretruth.npz',position_world=p,rotation_world=R,time_s=[r['time_s'] for r in rows]);(O/'prediction_seal_pretruth.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prediction_sha256=sha(O/'predictions_pretruth.npz'),live_exact=True,future501_exact=True,code_sha256=sha(__file__),initial_gauge_privileged=True,flat_height_is_prior=True),indent=2)+'\n')
z=np.load(D/'physics.npz');t=z['simulation_time_s'];np.testing.assert_array_equal(t,[r['time_s'] for r in rows]);mask=t>=1;error=np.linalg.norm(p-z['root_position_world_m'],axis=1);gt=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();angle=np.degrees(Rotation.from_matrix(R@gt.transpose(0,2,1)).magnitude());rms=float(np.sqrt(np.mean(error[mask]**2)));peak=float(error[mask].max());p90=float(np.percentile(angle[mask],90));gates=dict(complete=True,rms=rms<=.015,peak=peak<=.035,attitude=p90<=2.)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_NATIVE_CURRENT_MOTION_SENSOR_LAYER_ONLY' if all(gates.values()) else 'REJECT_NATIVE_CURRENT_MOTION_LAYER',gates=gates,rows=len(t),evaluated=int(mask.sum()),translation_rms_m=rms,translation_peak_m=peak,attitude_p90_deg=p90,interval_angle_p90_deg=[float(np.percentile(angle[(t>=lo)&(t<=hi)],90)) for lo,hi in [(1,5),(5,10),(10,14)]],old_strict_reject_preserved=True,not_navigation=True,conditional_gated120_TaskH_sequence=True,not_original35=True,newfit=False,not_source_registration_pose=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
