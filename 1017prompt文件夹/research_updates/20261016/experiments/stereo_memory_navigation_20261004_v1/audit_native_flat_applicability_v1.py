"""Historical applicability ONLY; frozen native model plus separately labelled floor prior."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
from imu_gravity_rolling import GravityRolling
H=Path(__file__).resolve().parent;D=H/'imu_complex_online_run';O=H/'native_flat_applicability_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old_protocol=json.loads((H/'imu_complex_online_protocol.json').read_text())
assert sha(H/'imu_gravity_rolling.py')==old_protocol['candidate_model_sha256']
assert sha(H/'imu_interval_packet.py')==old_protocol['packet_source_sha256']
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())
for f,s in manifest['artifact_sha256'].items():assert sha(f)==s,f
packets=json.loads((D/'native_imu_packets.json').read_text());identity=json.loads((D/'native_imu_identity.json').read_text())
assert not identity['orientation_field_read'];assert all(p is not None for p in packets)
# Historical t0 gauge is an explicit declared input, not dynamic pose access.
with np.load(D/'physics.npz') as archive:
 p0=archive['root_position_world_m'][0].copy();q0=archive['root_quaternion_world_wxyz'][0].copy()
R0=Rotation.from_quat(q0[[1,2,3,0]]).as_matrix()
(O/'initial_gauge.json').write_text(json.dumps(dict(position=p0.tolist(),rotation=R0.tolist(),initial_only=True,known_gauge_privileged=True,floor_z_prior=0.,radius_prior=.4000000059604645),indent=2)+'\n')
def predict(rows):
 model=GravityRolling(identity['initial_normal']);positions=[];rotations=[];relative=[]
 for packet in rows:
  r=model.update(packet);relative.append(r['position'].copy());world=p0+R0@r['position'];world[2]=.4000000059604645
  positions.append(world);rotations.append(R0@r['rotation'])
 return np.asarray(positions),np.asarray(rotations),np.asarray(relative)
p,R,rel=predict(packets);prefix=predict(packets[:501]);poison=json.loads(json.dumps(packets))
for row in poison[501:]:row['gyro_body_rad_s']=[999,888,777];row['linear_acceleration_body_m_s2']=[111,222,333];row['main_encoder_rad_s']=999
replaced=predict(poison)
for full,short,modified in zip((p,R,rel),prefix,replaced):
 np.testing.assert_array_equal(full[:501],short);np.testing.assert_array_equal(full[:501],modified[:501])
old_live=json.loads((D/'native_imu_predictions.json').read_text())
np.testing.assert_array_equal(rel,np.asarray([r['position'] for r in old_live]))
np.testing.assert_array_equal(R,np.asarray([R0@np.asarray(r['rotation']) for r in old_live]))
np.savez_compressed(O/'predictions_pretruth.npz',time_s=np.asarray([r['time_s'] for r in packets]),position_world=p,rotation_world=R)
inputs=[D/'native_imu_packets.json',D/'native_imu_identity.json',D/'native_imu_predictions.json',O/'initial_gauge.json',H/'imu_gravity_rolling.py',H/'imu_interval_packet.py',H/'rolling_odometry.py',H.parent.parent/'docs/superpowers/plans/2026-10-08-native-gravity-flat-prior-applicability.md']
(O/'prediction_seal_pretruth.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(f):sha(f) for f in inputs},code_sha256=sha(__file__),prediction_sha256=sha(O/'predictions_pretruth.npz'),future501_exact=True,old_live_relative_exact=True,historical_seen=True,flat_height_is_prior_not_observation=True),indent=2)+'\n')
# Dynamic labels are inspected only after prediction/seal. Old strict result not changed.
z=np.load(D/'physics.npz');t=z['simulation_time_s'];np.testing.assert_array_equal(t,[r['time_s'] for r in packets]);mask=t>=1.;gt=z['root_position_world_m'];Rgt=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();delta=p-gt;pe=np.linalg.norm(delta,axis=1);ae=np.degrees(Rotation.from_matrix(R@Rgt.transpose(0,2,1)).magnitude());rms=float(np.sqrt(np.mean(pe[mask]**2)));peak=float(pe[mask].max());angle=float(np.percentile(ae[mask],90));gates=dict(complete=True,rms=rms<=.05,peak=peak<=.1,attitude=angle<=2.)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_TASK_H_APPLICABILITY_ONLY' if all(gates.values()) else 'REJECT_FROZEN_NATIVE_FLAT_REUSE',gates=gates,rows=len(t),evaluated=int(mask.sum()),translation_rms_m=rms,translation_peak_m=peak,attitude_p90_deg=angle,xy_rms_m=float(np.sqrt(np.mean(np.sum(delta[mask,:2]**2,axis=1)))),z_rms_m=float(np.sqrt(np.mean(delta[mask,2]**2))),old_strict_3d_reject_preserved=True,original_v2_navigation_reject_preserved=True,not_fresh_heldout=True,not_online_new_evidence=True,new_fit=False,prior_is_observation=False,formal35=False,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
