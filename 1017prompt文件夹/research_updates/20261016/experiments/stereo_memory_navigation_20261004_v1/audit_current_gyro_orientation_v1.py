from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
from rolling_odometry import exponential
H=Path(__file__).resolve().parent;D=H/'execution_domain_four_navigation_R1_run_v2';O=H/'current_gyro_orientation_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();sensorfile=D/'ideal_rolling_sensors.json';gaugefile=D/'initial_gauge.json';sensors=json.loads(sensorfile.read_text());gauge=json.loads(gaugefile.read_text())
def predict(rows,mode):
 R=np.asarray(gauge['rotation'],float).copy();out=[];prev=None
 for row in rows:
  assert set(row)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'}
  if prev is not None:
   dt=float(row['time_s']-prev['time_s']);assert dt>0
   omega=np.asarray(prev['gyro_body_rad_s']) if mode=='original_previous' else .5*(np.asarray(prev['gyro_body_rad_s'])+np.asarray(row['gyro_body_rad_s']))
   R=R@exponential(omega*dt)
  out.append(R.copy());prev=row
 return np.asarray(out)
a=predict(sensors,'original_previous');b=predict(sensors,'unique_trapezoid');np.testing.assert_array_equal(b[:501],predict(sensors[:501],'unique_trapezoid'));poison=json.loads(json.dumps(sensors));
for row in poison[501:]:row['gyro_body_rad_s']=[999,888,777]
np.testing.assert_array_equal(b[:501],predict(poison,'unique_trapezoid')[:501]);np.savez_compressed(O/'predictions_pretruth.npz',time_s=np.array([r['time_s'] for r in sensors]),original_rotation=a,trapezoid_rotation=b)
(O/'prediction_seal_pretruth.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(f):sha(f) for f in [sensorfile,gaugefile,H/'rolling_odometry.py',H.parent.parent/'docs/superpowers/plans/2026-10-08-current-gyro-orientation-consistency.md']},code_sha256=sha(__file__),prediction_sha256=sha(O/'predictions_pretruth.npz'),prefix501_and_future_replacement_exact=True,no_truth_model_input=True),indent=2)+'\n')
# Only now inspect physical labels / compare the frozen original bridge output.
z=np.load(D/'physics.npz');Rgt=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();t=z['simulation_time_s'];mask=t>=1.;results=[]
for mode,rotation in [('original_previous',a),('unique_trapezoid',b)]:
 e=np.degrees(Rotation.from_matrix(rotation@Rgt.transpose(0,2,1)).magnitude());results.append(dict(mode=mode,angle_p90_deg=float(np.percentile(e[mask],90)),angle_peak_deg=float(e[mask].max()),gate=float(np.percentile(e[mask],90))<=2,interval_p90_deg=[float(np.percentile(e[(t>=lo)&(t<hi if hi!=60 else t<=hi)],90)) for lo,hi in [(1,20),(20,40),(40,60)]]))
pred=json.loads((D/'rolling_predictions.json').read_text());np.testing.assert_allclose(a,np.asarray(gauge['rotation'])@np.asarray([r['rotation'] for r in pred]),atol=1e-14,rtol=0)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),results=results,decision='PASS_HISTORICAL_ORIENTATION_QUADRATURE_ONLY' if results[1]['gate'] else 'REJECT_UNIQUE_TRAPEZOID_ORIENTATION_HYPOTHESIS',original_angle_gate_reject_preserved=True,not_full_position_or_source_or_navigation_validation=True,not_fresh_heldout=True,no_fit_or_newphysics=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
