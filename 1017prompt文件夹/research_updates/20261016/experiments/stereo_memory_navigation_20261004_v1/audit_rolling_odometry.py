"""Post-serialized truth-only rolling-odometry assessment."""
from pathlib import Path
import datetime,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'rolling_odometry_protocol.json').read_text());m=json.loads((H/'rolling_odometry_predictions/manifest.json').read_text());started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    assert m['all_two_saved_before_truth'] and all(r['ended_utc']<started for r in m['rows']);rows=[]
    for seq,predrow in zip(p['sequences'],m['rows']):
        assert sha(predrow['prediction'])==predrow['sha256'];assert sha(seq['offline_truth'])==p['evaluation_input_sha256'][seq['offline_truth']]
        pred=np.load(predrow['prediction']);truth=np.load(seq['offline_truth']);t=truth['simulation_time_s'];q=truth['root_quaternion_world_wxyz'];R=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();R0=R[0]
        assert np.array_equal(pred['time_s'],t)
        actual=(truth['root_position_world_m']-truth['root_position_world_m'][0])@R0
        pe=np.linalg.norm(pred['position_gauge']-actual,axis=1);relative=R0.T[None]@R;delta=np.transpose(relative,(0,2,1))@pred['rotation_gauge_body'];ae=np.degrees(np.arccos(np.clip((np.trace(delta,axis1=1,axis2=2)-1)/2,-1,1)))
        valid=t>=1.;rms=float(np.sqrt(np.mean(pe[valid]**2)));peak=float(pe[valid].max());p90=float(np.percentile(ae[valid],90))
        gates=dict(position_rms=rms<=.05,position_peak=peak<=.1,attitude_p90=p90<=2)
        row=dict(name=seq['name'],samples=len(t),evaluated_samples=int(valid.sum()),initial_gauge_only_alignment=True,translation_rms_m=rms,translation_peak_m=peak,attitude_p90_deg=p90,gates=gates,accepted=all(gates.values()))
        rows.append(row);np.savez_compressed(H/('rolling_odometry_'+seq['name']+'_measurement.npz'),time_s=t,position_error_m=pe,attitude_error_deg=ae,actual_position_gauge=actual,predicted_position_gauge=pred['position_gauge']);print('ODOMETRY',row,flush=True)
    result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,ideal_sensor_diagnostic_pass=all(r['accepted'] for r in rows),no_online_interface_verified=True,real_imu_or_bias_noise_verified=False,fresh_sequences=0,new_physics=0,new_fits=0,depth_or_navigation_admitted=False,whole_research_complete=False)
    (H/'rolling_odometry_audit.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
