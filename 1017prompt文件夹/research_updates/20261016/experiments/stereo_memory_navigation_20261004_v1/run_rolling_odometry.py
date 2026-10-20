"""Two historical ideal-sensor predictions; no dynamic trajectory truth input."""
from pathlib import Path
import datetime,json,hashlib
import numpy as np
from rolling_odometry import RollingOdometry
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
    p=json.loads((H/'rolling_odometry_protocol.json').read_text())
    for key in ['source_sha256','prediction_input_sha256']:
        for file,value in p[key].items():assert sha(file)==value,file
    rows=[];out=H/'rolling_odometry_predictions';out.mkdir(exist_ok=False)
    for row in p['sequences']:
        inp=np.load(row['sensor_packet']);assert set(inp.files)=={'time_s','gyro_body_rad_s','main_encoder_rad_s'}
        calibration=json.loads(Path(row['calibration']).read_text());model=RollingOdometry(radius=calibration['radius'],normal=calibration['normal_initial_body'],axis=calibration['joint1_axis_body'])
        positions=[];rotations=[];velocities=[];started=now()
        for t,g,qdot in zip(inp['time_s'],inp['gyro_body_rad_s'],inp['main_encoder_rad_s']):
            r=model.update(t,g,qdot);positions.append(r['position']);rotations.append(r['rotation']);velocities.append(r['velocity'])
        dest=out/(row['name']+'.npz');np.savez_compressed(dest,time_s=inp['time_s'],position_gauge=np.array(positions),rotation_gauge_body=np.array(rotations),velocity_gauge=np.array(velocities))
        rows.append(dict(name=row['name'],samples=len(positions),prediction=str(dest),sha256=sha(dest),started_utc=started,ended_utc=now()))
        print('ODOMETRY_PREDICTED',row['name'],len(positions),flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),all_two_saved_before_truth=True,rows=rows,protocol_sha256=sha(H/'rolling_odometry_protocol.json')),indent=2)+'\n')
if __name__=='__main__':main()
