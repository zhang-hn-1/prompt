"""Three fixed quadrature measurements; truth used only as offline diagnostic labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from rolling_odometry import exponential
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stats(e):return dict(rms=float(np.sqrt(np.mean(e**2))),p90=float(np.percentile(e,90)),peak=float(np.max(e)))
def main():
    protocol=H/'rolling_diagnostic_protocol.json';z=np.load(H/'rolling_online_run/physics.npz');sensor=json.loads((H/'rolling_online_run/ideal_sensor_inputs.json').read_text())
    t=z['simulation_time_s'];gyro=np.array([r['gyro_body_rad_s'] for r in sensor]);q=z['root_quaternion_world_wxyz'];R=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();relative=R[0].T[None]@R;normal=R[0].T@np.array([0.,0.,1.]);valid=t>=1
    out={};curves={}
    for method in ['previous-rate','current-end-rate','endpoint-average']:
        estimate=np.eye(3);rots=[estimate.copy()]
        for i in range(1,len(t)):
            omega=gyro[i-1] if method=='previous-rate' else gyro[i] if method=='current-end-rate' else (gyro[i-1]+gyro[i])/2
            estimate=estimate@exponential(omega*(t[i]-t[i-1]));rots.append(estimate.copy())
        delta=np.transpose(relative,(0,2,1))@np.array(rots);angle=np.degrees(Rotation.from_matrix(delta).magnitude());out[method]=stats(angle[valid]);curves[method+'_attitude_error_deg']=angle
    measurement=np.load(H/'rolling_online_measurement.npz');error=measurement['predicted_position_gauge']-measurement['actual_position_gauge'];worlderror=error@R[0].T
    out['translation_3d_m']=stats(np.linalg.norm(worlderror[valid],axis=1));out['translation_xy_m']=stats(np.linalg.norm(worlderror[valid,:2],axis=1));out['translation_vertical_m']=stats(abs(worlderror[valid,2]));out['actual_height_change_m']=dict(initial=float(z['root_position_world_m'][0,2]),minimum=float(z['root_position_world_m'][:,2].min()),final=float(z['root_position_world_m'][-1,2]))
    # This diagnostic explicitly supplies measured attitude: NOT an estimator or causal pose pass.
    shell=R@((gyro+z['joint_velocity_rad_s'][:,0,None]*np.array([0.,1.,0.]))[...,None]);shell=shell[...,0];nominal=.4000000059604645*np.cross(shell,np.array([0.,0.,1.]));residual=nominal-z['linear_velocity_world_m_s'];out['measured_attitude_no_slip_xy_velocity_residual_m_s']=stats(np.linalg.norm(residual[valid,:2],axis=1))
    position=np.zeros(3);positions=[position.copy()]
    for i in range(1,len(t)):
        position=position+nominal[i-1]*(t[i]-t[i-1]);positions.append(position.copy())
    positionerror=np.array(positions)-(z['root_position_world_m']-z['root_position_world_m'][0]);out['measured_attitude_diagnostic_xy_position_error_m']=stats(np.linalg.norm(positionerror[valid,:2],axis=1))
    api=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py')
    result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),protocol_sha256=sha(protocol),source_sha256={str(Path(__file__)):sha(__file__),str(api):sha(api)},input_sha256={str(H/'rolling_online_run'/p):sha(H/'rolling_online_run'/p) for p in ['physics.npz','ideal_sensor_inputs.json']},metrics=out,api_finding='Valid articulation handles: get_world_poses uses physics_view.get_root_transforms; angular velocities use physics_view.get_root_velocities. No USD/Fabric fallback in normal initialized run; cache blame is not established.',old_online_v1_rejection_preserved=True,candidate_admission=False,new_physics=0,new_fits=0,unresolved='Finite attitude increments include numerical/solver effects; these quadratures alone do not identify a gyro bias or prove a runtime timestamp bug.')
    (H/'rolling_temporal_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n');np.savez_compressed(H/'rolling_temporal_diagnosis.npz',time_s=t,world_position_error=worlderror,**curves)
    print(json.dumps(out,indent=2))
if __name__=='__main__':main()
