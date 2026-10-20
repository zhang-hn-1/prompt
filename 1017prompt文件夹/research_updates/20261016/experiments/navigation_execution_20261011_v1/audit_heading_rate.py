"""One read-only coordinate audit of a sealed episode; never replays a policy."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from heading_rate import projected_heading_rate

HERE=Path(__file__).resolve().parent
RUN=HERE.parent/'stereo_memory_navigation_20261004_v1/dense_current_live_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    out=HERE/'heading_rate_coordinate_audit_v1.json'
    if out.exists():
        raise FileExistsError('single archived diagnostic already closed')
    manifest_path=RUN/'artifact_manifest_post_close.json'
    seal=json.loads(manifest_path.read_text())['artifact_sha256']
    inputs={str(manifest_path):sha(manifest_path)}
    def verify(p):
        digest=sha(p)
        if seal.get(str(p))!=digest:
            raise ValueError('archived input identity mismatch: '+str(p))
        inputs[str(p)]=digest
    for name in ['states.json','physics.npz','policy_records.json']:
        verify(RUN/name)
    states=json.loads((RUN/'states.json').read_text())
    policy_rows=json.loads((RUN/'policy_records.json').read_text())
    physics=dict(np.load(RUN/'physics.npz'))
    rows=[]
    for i,state in enumerate(states):
        file=RUN/f'history/frame_{i:03d}.npz';verify(file)
        with np.load(file) as history:
            current=history['proprio_history'][-1]
            quaternion=history['quaternion_world_wxyz_history'][-1]
        np.testing.assert_allclose(quaternion,state['quaternion'],atol=1e-6,rtol=0)
        np.testing.assert_allclose(quaternion,physics['root_quaternion_world_wxyz'][i*5],atol=1e-6,rtol=0)
        np.testing.assert_allclose(current[3:6],physics['angular_velocity_world_rad_s'][i*5],atol=1e-6,rtol=0)
        if abs(state['time_s']-physics['simulation_time_s'][i*5])>1e-6:
            raise ValueError('capture/physics clock mismatch')
        geometry=projected_heading_rate(quaternion,current[3:6])
        if not geometry['supported']:
            raise ValueError('unexpected singularity in declared diagnostic coverage')
        tracking=i<len(policy_rows) and policy_rows[i]['planner_act_called'] and policy_rows[i]['tracker_state_after_act']['braking_state']=='tracking'
        rows.append(dict(frame=i,time_s=state['time_s'],ordinary_tracking=tracking,**geometry,
                         absolute_rate_difference_rad_s=abs(geometry['yaw_rate_rad_s']-geometry['world_z_rate_rad_s'])))
    rotation=Rotation.from_quat(physics['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix()
    axis=rotation[:,:,0];derivative=np.cross(physics['angular_velocity_world_rad_s'],axis)
    projected=(axis[:,0]*derivative[:,1]-axis[:,1]*derivative[:,0])/(axis[:,0]**2+axis[:,1]**2)
    yaw=np.unwrap(np.arctan2(axis[:,1],axis[:,0]));secant=np.diff(yaw)/np.diff(physics['simulation_time_s'])
    new_residual=np.abs(secant-.5*(projected[:-1]+projected[1:]))
    old=physics['angular_velocity_world_rad_s'][:,2]
    old_residual=np.abs(secant-.5*(old[:-1]+old[1:]))
    percentiles=lambda a:dict(zip(['median','p90','maximum'],np.percentile(a,[50,90,100]).tolist()))
    result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        captures=len(rows),physics_samples=len(axis),past_current_only_rate_inputs=True,
        original_manifest_all_used_inputs_verified=True,
        capture_to_physics_current_quaternion_and_angular_velocity_verified=True,
        rate_difference_all_rad_s=percentiles([r['absolute_rate_difference_rad_s'] for r in rows]),
        rate_difference_tracking_rad_s=percentiles([r['absolute_rate_difference_rad_s'] for r in rows if r['ordinary_tracking']]),
        physics_yaw_secant_residual_projected_rad_s=percentiles(new_residual),
        physics_yaw_secant_residual_worldZ_rad_s=percentiles(old_residual),
        interpretation='Coordinate-consistency evidence, not a proof of navigation benefit or exact transient dynamics. Physics secants are retrospective diagnostic only.',
        new_physics=0,new_solver_queries=0,new_inferences=0,new_teacher_queries=0,
        inputs_sha256=inputs,source_sha256={str(p):sha(p) for p in [Path(__file__).resolve(),HERE/'heading_rate.py']},rows=rows,
        whole_mission_complete=False)
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['rows','inputs_sha256','source_sha256']},indent=2))

if __name__=='__main__':
    main()
