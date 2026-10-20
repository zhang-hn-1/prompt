"""Source-only numerical conditioning diagnostic, not a pose optimizer."""
from pathlib import Path
import json, datetime, hashlib
import numpy as np
from rolling_odometry import exponential

H=Path(__file__).resolve().parent
D=H/'RGB_memory_frozen_pose_stress_v1'
out=D/'projection_observability_v1'
assert not out.exists()
out.mkdir()
sources={}
def load(p):
    sources[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
    return np.load(p)
def project(world, p, R):
    body_optical=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
    values=[]
    for y in [.3,-.3]:
        camera_R=R@body_optical
        camera_p=p+R@np.array([.2,y,.05])
        v=(world-camera_p)@camera_R
        assert np.all(v[:,2]>0)
        values.append((v[:,:2]/v[:,2,None]*584.8888888888889+[319.5,239.5]).ravel())
    return np.concatenate(values)
def svd_info(J):
    _,s,Vt=np.linalg.svd(J,full_matrices=False)
    rank=int(np.linalg.matrix_rank(J))
    sd=np.sqrt(np.diag(np.linalg.pinv(J.T@J)))
    return dict(rank=rank,singular_values=s.tolist(),condition=float(s[0]/s[-1]),
                optimistic_independent_1pixel_std=sd.tolist(),weakest_mode=Vt[-1].tolist())
rows=[]
for frame in [185,190,195]:
    pose=load(D/'estimated_poses'/f'frame_{frame:03d}.npz')
    proof=load(D/'memory_predictions'/f'frame_{frame:03d}.npz')
    world=proof['past_world'][proof['appearance_corroborated']]
    p=pose['root_position_world_m'];R=pose['root_rotation_world']
    step=1e-5;cols=[]
    for axis in range(6):
        v=np.eye(3)[axis%3]*step
        plus=project(world,p+v,R) if axis<3 else project(world,p,R@exponential(v))
        minus=project(world,p-v,R) if axis<3 else project(world,p,R@exponential(-v))
        cols.append((plus-minus)/(2*step))
    J=np.array(cols).T
    assert np.isfinite(J).all()
    normalized=J/np.linalg.norm(J,axis=0)
    cross=normalized[:,:3].T@normalized[:,3:]
    np.savez_compressed(out/f'frame_{frame:03d}_jacobian.npz',J=J,world=world)
    rows.append(dict(frame=frame,support=len(world),world_XY_span_m=np.ptp(world[:,:2],axis=0).tolist(),
                     rotation_only=svd_info(J[:,3:]),full6DOF=svd_info(J),
                     translation_rotation_cosine=cross.tolist(),maximum_absolute_coupling=float(np.abs(cross).max())))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,
            axes='world translation metres then body right-multiplied rotation radians',
            geometry_initial_gauge_idealgyro_privileged=True,current_truth_pose_read=False,
            renderer_labels_read=False,pose_correction_or_control_applied=False,
            independent_1pixel_noise_only_an_optimistic_assumption=True,
            old_reject_and_gates_unchanged=True,development_not_fresh=True,
            input_sha256=sources,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(out/'source_only_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='input_sha256'},indent=2))
