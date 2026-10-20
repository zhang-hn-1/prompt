"""Separate calibration, geometric overlap and same-pixel content changes.

No reconstructed-height filter is an independent exposure-alignment certificate.
These checks do not themselves prove all surfaces are synchronized to physics.
"""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from stereo_geometry import FOCAL,BASELINE
H=Path(__file__).resolve().parent
masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
y,x=np.indices((480,640));rays=np.stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)),axis=-1)
rows=[]
for row in json.loads((H/'transaction_development_manifest.json').read_text())['samples']:
    truth=np.load(row['truth_file']);left=truth['left_camera_usd_world'];right=truth['right_camera_usd_world']
    lr=left[:3,:3]@np.diag([1,-1,-1]);rr=right[:3,:3]@np.diag([1,-1,-1]);z=truth['left_depth_m']
    valid=np.isfinite(z)&(z>=.5)&(z<=3)&~masks['left']
    with np.errstate(invalid='ignore'):
        world=left[:3,3]+(rays*z[...,None])@lr.T
        pr=(world-right[:3,3])@rr
        xr=319.5+FOCAL*pr[...,0]/pr[...,2];yr=239.5+FOCAL*pr[...,1]/pr[...,2]
    disparity=FOCAL*BASELINE/z
    calib_pixel_errors=np.abs(xr-x)[valid]
    expected_vertical=np.abs(yr-(y-disparity))[valid]
    previous=truth['left_pretransaction_depth_m'];common=valid&np.isfinite(previous)&(previous>=.5)&(previous<=3)
    motion=np.abs(previous-z)*np.linalg.norm(rays,axis=-1)
    q=truth['root_quaternion_world_wxyz'];r=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
    extrinsics={}
    for eye,side,pose in [('left',1,left),('right',-1,right)]:
        expected=truth['root_position_world_m']+r@np.array([.2,side*.3,.05])
        extrinsics[eye]=float(np.linalg.norm(expected-pose[:3,3]))
    in_right=valid&(xr>=0)&(xr<640)&(yr>=0)&(yr<480)
    ix=np.rint(np.clip(np.nan_to_num(xr,nan=0,posinf=0,neginf=0),0,639)).astype(int)
    iy=np.rint(np.clip(np.nan_to_num(yr,nan=0,posinf=0,neginf=0),0,479)).astype(int)
    zright=truth['right_depth_m'][iy,ix]
    overlap=in_right&~masks['right'][iy,ix]&np.isfinite(zright)&(np.abs(zright-pr[...,2])<=.03+.01*z)
    body=overlap&(world[...,2]>=.05)&(world[...,2]<=1.5)&(np.linalg.norm(world-truth['root_position_world_m'],axis=-1)>=.55)
    rows.append(dict(sequence=row['sequence'],frame=Path(row['rgb_file']).name,
        calibration_horizontal_error_max_px=float(np.max(calib_pixel_errors)) if valid.any() else None,
        calibration_vertical_error_max_px=float(np.max(expected_vertical)) if valid.any() else None,
        optical_baseline_m=(rr.T@(left[:3,3]-right[:3,3])).tolist(),
        camera_position_error_m=extrinsics,
        common_pixels=int(common.sum()),pre_post_content_change_fraction=float((motion[common]>.001).mean()) if common.any() else None,
        pre_post_ray_displacement_p90_m=float(np.percentile(motion[common],90)) if common.any() else None,
        body_overlap_n=int(body.sum()),body_overlap_disparity_over_416_fraction=float((disparity[body]>416).mean()) if body.any() else None,
        body_overlap_true_depth_p10_m=float(np.percentile(z[body],10)) if body.any() else None))
result=dict(role='offline calibration and correspondence support audit; no model/online GT input',rows=rows,
    limitations='Extrinsic and epipolar consistency validate coordinate definitions, not all-scene exposure synchronization. Pre/post differences are content changes, not error reduction by themselves. Height subgroup eligibility uses reconstructed current-pose points.',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'transaction_development_manifest.json',H/'stereo_geometry.py']})
(H/'transaction_geometry_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print('GEOMETRY_ROWS',len(rows))
