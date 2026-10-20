"""GT postseal necessary visibility check only; not a ground-contact detector."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1'
protocol=json.loads((H/'monocular_ground_contact_visibility_protocol_v1.json').read_text());scene=H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json';shape=next(x for x in json.loads(scene.read_text()) if x['path']=='/E1Detour/Divider/divider_06');c=np.array(shape['position']);radius=shape['radius'];half=shape['height']/2
y,x=np.indices((480,640));rays=np.stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones_like(x)),axis=-1);rows=[];inputs={str(scene):hashlib.sha256(scene.read_bytes()).hexdigest()}
for f in protocol['frames']:
    files=[D/'frames'/f'frame_{f:03d}.npz',D/'known_poses'/f'frame_{f:03d}.npz']
    for p in files:inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    gt=np.load(files[0]);poses=np.load(files[1]);eyes=[]
    for eye in ['left','right']:
        depth=gt[eye+'_depth_m'];pose=poses[eye+'_camera_usd_world'];R=pose[:3,:3]@np.diag([1.,-1.,-1.])
        with np.errstate(invalid='ignore'):world=pose[:3,3]+(rays*depth[...,None])@R.T
        target=np.isfinite(world).all(-1)&np.isfinite(depth)&(depth>0)&(np.linalg.norm(world[...,:2]-c[:2],axis=-1)<=radius+.001)&(np.abs(world[...,2]-c[2])<=half+.001)
        near=target&(world[...,2]>=-.001)&(world[...,2]<=.05);heights=world[...,2][target];v,u=np.nonzero(near)
        eyes.append(dict(eye=eye,visible_target_pixels=int(target.sum()),minimum_visible_target_height_m=float(heights.min()) if len(heights) else None,ground_contact_band_pixels=int(near.sum()),contact_band_pixel_bbox_xy=[int(u.min()),int(v.min()),int(u.max()),int(v.max())] if len(u) else None))
    rows.append(dict(frame=f,eyes=eyes))
result=dict(protocol=protocol,rows=rows,raw_before_CAD=True,CAD_removed_pixels_not_computed=True,not_RGB_detector=True,no_current_or_future_action=True,no_full_3D_claim=True,whole_research_complete=False,input_sha256=inputs,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(H/'monocular_ground_contact_visibility_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows,indent=2))
