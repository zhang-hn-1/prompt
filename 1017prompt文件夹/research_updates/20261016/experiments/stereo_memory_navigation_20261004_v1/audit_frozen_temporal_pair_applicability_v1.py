"""Necessary rectification and postsealed visibility diagnostics; no actor/NN."""
from pathlib import Path
import hashlib,json
import numpy as np
H=Path(__file__).resolve().parent
O=H/'dense_temporal_left_development_v1'
D=H/'minimum_prefix_transition_navigation_run_v1'
identity=json.loads((O/'pair_identity.json').read_text())
shapes=json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text())
shape=next(s for s in shapes if s['path']=='/E1Detour/Divider/divider_06')
c=np.array(shape['position']);radius=shape['radius'];half=shape['height']/2
y,x=np.indices((480,640));rays=np.stack(((x-319.5)/584.8889,(y-239.5)/584.8889,np.ones_like(x)),axis=-1)
def visible(frame):
    # Evaluation only, never input to a model or action. No producer is rerun.
    truth=np.load(D/'frames'/f'frame_{frame:03d}.npz')
    pose=np.load(D/'known_poses'/f'frame_{frame:03d}.npz')['left_camera_usd_world']
    depth=truth['left_depth_m'];R=pose[:3,:3]@np.diag([1,-1,-1])
    with np.errstate(invalid='ignore'):
        world=pose[:3,3]+(rays*depth[...,None])@R.T
    valid=np.isfinite(world).all(-1)&np.isfinite(depth)&(depth>0)
    target=valid&(np.linalg.norm(world[...,:2]-c[:2],axis=-1)<=radius+.001)&(np.abs(world[...,2]-c[2])<=half+.001)
    return int(target.sum())
rows=[]
for pair in identity['pairs']:
    p=O/pair['pair_file'];assert hashlib.sha256(p.read_bytes()).hexdigest()==identity['pair_sha256'][str(p)]
    z=np.load(p);q=~z['query_mask'];b=~z['past_mask'];possible=np.zeros_like(q)
    for d in range(1,min(416,q.shape[1]-1)+1):possible[:,d:] |= q[:,d:]&b[:,:-d]
    bounds=[]
    for name in ['query','past']:
        bounds.append(int(((z[name+'_native_x']>=0)&(z[name+'_native_x']<=639)&(z[name+'_native_y']>=0)&(z[name+'_native_y']<=479)).sum()))
    # Complete empty native support proves impossibility even for noninteger disparities.
    rows.append(dict(current=pair['current_frame'],past=pair['past_frame'],query_inbounds_pixels=bounds[0],past_inbounds_pixels=bounds[1],positive_disparity_possible_query_pixels=int(possible.sum()),current_target_GT_pixels=visible(pair['current_frame']),past_target_GT_pixels=visible(pair['past_frame']),decision='REJECT_FROZEN_RECTIFICATION_EMPTY_NATIVE_SUPPORT' if not min(bounds) else 'RECTIFICATION_NECESSARY_SUPPORT_ONLY'))
result=dict(rows=rows,pair_choices_unchanged=True,alpha_unchanged=1,no_NN_launch=True,no_actor_or_control_change=True,GT_postsealed_evaluation_only=True,scope='These two frozen planar rectification pairs only; not a rejection of general temporal depth or raw-image correspondence.',old_same_pixel_overlap_not_valid_stereo_gate=True,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O/'applicability_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
