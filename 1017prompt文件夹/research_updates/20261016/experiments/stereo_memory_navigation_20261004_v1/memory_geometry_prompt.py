"""Historical metric geometry with current appearance; completion is predicted."""
import numpy as np
from strict_bilinear_support import unmasked_linear_support


def memory_geometry_prompt(proof, pose, mask, now, frame, eye):
    if eye not in ('left', 'right'):
        raise ValueError('known eye required')
    t, old = float(now), float(proof['source_geometry_time_s'])
    clocks=[float(proof['current_appearance_time_s']),float(pose['time_s']),float(mask['time_s'])]
    sf=float(proof['source_frame'])
    if (not np.isfinite(t) or not np.isfinite(old) or old>t or old<0
            or any(not np.isfinite(c) or abs(c-t)>1e-6 for c in clocks)
            or int(mask['frame'])!=frame or not np.isfinite(sf)
            or sf<0 or int(sf)!=sf or sf>frame):
        raise ValueError('current/historical clock identity required before payload')
    result=dict(prompt_depth_m=np.zeros((480,640),np.float32),
                past_point_index=np.full((480,640),-1,np.int32),
                current_time_s=t,frame=int(frame),eye=eye,
                source_geometry_time_s=old,source_frame=int(sf),
                is_current_depth_observation=False,certified_free=False,
                predicted_output_is_observed=False,nonzero_prompts=0,
                status='NOT_EXERCISED_EMPTY_PROMPT')
    if t-old>10.:
        result['status']='EXPIRED_HISTORICAL_GEOMETRY'
        return result
    world=np.asarray(proof['past_world'],float)
    uv=np.asarray(proof[eye+'_uv'],float);z=np.asarray(proof[eye+'_z'],float)
    score=np.asarray(proof['patch_NCC'],float);std=np.asarray(proof['patch_std'],float)
    n=len(world)
    if (world.shape!=(n,3) or not np.isfinite(world).all() or uv.shape!=(n,2)
            or z.shape!=(n,) or score.shape!=(n,2) or std.shape!=(n,2,2)):
        raise ValueError('finite geometry/appearance shape required')
    T=np.asarray(pose[eye+'_camera_usd_world'],float)
    if (T.shape!=(4,4) or not np.isfinite(T).all()
            or not np.allclose(T[3],[0,0,0,1],atol=1e-8)
            or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)):
        raise ValueError('current rigid camera required')
    local=(world-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]))
    with np.errstate(divide='ignore',invalid='ignore'):
        recomputed=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
    if not np.allclose(z,local[:,2],atol=1e-6,rtol=0,equal_nan=True) or not np.allclose(uv,recomputed,atol=1e-6,rtol=0,equal_nan=True):
        raise ValueError('current projection does not match historical world geometry')
    ei=0 if eye=='left' else 1
    accepted=(np.isfinite(uv).all(1)&np.isfinite(z)&(z>=.5)&(z<=3)
              &np.isfinite(score[:,ei])&(score[:,ei]>=.8)&(std[:,ei]>=5).all(1))
    off=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)])
    taps=uv[:,None,:]+off
    accepted &= unmasked_linear_support(np.asarray(mask[eye],bool),taps[:,:,0].astype(np.float32),taps[:,:,1].astype(np.float32)).all(1)
    # Preserve original point indices. Equal-depth ties select the earlier index.
    indices=np.flatnonzero(accepted)
    for j in indices[np.argsort(z[indices],kind='stable')]:
        x,y=np.rint(uv[j]).astype(int)
        if result['past_point_index'][y,x]<0:
            result['prompt_depth_m'][y,x]=z[j]
            result['past_point_index'][y,x]=j
    result['nonzero_prompts']=int((result['prompt_depth_m']>0).sum())
    if result['nonzero_prompts']:result['status']='HISTORICAL_GEOMETRY_WITH_CURRENT_APPEARANCE'
    return result


def world_up_pair(rgb, prompt_depth):
    rgb=np.asarray(rgb);depth=np.asarray(prompt_depth)
    if rgb.shape!=(480,640,3) or depth.shape!=(480,640):
        raise ValueError('native calibrated image/prompt pair required')
    return np.rot90(rgb,1).copy(),np.rot90(depth,1).copy()


def restore_native_prediction(world_up_depth):
    depth=np.asarray(world_up_depth)
    if depth.shape!=(640,480):raise ValueError('native world-up completion shape required')
    return np.rot90(depth,-1).copy()
