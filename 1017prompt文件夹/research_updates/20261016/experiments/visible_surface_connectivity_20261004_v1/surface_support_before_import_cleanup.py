"""Native current-depth support; never fills between masked/invalid pixels."""
import numpy as np
from scipy.ndimage import minimum_filter

def quad_support(points,valid,root,stride=4):
    points=np.asarray(points);valid=np.asarray(valid,dtype=bool)
    floor=valid&np.isfinite(points).all(-1)&(points[...,2]>=-.05)&(points[...,2]<=.05)&(np.linalg.norm(points-root,axis=-1)>=.55)
    # A top-left native pixel requires its entire closed (stride+1)^2 quad.
    patches=np.lib.stride_tricks.sliding_window_view(floor,(stride+1,stride+1))
    all_valid=patches.all((-1,-2))
    rows=(points.shape[0]-1)//stride;cols=(points.shape[1]-1)//stride
    return all_valid[np.arange(rows)[:,None]*stride,np.arange(cols)[None,:]*stride]

def native_ground_mesh(depth,pose,self_mask,root,focal=16.45/18*640,stride=4):
    depth=np.asarray(depth);y,x=np.indices(depth.shape)
    rays=np.stack(((x-319.5)/focal,(y-239.5)/focal,np.ones_like(x)),axis=-1)
    rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
    valid=np.isfinite(depth)&(depth>=.45)&(depth<=8)&~self_mask
    with np.errstate(invalid='ignore'):
        points=pose[:3,3]+(rays*depth[...,None])@rotation.T
    supported=quad_support(points,valid,root,stride)
    mesh=points[::stride,::stride].copy()
    vertex_valid=valid[::stride,::stride].copy()
    # Conservatively remove every vertex touched by an unsupported quad.
    # Consequently every accepted sampled triangle has a fully supported native quad.
    for di,dj in ((0,0),(0,1),(1,0),(1,1)):
        vertex_valid[di:di+supported.shape[0],dj:dj+supported.shape[1]]&=supported
    mesh[~vertex_valid]=np.nan
    return mesh
