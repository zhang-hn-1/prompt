"""Frozen current RGB refinement adapter; no truth, mapper or policy input."""
import numpy as np
from local_binocular_refinement import refine
from stereo_geometry import FOCAL

def refined_surface(surface,left,right,masks,right_positive,disparity_native,T,now,hashes):
    t=float(surface['acquisition_time_s'])
    if not np.isfinite(t) or not np.isfinite(now) or t>now:raise ValueError('acquisition before image payload')
    q=np.asarray(surface['query_pixel_id']);y,x=np.divmod(q,640);d=disparity_native[y,x]
    match=refine(left,right,masks['left'],masks['right'],right_positive,np.column_stack([x,y]),d)
    depth=FOCAL*.6/match['disparity_px'];keep=match['keep']&np.isfinite(depth)&(depth>=.5)&(depth<=3)
    rays=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))]);world=T[:3,3]+(rays[keep]*depth[keep,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
    partner=np.rint(y[keep]-match['disparity_px'][keep]).astype(int)*640+x[keep]
    return dict(frame=int(surface['frame']),time_s=t,world=world,query_pixel_id=q[keep],partner_pixel_id=partner,input_sha256=hashes)
