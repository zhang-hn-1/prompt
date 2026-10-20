"""Known robot geometry mask. No scene observations or depth inputs."""
import cv2
import numpy as np


def _clip(poly, signed_distance):
    output=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da,db=signed_distance(a),signed_distance(b)
        if da>=0: output.append(a)
        if (da>=0)!=(db>=0): output.append(a+(b-a)*da/(da-db))
    return np.asarray(output)


def mask_triangles(triangles, width, height, focal, cx, cy):
    triangles=np.asarray(triangles,dtype=float).reshape(-1,3,3)
    mask=np.zeros((height,width),np.uint8)
    positive=(triangles[:,:,2]>=.01).all(axis=1)
    front=triangles[positive]
    uv=front[:,:,:2]/front[:,:,2,None]*focal+np.array([cx,cy])
    keep=(uv[:,:,0].max(1)>=0)&(uv[:,:,0].min(1)<width)&(uv[:,:,1].max(1)>=0)&(uv[:,:,1].min(1)<height)
    polygons=list(uv[keep])
    crossing=triangles[~positive&(triangles[:,:,2].max(1)>=.01)]
    for tri in crossing:
        clipped=_clip(tri,lambda p:p[2]-.01)
        if len(clipped)>=3:polygons.append(clipped[:,:2]/clipped[:,2,None]*focal+np.array([cx,cy]))
    for poly in polygons:
        for fn in (lambda p:p[0]+1,lambda p:width-p[0],lambda p:p[1]+1,lambda p:height-p[1]):
            if not len(poly):break
            if any(fn(p)<0 for p in poly):poly=_clip(poly,fn)
        if len(poly)>=3:cv2.fillConvexPoly(mask,np.rint(poly).astype(np.int32),1)
    return mask.astype(bool)
