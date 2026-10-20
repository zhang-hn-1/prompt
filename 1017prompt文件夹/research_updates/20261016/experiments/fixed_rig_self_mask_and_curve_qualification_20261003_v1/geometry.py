"""Pose-guided same-eye multi-view matching. No labels, map, or future input."""
from collections import deque
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

FOCAL=16.45/18.*640
CX,CY=319.5,239.5
OFFSETS={'left':np.array([.1,.3,.1]),'right':np.array([.1,-.3,.1])}
OPTICAL_TO_BASE=np.array([[0.,0.,1.],[0.,-1.,0.],[1.,0.,0.]])
DEPTHS=1/np.linspace(1/8.,1/.55,96)


def camera_pose(xy,q,eye):
    xy=np.asarray(xy,dtype=float);q=np.asarray(q,dtype=float)
    if xy.shape!=(2,) or q.shape!=(4,) or not np.isfinite(xy).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('finite XY and nonzero WXYZ required')
    rotation=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
    return np.r_[xy,.4]+rotation@OFFSETS[eye],rotation@OPTICAL_TO_BASE


def project(points,pose):
    center,rotation=pose
    local=(np.asarray(points)-center)@rotation
    with np.errstate(divide='ignore',invalid='ignore'):
        uv=np.stack((CX+FOCAL*local[...,0]/local[...,2],CY+FOCAL*local[...,1]/local[...,2]),axis=-1)
    return uv,local[...,2]


def gray(rgb):
    rgb=np.asarray(rgb)
    if rgb.shape==(480,640,3):
        return cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(np.float32)
    if rgb.shape==(480,640):
        return rgb.astype(np.float32)
    raise ValueError('fixed rig image shape required')


def empty_result(corners=0,views=0):
    return dict(uv=np.empty((0,2)),depth_m=np.empty(0),points_world=np.empty((0,3)),
                ncc=np.empty(0),uniqueness=np.empty(0),corners=int(corners),support_views=int(views))


def recover_depth(current_rgb,current_pose,past,*,uv=None,valid_mask=None):
    image=gray(current_rgb)
    candidates=[]
    for old,pose in reversed(past):
        baseline=float(np.linalg.norm(pose[0]-current_pose[0]))
        if .08<=baseline<=1.2:
            candidates.append((gray(old),pose))
        if len(candidates)==3:break
    if len(candidates)<2:return empty_result(views=len(candidates))
    if uv is None:
        corners=cv2.goodFeaturesToTrack(np.uint8(image),maxCorners=300,qualityLevel=.01,minDistance=7,blockSize=7,
                                       mask=None if valid_mask is None else np.uint8(valid_mask)*255)
        if corners is None:return empty_result(views=len(candidates))
        uv=corners.reshape(-1,2)
    uv=np.asarray(uv,dtype=float).reshape(-1,2)
    inside=(uv[:,0]>=3)&(uv[:,0]<637)&(uv[:,1]>=3)&(uv[:,1]<477)
    uv=uv[inside]
    if valid_mask is not None:
        coords=np.rint(uv).astype(int)
        uv=uv[valid_mask[coords[:,1],coords[:,0]]]
    if not len(uv):return empty_result(views=len(candidates))
    yy,xx=np.mgrid[-2:3,-2:3];offset=np.column_stack((xx.ravel(),yy.ravel()))
    patches=uv[:,None,:]+offset
    values=cv2.remap(image,patches[:,:,0].astype('float32'),patches[:,:,1].astype('float32'),cv2.INTER_LINEAR)
    textured=values.var(axis=1)>=9.
    if valid_mask is not None:
        coords=np.rint(patches).astype(int)
        textured&=valid_mask[coords[:,:,1],coords[:,:,0]].all(axis=1)
    uv=uv[textured];patches=patches[textured];values=values[textured]
    if not len(uv):return empty_result(views=len(candidates))
    a=values-values.mean(axis=1,keepdims=True)
    a/=np.maximum(np.linalg.norm(a,axis=1,keepdims=True),1e-9)
    rays=np.concatenate(((patches-np.array([CX,CY]))/FOCAL,np.ones((*patches.shape[:2],1))),axis=2)
    center,rotation=current_pose
    world=center+(rays[:,None,:,:]*DEPTHS[None,:,None,None])@rotation.T
    scores=[];parallax=[]
    for old,pose in candidates:
        mapped,z=project(world,pose)
        valid=((z>0)&(mapped[...,0]>=0)&(mapped[...,0]<639)&(mapped[...,1]>=0)&(mapped[...,1]<479)).all(axis=2)
        if valid_mask is not None:
            coords=np.rint(mapped).astype(int)
            visible=valid_mask[np.clip(coords[...,1],0,479),np.clip(coords[...,0],0,639)]
            valid&=visible.all(axis=2)
        n,d,p=mapped.shape[:3]
        warped=cv2.remap(old,mapped[...,0].reshape(n,d*p).astype('float32'),
                         mapped[...,1].reshape(n,d*p).astype('float32'),cv2.INTER_LINEAR).reshape(n,d,p)
        b=warped-warped.mean(axis=2,keepdims=True)
        b/=np.maximum(np.linalg.norm(b,axis=2,keepdims=True),1e-9)
        ncc=np.sum(a[:,None,:]*b,axis=2)
        ncc[~valid]=-2.
        scores.append(ncc)
        parallax.append(np.linalg.norm(mapped[:,0,12,:]-mapped[:,-1,12,:],axis=1))
    scores=np.stack(scores)
    mean=scores.mean(axis=0)
    best=mean.argmax(axis=1);rows=np.arange(len(uv));depth=DEPTHS[best]
    peak=mean[rows,best]
    remote=np.abs(DEPTHS[None,:]/depth[:,None]-1)>=.2
    second=np.max(np.where(remote,mean,-2.),axis=1)
    margin=peak-second
    accepted=(peak>=.85)&(scores[:,rows,best].min(axis=0)>=.70)&(margin>=.05)
    accepted&=(np.max(np.stack(parallax),axis=0)>=4)
    rays_center=np.column_stack(((uv[:,0]-CX)/FOCAL,(uv[:,1]-CY)/FOCAL,np.ones(len(uv))))
    points=center+(rays_center*depth[:,None])@rotation.T
    return dict(uv=uv[accepted].copy(),depth_m=depth[accepted].copy(),points_world=points[accepted].copy(),
                ncc=peak[accepted].copy(),uniqueness=margin[accepted].copy(),
                corners=len(uv),support_views=len(candidates))


class MultiViewGeometry:
    def __init__(self,masks=None):
        self.history={'left':deque(),'right':deque()};self.last_saved=-np.inf;self.last_time=-np.inf
        self.masks=None if masks is None else {k:~np.asarray(v,dtype=bool).copy() for k,v in masks.items()}

    def update(self,left,right,xy,q,time_s):
        t=float(time_s)
        if not np.isfinite(t) or t<self.last_time-1e-8:raise ValueError('past-only monotonic clock required')
        self.last_time=t;results={};new={}
        for eye,rgb in (('left',left),('right',right)):
            pose=camera_pose(xy,q,eye)
            while self.history[eye] and self.history[eye][0][0]<t-2.:
                self.history[eye].popleft()
            old=[(image,old_pose) for _,image,old_pose in self.history[eye]]
            results[eye]=recover_depth(rgb,pose,old,valid_mask=None if self.masks is None else self.masks[eye])
            new[eye]=(t,gray(rgb).copy(),(pose[0].copy(),pose[1].copy()))
        if t-self.last_saved>=.2-1e-6:
            for eye in self.history:self.history[eye].append(new[eye])
            self.last_saved=t
        return results
