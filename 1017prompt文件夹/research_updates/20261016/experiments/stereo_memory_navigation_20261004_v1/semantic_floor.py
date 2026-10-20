"""RGB semantic evidence and separate known-plane projection; no stereo depths."""
import numpy as np
import cv2
from multiview_rgb import optical_rotation

def validate_rgb_packet(packet):
    if set(packet)!={'rgb','time_s','frame','eye'}:raise ValueError('RGB-only input required')
    rgb=np.asarray(packet['rgb'])
    if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8 or not np.isfinite(float(packet['time_s'])):raise ValueError('uint8 RGB and finite timestamp required')

def upright_rgb(rgb):return np.rot90(np.asarray(rgb),1).copy()

def native_floor_mask(probability,class_is_floor,threshold):
    p=np.asarray(probability,float);classes=np.asarray(class_is_floor,bool)
    if p.ndim!=2 or p.shape!=classes.shape:raise ValueError('matching semantic outputs required')
    return np.rot90(classes&np.isfinite(p)&(p>=threshold),-1).copy()

def project_floor(semantic_mask,pose,selfmask,*,focal,cx,cy,patch_radius,depth_min,depth_max):
    semantic=np.asarray(semantic_mask,bool);mask=np.asarray(selfmask,bool)
    if semantic.shape!=mask.shape or semantic.ndim!=2:raise ValueError('matching native masks required')
    h,w=semantic.shape;yy,xx=np.mgrid[:h,:w];rays=np.stack([(xx-cx)/focal,(yy-cy)/focal,np.ones((h,w))],axis=-1);pose=np.asarray(pose,float)
    worldray=rays@optical_rotation(pose).T
    depth=np.divide(-pose[2,3],worldray[...,2],out=np.full((h,w),np.nan),where=abs(worldray[...,2])>1e-12)
    geom=np.isfinite(depth)&(depth>=depth_min)&(depth<=depth_max)
    k=2*patch_radius+1;unmasked=cv2.boxFilter((~mask).astype(np.float32),-1,(k,k),normalize=True,borderType=cv2.BORDER_CONSTANT)>=1-1e-6
    confidence=semantic&geom&unmasked
    return dict(confidence=confidence,depth_m=np.where(confidence,depth,np.nan).astype(np.float32),plane_axis_depth_m=np.where(geom,depth,np.nan).astype(np.float32),semantic_floor_native=semantic)
