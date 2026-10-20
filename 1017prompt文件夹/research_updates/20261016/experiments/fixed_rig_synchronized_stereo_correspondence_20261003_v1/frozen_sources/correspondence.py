"""Validate actual stereo endpoints. No scene depth or future observation inputs."""
from pathlib import Path
import sys
import cv2
import numpy as np
HERE=Path(__file__).parent
sys.path.insert(0,str(HERE.parent/'stereo_vertical_baseline_20260929_v1'))
from vertical_depth import vertical_stereo_depth,FOCAL_LENGTH_PX,BASELINE_M


def validate_correspondence(left,right,source_disparity,other_disparity,masks,anchor='left'):
    if anchor not in ('left','right'):raise ValueError('valid eye required')
    left=np.asarray(left);right=np.asarray(right);d=np.asarray(source_disparity,dtype=np.float32);other=np.asarray(other_disparity,dtype=np.float32)
    if left.shape!=(480,640,3) or right.shape!=left.shape or d.shape!=(480,640) or other.shape!=d.shape:raise ValueError('fixed rig frames/disparities required')
    source_rgb,target_rgb=(left,right) if anchor=='left' else (right,left)
    target_eye='right' if anchor=='left' else 'left';direction=-1 if anchor=='left' else 1
    yy,xx=np.mgrid[:480,:640];map_x=xx.astype(np.float32);map_y=yy.astype(np.float32)+direction*np.nan_to_num(d,nan=0,posinf=0,neginf=0)
    source_free=cv2.erode((~np.asarray(masks[anchor],bool)).astype(np.uint8),np.ones((5,5),np.uint8))
    target_free=cv2.erode((~np.asarray(masks[target_eye],bool)).astype(np.uint8),np.ones((5,5),np.uint8))
    target_good=cv2.remap(target_free,map_x,map_y,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)>0
    d_other=cv2.remap(other,map_x,map_y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    minimum=FOCAL_LENGTH_PX*BASELINE_M/8.;maximum=FOCAL_LENGTH_PX*BASELINE_M/.6
    valid=np.isfinite(d)&(d>=minimum)&(d<=maximum)&(xx>=2)&(xx<638)&(yy>=2)&(yy<478)&(map_y>=2)&(map_y<478)&(source_free>0)&target_good
    valid&=np.isfinite(d_other)&(d_other>0)&(np.abs(d_other-d)<=1.)
    y,x=np.where(valid);output=np.zeros(d.shape,bool)
    if not len(x):return output
    oy,ox=np.mgrid[-2:3,-2:3];ox=ox.ravel();oy=oy.ravel()
    source=cv2.cvtColor(source_rgb,cv2.COLOR_RGB2GRAY).astype(np.float32);target=cv2.cvtColor(target_rgb,cv2.COLOR_RGB2GRAY).astype(np.float32)
    for start in range(0,len(x),8000):
        bx=x[start:start+8000];by=y[start:start+8000]
        u=bx[:,None]+ox;v=by[:,None]+oy;mv=map_y[by,bx][:,None]+oy
        a=source[v,u];b=cv2.remap(target,u.astype(np.float32),mv.astype(np.float32),cv2.INTER_LINEAR)
        a=a-a.mean(1,keepdims=True);b=b-b.mean(1,keepdims=True)
        std_a=np.sqrt(np.mean(a*a,axis=1));std_b=np.sqrt(np.mean(b*b,axis=1));denominator=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)
        correlation=np.sum(a*b,axis=1)/np.maximum(denominator,1e-9)
        accepted=(std_a>=5)&(std_b>=5)&(correlation>=.9)
        output[by[accepted],bx[accepted]]=True
    return output


def stereo_estimate(left,right,masks):
    depths={};raw_valid={};disparities={}
    for eye in ('left','right'):
        depths[eye],raw_valid[eye]=vertical_stereo_depth(left,right,anchor=eye,min_depth_m=.6,max_depth_m=8.)
        disparities[eye]=np.where(raw_valid[eye],FOCAL_LENGTH_PX*BASELINE_M/depths[eye],np.nan).astype(np.float32)
    return {eye:{'depth':depths[eye], 'raw_valid':raw_valid[eye],
                 'valid':validate_correspondence(left,right,disparities[eye],disparities['right' if eye=='left' else 'left'],masks,anchor=eye)} for eye in ('left','right')}
