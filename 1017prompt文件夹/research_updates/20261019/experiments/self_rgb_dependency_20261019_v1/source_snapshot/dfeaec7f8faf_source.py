"""Fixed traditional matcher, current rig, four signed query directions.
Only RGB, current joints, nominal K/extrinsics and causal estimated body pose.
Depth validity is a matching heuristic, not a calibrated error probability.
Height is an explicitly weak risk proxy, NOT predicted RGB semantics.
"""
from paths import *
import numpy as np, cv2, time
from four_camera import NAMES,nominal_transforms
from dynamic_self_mask import RobotSelfMask
from self_mask import mask_triangles
AX=np.diag([1.,-1.,-1.]); F=294.77087816197377; B=.65012
LOCAL=nominal_transforms(.18197,.32506,.11503)
PAIRS=(('front_left','front_right'),('rear_right','rear_left'))
CONFIG=dict(numDisparities=448,blockSize=5,P1=8*3*25,P2=32*3*25,disp12MaxDiff=1,uniquenessRatio=10,speckleWindowSize=100,speckleRange=2,preFilterCap=31,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
YY,XX=np.indices((480,640));RAYS=np.stack([(XX-319.5)/F,(YY-239.5)/F,np.ones_like(XX)],-1)

def depths(disparity,other,own_mask,other_mask,sign):
    y,x=np.indices(disparity.shape); xp=x-disparity
    ix=np.rint(np.clip(np.nan_to_num(xp),0,x.shape[1]-1)).astype(int)
    partner=other[y,ix]
    valid=np.isfinite(disparity)&(disparity*sign>0)&(xp>=0)&(xp<=x.shape[1]-1)&~own_mask
    valid &= ~other_mask[y,ix]&np.isfinite(partner)&(partner*sign<0)&(abs(disparity+partner)<=1.5)
    z=np.full(disparity.shape,np.nan,np.float32);z[valid]=F*B/abs(disparity[valid])
    return np.ascontiguousarray(np.rot90(z,-1)),np.ascontiguousarray(np.rot90(valid,-1)),np.ascontiguousarray(np.rot90(disparity,-1))

class FourSGBM:
    def __init__(self):
        cv2.setNumThreads(1)
        self.left=cv2.StereoSGBM_create(minDisparity=0,**CONFIG)
        self.right=cv2.StereoSGBM_create(minDisparity=-448,**CONFIG)
        self.cad=RobotSelfMask()
    def masks(self,joints):
        bodies=self.cad.body_transforms(joints)
        tri=np.concatenate([v@bodies[n][:3,:3].T+bodies[n][:3,3] for n,v in self.cad.triangles.items()])
        return {n:mask_triangles((tri-T[:3,3])@(T[:3,:3]@AX),640,480,F,319.5,239.5) for n,T in zip(NAMES,LOCAL)}
    def predict(self,rgb,joints):
        begin=time.monotonic(); masks=self.masks(joints);out={}
        for q,p in PAIRS:
            a,b=[np.ascontiguousarray(np.rot90(rgb[n])) for n in (q,p)]
            lm,rm=[np.ascontiguousarray(np.rot90(masks[n])) for n in (q,p)]
            ap,bp=[cv2.copyMakeBorder(v,0,0,448,448,cv2.BORDER_CONSTANT,value=0) for v in (a,b)]
            dl=self.left.compute(ap,bp).astype(np.float32)[:,448:-448]/16.
            dr=self.right.compute(bp,ap).astype(np.float32)[:,448:-448]/16.
            for eye,partner,dd,od,mm,om,sign in [(q,p,dl,dr,lm,rm,1),(p,q,dr,dl,rm,lm,-1)]:
                z,valid,d=depths(dd,od,mm,om,sign)
                out[eye]=dict(depth=z,valid=valid,disparity=d,CAD=masks[eye],partner=partner,sign=sign)
        return out,time.monotonic()-begin

def points(pred,body,frame,t):
    records=[];arrays={}
    for n,Tlocal in zip(NAMES,LOCAL):
        item=pred[n];z=item['depth'];d=item['disparity']
        ok=item['valid']&np.isfinite(z)&(z>=.5)&(z<=3)&(XX%2==0)&(YY%2==0)
        q=np.flatnonzero(ok);T=body@Tlocal
        world=(RAYS.reshape(-1,3)[q]*z.reshape(-1)[q,None])@(T[:3,:3]@AX).T+T[:3,3]
        keep=(np.linalg.norm(world-body[:3,3],axis=1)>=.55)&(world[:,2]>.05)&(world[:,2]<=1.5)
        world=world[keep];q=q[keep]
        pv=np.rint(YY.reshape(-1)[q]-d.reshape(-1)[q]).astype(int);assert np.all((pv>=0)&(pv<480))
        partner=pv*640+XX.reshape(-1)[q]
        rec=dict(frame=int(frame),time_s=float(t),world=world,query_camera=n,partner_camera=item['partner'],query_pixel_id=q,partner_pixel_id=partner,pose_source='causal_gyro_encoder_initial_gauge_privileged',depth_source='fixed_SGBM_current_K_baseline',layer='weak_height_risk_proxy_not_RGB_semantics')
        records.append(rec)
        arrays[n+'_world']=world;arrays[n+'_query_pixel']=q;arrays[n+'_partner_pixel']=partner
    return records,arrays
