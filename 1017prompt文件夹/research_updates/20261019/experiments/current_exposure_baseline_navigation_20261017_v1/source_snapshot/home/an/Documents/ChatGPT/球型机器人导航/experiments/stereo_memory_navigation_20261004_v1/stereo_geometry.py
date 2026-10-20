"""Frozen native vertical stereo rig; inference never receives depth labels."""
import numpy as np
FOCAL=584.8888888888889
BASELINE=.6

def rectify_pair(left,right):
    if np.shape(left)!=np.shape(right):raise ValueError('stereo shapes differ')
    return np.ascontiguousarray(np.rot90(left)),np.ascontiguousarray(np.rot90(right))

def restore_native(rectified):
    return np.ascontiguousarray(np.rot90(rectified,-1))

def disparity_depth(disparity,right_disparity,left_mask,right_mask,lr_tolerance_px=1.5):
    d=np.asarray(disparity,dtype=float);dr=np.asarray(right_disparity,dtype=float)
    if not (d.shape==dr.shape==np.shape(left_mask)==np.shape(right_mask)):
        raise ValueError('disparity/mask dimensions differ')
    y,x=np.indices(d.shape)
    xr=x-np.where(np.isfinite(d),d,0.)
    ix=np.rint(np.clip(xr,0,d.shape[1]-1)).astype(int)
    partner=dr[y,ix]
    valid=np.isfinite(d)&(d>0)&(xr>=0)&(xr<=d.shape[1]-1)&~np.asarray(left_mask,bool)
    valid&=~np.asarray(right_mask,bool)[y,ix]&np.isfinite(partner)&(partner<0)&(np.abs(d+partner)<=lr_tolerance_px)
    z=np.full(d.shape,np.nan,np.float32)
    z[valid]=FOCAL*BASELINE/d[valid]
    return z,valid

class SGBMStereo:
    """One fixed baseline; padding removes algorithmic disparity-window border loss.

    Padding is never accepted as a valid partner: final confidence enforces native
    image support and bilateral masks. True overlap is bounded by the actual rig.
    """
    def __init__(self):
        import cv2
        self.cv2=cv2;self.pad=448
        config=dict(numDisparities=448,blockSize=5,P1=8*3*25,P2=32*3*25,
                    disp12MaxDiff=1,uniquenessRatio=10,speckleWindowSize=100,
                    speckleRange=2,preFilterCap=31,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.left=cv2.StereoSGBM_create(minDisparity=0,**config)
        self.right=cv2.StereoSGBM_create(minDisparity=-448,**config)
    def predict(self,left,right,left_mask,right_mask):
        l,r=rectify_pair(left,right)
        lm,rm=rectify_pair(left_mask,right_mask)
        cv=self.cv2;p=self.pad
        lp=cv.copyMakeBorder(l,0,0,p,p,cv.BORDER_CONSTANT,value=0)
        rp=cv.copyMakeBorder(r,0,0,p,p,cv.BORDER_CONSTANT,value=0)
        dl=self.left.compute(lp,rp).astype(np.float32)/16
        dr=self.right.compute(rp,lp).astype(np.float32)/16
        dl=dl[:,p:-p];dr=dr[:,p:-p]
        z,valid=disparity_depth(dl,dr,lm,rm)
        return dict(depth_m=restore_native(z),confidence=restore_native(valid),disparity_px=dl)
