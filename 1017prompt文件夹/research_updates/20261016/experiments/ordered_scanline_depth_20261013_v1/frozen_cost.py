"""Frozen current-pair Census correspondence. No SGBM, GT, history or map input."""
import cv2
import numpy as np
import torch

INVALID=30000

def census_codes(gray):
    a=np.asarray(gray)
    if a.ndim!=2 or a.dtype!=np.uint8 or min(a.shape)<9: raise ValueError('uint8 grayscale image >=9x9')
    h,w=a.shape;out=np.zeros((h,w),np.int64);center=a[3:h-3,3:w-3];k=0
    for dy in range(-3,4):
        for dx in range(-3,4):
            if dy==dx==0:continue
            out[3:h-3,3:w-3] |= (a[3+dy:h-3+dy,3+dx:w-3+dx]<center).astype(np.int64)<<k;k+=1
    return out

def popcount48(x):
    x=x-((x>>1)&0x5555555555555555)
    x=(x&0x3333333333333333)+((x>>2)&0x3333333333333333)
    x=(x+(x>>4))&0x0F0F0F0F0F0F0F0F
    x=x+(x>>8);x=x+(x>>16);x=x+(x>>32)
    return x&0x7F

class CensusStereo:
    def __init__(self,device='cpu',focal=584.8888888888889,baseline=.6):
        self.device=torch.device(device);self.focal=float(focal);self.baseline=float(baseline)
    @torch.no_grad()
    def match_rectified(self,left,right,left_mask,right_mask):
        l=np.asarray(left);r=np.asarray(right)
        if l.shape!=r.shape or l.ndim!=2 or l.dtype!=np.uint8 or r.dtype!=np.uint8 or np.shape(left_mask)!=l.shape or np.shape(right_mask)!=l.shape:raise ValueError('aligned uint8 pair and CAD masks required')
        h,w=l.shape;dev=self.device
        def support(mask):
            return cv2.erode((~np.asarray(mask,bool)).astype(np.uint8),np.ones((9,9),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
        L=torch.as_tensor(census_codes(l),device=dev);R=torch.as_tensor(census_codes(r),device=dev)
        vl=torch.as_tensor(support(left_mask),device=dev);vr=torch.as_tensor(support(right_mask),device=dev)
        xs=torch.arange(w,device=dev)[None,None,:];ds=torch.arange(1,w,device=dev)
        cost=torch.full((w-1,h,w),INVALID,dtype=torch.int16,device=dev);reverse=torch.full_like(cost,INVALID)
        # Integer operations and exact sums: batch is a memory bound, not a search parameter.
        for start in range(0,w-1,24):
            d=ds[start:start+24,None,None];xr=xs-d;ix=xr.clamp(0,w-1)
            shifted=R[None].expand(len(d),h,w).gather(2,ix.expand(-1,h,-1))
            raw=popcount48(L[None]^shifted).to(torch.int16)
            total=torch.zeros_like(raw)
            for dy in range(-1,2):
                for dx in range(-1,2):total[:,1:h-1,1:w-1]+=raw[:,1+dy:h-1+dy,1+dx:w-1+dx]
            valid=vl[None]&vr[None].expand(len(d),h,w).gather(2,ix.expand(-1,h,-1))&(xr>=0)
            total=torch.where(valid,total,INVALID);cost[start:start+len(d)]=total
            xl=xs+d;reverse[start:start+len(d)]=torch.where(xl<w,total.gather(2,xl.clamp(0,w-1).expand(-1,h,-1)),INVALID)
        def winner(c):
            best,idx=c.min(0);second=torch.full_like(best,INVALID)
            for start in range(0,w-1,24):
                ids=torch.arange(start,min(start+24,w-1),device=dev)[:,None,None]
                v=torch.where(torch.abs(ids-idx[None])>1,c[start:start+24],INVALID)
                second=torch.minimum(second,v.min(0).values)
            valid=(best<=108)&(second<INVALID)&(second.to(torch.int32)*10>best.to(torch.int32)*11)&(second>best)
            prev=c.gather(0,(idx-1).clamp(0,w-2)[None])[0].float();nxt=c.gather(0,(idx+1).clamp(0,w-2)[None])[0].float();bf=best.float();den=prev+nxt-2*bf
            sub=torch.where((idx>0)&(idx<w-2)&(prev<INVALID)&(nxt<INVALID)&(den>0),(.5*(prev-nxt)/den.clamp_min(1)).clamp(-.5,.5),0.)
            return idx.float()+1+sub,valid,best,second
        dl,lv,best,second=winner(cost);dr,rv,_,_=winner(reverse)
        partner=torch.round(xs[0].expand(h,-1)-dl).long();ip=partner.clamp(0,w-1)
        valid=lv&rv.gather(1,ip)&(partner>=0)&(partner<w)&(torch.abs(dl-dr.gather(1,ip))<=1.5)
        return {k:v.cpu().numpy() for k,v in dict(disparity=dl,valid=valid,best=best,second=second).items()}
    def predict(self,left_rgb,right_rgb,left_mask,right_mask):
        if np.shape(left_rgb)!=np.shape(right_rgb) or np.ndim(left_rgb)!=3 or np.shape(left_rgb)[2]!=3:raise ValueError('paired RGB arrays')
        l=cv2.cvtColor(np.ascontiguousarray(np.rot90(left_rgb)),cv2.COLOR_RGB2GRAY);r=cv2.cvtColor(np.ascontiguousarray(np.rot90(right_rgb)),cv2.COLOR_RGB2GRAY)
        v=self.match_rectified(l,r,np.rot90(left_mask),np.rot90(right_mask));z=self.focal*self.baseline/v['disparity'];valid=v['valid']&(z>=.5)&(z<=3.)
        return dict(depth_m=np.ascontiguousarray(np.rot90(np.where(valid,z,np.nan),-1)),confidence=np.ascontiguousarray(np.rot90(valid,-1)),disparity_px=v['disparity'],cost_best=v['best'],cost_second=v['second'])
