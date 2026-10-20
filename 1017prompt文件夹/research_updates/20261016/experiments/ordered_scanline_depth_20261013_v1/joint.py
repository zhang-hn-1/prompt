"""Exact ordered one-to-one full-scanline assignment with explicit unmatched pixels.

Current pair only. Frozen v56 local costs; no SGBM seed, GT or temporal input.
"""
import numpy as np
import cv2
import torch
from frozen_cost import census_codes,popcount48,INVALID

@torch.jit.script
def alignment(cost:torch.Tensor, gl:torch.Tensor, gr:torch.Tensor):
    h,w,_=cost.shape
    f=torch.full((h,w+1,w+1),100000000,dtype=torch.int32,device=cost.device)
    b=torch.full_like(f,100000000)
    f[:,0,0]=0
    f[:,1:,0]=torch.cumsum(gl,1).to(torch.int32)
    f[:,0,1:]=torch.cumsum(gr,1).to(torch.int32)
    b[:,w,w]=0
    b[:,:w,w]=torch.flip(torch.cumsum(torch.flip(gl,[1]),1),[1]).to(torch.int32)
    b[:,w,:w]=torch.flip(torch.cumsum(torch.flip(gr,[1]),1),[1]).to(torch.int32)
    for k in range(2,2*w+1):
        i=torch.arange(max(1,k-w),min(w,k-1)+1,device=cost.device)
        j=k-i
        f[:,i,j]=torch.minimum(torch.minimum(f[:,i-1,j]+gl[:,i-1],f[:,i,j-1]+gr[:,j-1]),f[:,i-1,j-1]+cost[:,i-1,j-1])
    for k in range(2*w-2,-1,-1):
        i=torch.arange(max(0,k-w+1),min(w-1,k)+1,device=cost.device)
        j=k-i
        b[:,i,j]=torch.minimum(torch.minimum(b[:,i+1,j]+gl[:,i],b[:,i,j+1]+gr[:,j]),b[:,i+1,j+1]+cost[:,i,j])
    optimum=f[:,w,w]
    on_path=f[:,:w,:w]+cost+b[:,1:,1:]==optimum[:,None,None]
    possible=on_path.sum(2)
    partner=on_path.to(torch.int32).argmax(2)
    skip=(f[:,:w,:]+gl[:,:,None]+b[:,1:,:]==optimum[:,None,None]).any(2)
    forced=(possible==1)&~skip
    return partner,forced,possible,skip,optimum

class JointStereo:
    def __init__(self,device='cpu',focal=584.8888888888889,baseline=.6):
        self.device=torch.device(device);self.focal=focal;self.baseline=baseline
    @torch.no_grad()
    def costs(self,left,right,lmask,rmask):
        h,w=left.shape;dev=self.device
        support=lambda m:cv2.erode((~np.asarray(m,bool)).astype(np.uint8),np.ones((9,9),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
        L=torch.as_tensor(census_codes(left),device=dev);R=torch.as_tensor(census_codes(right),device=dev)
        vl=torch.as_tensor(support(lmask),device=dev);vr=torch.as_tensor(support(rmask),device=dev)
        xs=torch.arange(w,device=dev)[None,None,:];ds=torch.arange(1,w,device=dev)
        cost=torch.full((h,w,w),INVALID,dtype=torch.int32,device=dev)
        # Each (left,right) has exactly the previous frozen local cost.
        for start in range(0,w-1,24):
            d=ds[start:start+24,None,None];xr=xs-d;ix=xr.clamp(0,w-1)
            shifted=R[None].expand(len(d),h,w).gather(2,ix.expand(-1,h,-1))
            raw=popcount48(L[None]^shifted).to(torch.int32);total=torch.zeros_like(raw)
            for dy in range(-1,2):
                for dx in range(-1,2):total[:,1:h-1,1:w-1]+=raw[:,1+dy:h-1+dy,1+dx:w-1+dx]
            valid=vl[None]&vr[None].expand(len(d),h,w).gather(2,ix.expand(-1,h,-1))&(xr>=0)&(total<=108)
            for k in range(len(d)):
                disparity=int(ds[start+k]);leftx=torch.arange(disparity,w,device=dev)
                cost[:,leftx,leftx-disparity]=torch.where(valid[k,:,disparity:],total[k,:,disparity:],INVALID)
        return cost,vl.to(torch.int32)*54,vr.to(torch.int32)*54
    @torch.no_grad()
    def match_rectified(self,left,right,lmask,rmask):
        l=np.asarray(left);r=np.asarray(right)
        if l.shape!=r.shape or l.ndim!=2 or l.dtype!=np.uint8 or r.dtype!=np.uint8 or np.shape(lmask)!=l.shape or np.shape(rmask)!=l.shape:raise ValueError('aligned grayscale pair/masks')
        c,gl,gr=self.costs(l,r,lmask,rmask);p,ok,n,sk,opt=alignment(c,gl,gr)
        d=torch.arange(l.shape[1],device=self.device)[None]-p
        best=c.gather(2,p[:,:,None])[:,:,0]
        return {k:v.cpu().numpy() for k,v in dict(disparity=d.float(),valid=ok,best=best,possible_partners=n,skip_possible=sk,row_optimum=opt).items()}
    def predict(self,left,right,lmask,rmask):
        if np.shape(left)!=np.shape(right) or np.ndim(left)!=3 or np.shape(left)[2]!=3:raise ValueError('RGB pair')
        l=cv2.cvtColor(np.ascontiguousarray(np.rot90(left)),cv2.COLOR_RGB2GRAY);r=cv2.cvtColor(np.ascontiguousarray(np.rot90(right)),cv2.COLOR_RGB2GRAY)
        v=self.match_rectified(l,r,np.rot90(lmask),np.rot90(rmask))
        with np.errstate(divide='ignore',invalid='ignore'):z=self.focal*self.baseline/v['disparity']
        valid=v['valid']&(v['disparity']>0)&(z>=.5)&(z<=3)
        return dict(depth_m=np.ascontiguousarray(np.rot90(np.where(valid,z,np.nan),-1)),confidence=np.ascontiguousarray(np.rot90(valid,-1)),disparity_px=v['disparity'],cost_best=v['best'],possible_partners=v['possible_partners'],skip_possible=v['skip_possible'],row_optimum=v['row_optimum'])
