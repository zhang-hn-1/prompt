"""Current-to-past LK tracks with independent multiview triangulation checks."""
from collections import deque
from pathlib import Path
import sys
import cv2
import numpy as np
HERE=Path(__file__).parent
sys.path.insert(0,str(HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from geometry import camera_pose,project,FOCAL,CX,CY
K=np.array([[FOCAL,0,CX],[0,FOCAL,CY],[0,0,1.]])


def triangulate_matches(uv_a,pose_a,uv_b,pose_b):
    uv_a=np.asarray(uv_a,dtype=float).reshape(-1,2);uv_b=np.asarray(uv_b,dtype=float).reshape(-1,2)
    if uv_a.shape!=uv_b.shape:raise ValueError('corresponding pixels required')
    n=len(uv_a)
    if not n:return np.empty((0,3)),np.zeros(0,dtype=bool)
    if not .08<=np.linalg.norm(pose_a[0]-pose_b[0])<=1.2:return np.zeros((n,3)),np.zeros(n,dtype=bool)
    projection=lambda pose:K@np.column_stack((pose[1].T,-pose[1].T@pose[0]))
    homogeneous=cv2.triangulatePoints(projection(pose_a),projection(pose_b),uv_a.T,uv_b.T).T
    with np.errstate(divide='ignore',invalid='ignore'):points=homogeneous[:,:3]/homogeneous[:,3,None]
    a,za=project(points,pose_a);b,zb=project(points,pose_b)
    ra=points-pose_a[0];rb=points-pose_b[0]
    with np.errstate(divide='ignore',invalid='ignore'):angle=np.arccos(np.clip(np.sum(ra*rb,axis=1)/(np.linalg.norm(ra,axis=1)*np.linalg.norm(rb,axis=1)),-1,1))
    accepted=(np.isfinite(points).all(axis=1)&(za>=.55)&(za<=8)&(zb>=.55)&(zb<=8)&
              (np.linalg.norm(a-uv_a,axis=1)<=1)&(np.linalg.norm(b-uv_b,axis=1)<=1)&(angle>=np.deg2rad(.5)))
    return points,accepted


class TemporalTracks:
    def __init__(self,masks):
        self.masks={eye:np.asarray(masks[eye],bool).copy() for eye in ('left','right')}
        self.history={eye:deque() for eye in self.masks};self.last_time=-np.inf

    def update(self,left,right,xy,q,time_s):
        t=float(time_s)
        if not np.isfinite(t) or t<self.last_time-1e-8:raise ValueError('only monotonic current/past input allowed')
        self.last_time=t;result={}
        for eye,rgb in (('left',left),('right',right)):
            gray=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2GRAY);pose=camera_pose(xy,q,eye);history=self.history[eye]
            while history and history[0][0]<t-1.:history.popleft()
            past=[]
            for age in (.3,.6,.9):
                if not history:break
                entry=min(history,key=lambda h:abs((t-h[0])-age))
                if abs((t-entry[0])-age)<=.06 and .08<=np.linalg.norm(pose[0]-entry[2][0])<=1.2:
                    past.append(entry)
            mask=cv2.erode((~self.masks[eye]).astype(np.uint8),np.ones((15,15),np.uint8))*255
            corners=cv2.goodFeaturesToTrack(gray,500,.01,7,mask=mask,blockSize=7)
            if corners is None:corners=np.empty((0,1,2),np.float32)
            n=len(corners);all_points=[];all_valid=[]
            for _,old,old_pose in past:
                if not n:break
                tracked,ok,err=cv2.calcOpticalFlowPyrLK(gray,old,corners,None,winSize=(21,21),maxLevel=3)
                returned,back_ok,_=cv2.calcOpticalFlowPyrLK(old,gray,tracked,None,winSize=(21,21),maxLevel=3)
                uv=corners[:,0];old_uv=tracked[:,0];ii=np.rint(old_uv[:,0]).astype(int);jj=np.rint(old_uv[:,1]).astype(int)
                valid=(ok[:,0]>0)&(back_ok[:,0]>0)&(err[:,0]<=20)&(np.linalg.norm(returned[:,0]-uv,axis=1)<=.75)
                valid&=(ii>=7)&(ii<633)&(jj>=7)&(jj<473)
                valid&=mask[np.clip(jj,0,479),np.clip(ii,0,639)]>0
                points,geometric=triangulate_matches(uv,pose,old_uv,old_pose)
                all_points.append(points);all_valid.append(valid&geometric)
            accepted=np.zeros(n,bool);median=np.zeros((n,3));support=np.zeros(n,int)
            if len(all_points)>=2:
                stacked=np.stack(all_points);valid=np.stack(all_valid)
                for i in range(n):
                    good=stacked[valid[:,i],i]
                    if len(good)<2:continue
                    center=np.median(good,axis=0);tol=max(.15,.05*np.linalg.norm(center-pose[0]));consistent=np.linalg.norm(good-center,axis=1)<=tol
                    if consistent.sum()>=2:
                        accepted[i]=True;median[i]=np.median(good[consistent],axis=0);support[i]=consistent.sum()
            result[eye]={'uv':corners[:,0][accepted].copy(),'points_world':median[accepted].copy(),
                         'support':support[accepted].copy(),'corners':n,'past_views':len(past)}
            history.append((t,gray.copy(),(pose[0].copy(),pose[1].copy())))
        return result
