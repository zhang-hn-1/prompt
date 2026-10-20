"""Causal partial RGB support sets; all output geometry remains a hypothesis."""
import copy
import cv2
import numpy as np

def inflate_xy(xy, radius):
    if not np.isfinite(radius) or not 0 <= radius <= .2: raise ValueError('frozen uncertainty domain')
    xy=np.asarray(xy,float).reshape(-1,2)
    if not np.isfinite(xy).all(): raise ValueError('finite centres required')
    if not len(xy): return xy.copy()
    n=int(np.ceil(radius/.1)); offsets=np.array([(i*.1,j*.1) for i in range(-n,n+1) for j in range(-n,n+1) if np.hypot(i*.1,j*.1)<=radius+1e-12])
    return np.unique(np.round((xy[:,None,:]+offsets).reshape(-1,2),10),axis=0)

class PartialCueSetSource:
    def __init__(self,radius):
        inflate_xy(np.empty((0,2)),radius)
        self.radius=radius;self.last_time=-np.inf;self.views=[];self.model=None;self.birth_packet=None;self.class_masks={}
    def update(self,packet,rgb,poses,masks,qualified_seed,now,frame,root_xy,goal_xy,initial_start_xy):
        t=float(packet['time_s'])
        if not np.isfinite(t) or abs(t-now)>1e-9 or t<self.last_time:raise ValueError('current source clock required before payload')
        if int(packet['frame'])!=frame or abs(frame-10*t)>1e-6 or int(packet['frame_width'])!=640:raise ValueError('source identity')
        view=dict(frame=frame,time_s=t,rgb={},poses={},masks={})
        for eye in ['left','right']:
            im=np.asarray(rgb[eye+'_rgb']);T=np.asarray(poses[eye+'_camera_usd_world']);mask=np.asarray(masks[eye])
            if im.shape!=(480,640,3) or im.dtype!=np.uint8 or mask.shape!=(480,640) or mask.dtype!=bool:raise ValueError('native RGB/CAD contract')
            if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('rigid USD pose')
            view['rgb'][eye]=im.copy();view['poses'][eye]=T.copy();view['masks'][eye]=mask.copy()
        self.last_time=t;self.views=(self.views+[view])[-4:]
        base=dict(time_s=t,frame=frame,source_seed_time_s=-1.,possible_xy=np.empty((0,2)),raw_world_voxel_centres=np.empty((0,3)),model_hypothesis=True,measured_geometry=False,certified_free=False,source_model_admitted=False,pending_seed=self.model is None,calibrated_uncertainty_m=self.radius)
        if self.model is None and qualified_seed:
            world=np.asarray(packet['world']);haz=(world[:,2]>.05)&(world[:,2]<=1.5)
            if haz.sum()<100 or len(np.unique(np.floor(world[haz,:2]/.1),axis=0))<2:raise ValueError('qualified seed support')
            colors=[]
            for eye,key in [('left','query_pixel_id'),('right','partner_pixel_id')]:
                ids=np.asarray(packet[key])[haz]
                if not np.issubdtype(ids.dtype,np.integer) or np.any(ids<0) or np.any(ids>=480*640):raise ValueError('pixel UID')
                lab=cv2.cvtColor(view['rgb'][eye],cv2.COLOR_RGB2LAB).astype(float);colors.append(lab.reshape(-1,3)[ids])
            c=np.vstack(colors);centre=np.median(c,axis=0);scale=np.maximum(8.,1.4826*np.median(np.abs(c-centre),axis=0))
            self.model=dict(centre=centre,scale=scale,time_s=t,frame=frame)
            root=np.asarray(root_xy);goal=np.asarray(goal_xy);lo=np.minimum(root,goal)-2;hi=np.maximum(root,goal)+2
            axes=[np.arange(lo[k]+.05,hi[k],.1) for k in range(2)]+[np.arange(.15,1.5,.1)]
            self.vox=np.stack(np.meshgrid(*axes,indexing='ij'),-1).reshape(-1,3);self.start=np.asarray(initial_start_xy).copy()
        if self.model is None:return base
        w=self.vox;positive=np.zeros((len(w),2),int);exposures=np.zeros((len(w),len(self.views)),bool);self.class_masks={}
        for j,v in enumerate(self.views):
            if v['time_s']>now:raise ValueError('future buffered exposure')
            for e,eye in enumerate(['left','right']):
                lab=cv2.cvtColor(v['rgb'][eye],cv2.COLOR_RGB2LAB).astype(float)
                classified=(np.linalg.norm((lab-self.model['centre'])/self.model['scale'],axis=-1)<=3)&~v['masks'][eye]
                self.class_masks[(v['frame'],eye)]=classified
                T=v['poses'][eye];local=(w-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]))
                with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
                valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=3)&(uv[:,0]<636)&(uv[:,1]>=3)&(uv[:,1]<476)&(np.linalg.norm(w[:,:2]-root_xy,axis=1)>.55)
                idx=np.flatnonzero(valid);x,y=np.rint(uv[idx]).astype(int).T
                blocked=cv2.dilate(v['masks'][eye].astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool)
                idx=idx[~blocked[y,x]];x,y=np.rint(uv[idx]).astype(int).T;chosen=idx[classified[y,x]]
                positive[chosen,e]+=1;exposures[chosen,j]=True
        keep=(positive>0).all(1)&(exposures.sum(1)>=2)&(np.linalg.norm(w[:,:2]-self.start,axis=1)>.7)
        base.update(source_seed_time_s=self.model['time_s'],pending_seed=False,raw_world_voxel_centres=w[keep],possible_xy=inflate_xy(w[keep,:2],self.radius),support_frames=np.array([v['frame'] for v in self.views]),support_times_s=np.array([v['time_s'] for v in self.views]),hypothesis_birth_time_s=t,source_model_birth_time_s=self.model['time_s'])
        if self.birth_packet is None:self.birth_packet=copy.deepcopy(base)
        return base
