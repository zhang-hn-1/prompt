"""Frozen causal ORB/known-pose triangulation and sparse source-point memory.

No plane/layout/ID/depth truth inputs, free-space carving or navigation certificate.
"""
import copy
import cv2
import numpy as np

def rotation(pose):return np.asarray(pose,float)[:3,:3]@np.diag([1.,-1.,-1.])

def full_support(xy,size,mask):
    mask=np.asarray(mask,bool);x,y=np.rint(xy).astype(int);r=int(np.ceil(float(size)*np.sqrt(2)/2))
    return bool(x-r>=0 and y-r>=0 and x+r<mask.shape[1] and y+r<mask.shape[0] and not mask[y-r:y+r+1,x-r:x+r+1].any())

def extract(rgb,mask):
    gray=cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2GRAY)
    orb=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=31,patchSize=31,fastThreshold=20,WTA_K=2)
    keys,desc=orb.detectAndCompute(gray,(~np.asarray(mask,bool)).astype(np.uint8)*255)
    if desc is None:return np.empty((0,2)),np.empty((0,32),np.uint8),np.empty(0)
    ix=sorted([i for i,k in enumerate(keys) if full_support(k.pt,k.size,mask)],key=lambda i:(-keys[i].response,keys[i].pt[1],keys[i].pt[0],keys[i].octave))[:256]
    return np.array([keys[i].pt for i in ix]).reshape(-1,2),desc[ix],np.array([keys[i].size for i in ix])

def causal_sources(reference,sources):
    t=float(reference['time_s']);out=[];seen=set()
    for s in sources:
        st=float(s['time_s'])
        if not np.isfinite(st):raise ValueError('invalid source time')
        if st>t:continue
        identity=(int(s['frame']),s['eye'],st)
        if st==t and s['eye']==reference['eye']:continue
        if st==t and int(s['frame'])!=int(reference['frame']):raise ValueError('inconsistent simultaneous source')
        if identity in seen:raise ValueError('duplicate source')
        seen.add(identity);out.append(s)
    return out

def epipolar_distances(q,s,pose0,pose1,K):
    q=np.c_[q,np.ones(len(q))];s=np.c_[s,np.ones(len(s))];r0=rotation(pose0);r1=rotation(pose1)
    t=r1.T@(np.asarray(pose0)[:3,3]-np.asarray(pose1)[:3,3]);R=r1.T@r0
    x,y,z=t;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
    inv=np.linalg.inv(K);F=inv.T@cross@R@inv
    l1=q@F.T;l0=s@F;num=np.abs(l1@s.T)
    den1=np.linalg.norm(l1[:,:2],axis=1)[:,None];den0=np.linalg.norm(l0[:,:2],axis=1)[None,:]
    with np.errstate(divide='ignore',invalid='ignore'):d=np.maximum(num/den1,num/den0)
    return np.where(np.isfinite(d),d,np.inf)

def match_descriptors(q,qd,s,sd,pose0,pose1,K):
    if len(q)==0 or len(s)<2:return []
    epi=epipolar_distances(q,s,pose0,pose1,K);distance=np.empty(epi.shape,float)
    for j,row in enumerate(qd):distance[j]=np.unpackbits(np.bitwise_xor(row,sd),axis=1).sum(1)
    cost=np.where(epi<=1.5,distance,np.inf);reverse=cost.argmin(0);out=[]
    for i,row in enumerate(cost):
        order=np.argsort(row,kind='stable');a,b=order[:2]
        if np.isfinite(row[b]) and row[a]<=64 and row[a]<.75*row[b] and reverse[a]==i:out.append((i,int(a)))
    return out

def _point(a,b,pose0,pose1,K):
    rays=[]
    for xy,p in [(a,pose0),(b,pose1)]:
        r=rotation(p)@np.linalg.solve(K,np.r_[xy,1]);rays.append(r/np.linalg.norm(r))
    delta=np.asarray(pose1)[:3,3]-np.asarray(pose0)[:3,3]
    lam=np.linalg.lstsq(np.c_[rays[0],-rays[1]],delta,rcond=None)[0]
    point=(np.asarray(pose0)[:3,3]+lam[0]*rays[0]+np.asarray(pose1)[:3,3]+lam[1]*rays[1])/2
    return point,lam,rays

def triangulate(a,b,pose0,pose1,K):
    a=np.asarray(a,float);b=np.asarray(b,float);point,lam,rays=_point(a,b,pose0,pose1,K)
    theta=np.degrees(np.arccos(np.clip(rays[0]@rays[1],-1,1)))
    if not np.isfinite(point).all() or min(lam)<=0 or theta<.5:return None
    projected=[];depths=[]
    for p in [pose0,pose1]:
        op=rotation(p).T@(point-np.asarray(p)[:3,3]);depths.append(op[2]);x=K@op;projected.append(x[:2]/x[2])
    error=max(np.linalg.norm(projected[0]-a),np.linalg.norm(projected[1]-b))
    if min(depths)<=0 or error>1.5:return None
    variations=[]
    for j in range(4):
        for sign in [-1.,1.]:
            ab=np.r_[a,b].copy();ab[j]+=sign;pp,ll,_=_point(ab[:2],ab[2:],pose0,pose1,K)
            variations.append(np.linalg.norm(pp-point) if min(ll)>0 and np.isfinite(pp).all() else np.inf)
    return dict(world=point.tolist(),sigma_m=float(max(variations)),parallax_deg=float(theta),reprojection_px=float(error),depth_m=float(depths[0]))

def predict_features(reference,sources,K):
    sources=causal_sources(reference,sources);q=reference['xy'];candidates=[[] for _ in q]
    for s in sources:
        for i,j in match_descriptors(q,reference['desc'],s['xy'],s['desc'],reference['pose'],s['pose'],K):
            r=triangulate(q[i],s['xy'][j],reference['pose'],s['pose'],K)
            if r is None or r['sigma_m']>.1:continue
            r.update(query=[reference['frame'],reference['eye'],i,reference['time_s']],source=[s['frame'],s['eye'],j,s['time_s']]);candidates[i].append(r)
    accepted=[];conflicts=0
    for pool in candidates:
        if not pool:continue
        if any(np.linalg.norm(np.array(a['world'])-b['world'])>.1 for a in pool for b in pool):conflicts+=1;continue
        accepted.append(max(pool,key=lambda r:r['parallax_deg']))
    return dict(query_xy=q.tolist(),query_count=len(q),points=accepted,source_views=[[s['frame'],s['eye'],s['time_s']] for s in sources],conflicting_queries=conflicts)

class FeatureMemory:
    def __init__(self,max_age_s=10):self.max_age=float(max_age_s);self.time=-np.inf;self.points=[];self.pose_sigma=0
    def update(self,points,time_s,pose_sigma_m):
        if not np.isfinite(time_s) or time_s<self.time:raise ValueError('noncausal memory time')
        if any(p['query'][3]>time_s or p['source'][3]>time_s for p in points):raise ValueError('future point source')
        self.time=float(time_s);self.pose_sigma=float(pose_sigma_m)
        self.points=[p for p in self.points if time_s-p['last_seen_s']<=self.max_age]
        if not np.isfinite(self.pose_sigma) or self.pose_sigma>.05:return
        for p in points:
            if np.isfinite(p['world']).all() and np.isfinite(p['sigma_m']) and p['sigma_m']<=.1:self.points.append(copy.deepcopy({**p,'last_seen_s':float(time_s)}))
    def snapshot(self,time_s):
        if time_s<self.time:raise ValueError('past query reads future memory')
        if not np.isfinite(self.pose_sigma) or self.pose_sigma>.05:return []
        return [copy.deepcopy(p) for p in self.points if time_s-p['last_seen_s']<=self.max_age]
    def certified_free(self,xy,time_s):return False
