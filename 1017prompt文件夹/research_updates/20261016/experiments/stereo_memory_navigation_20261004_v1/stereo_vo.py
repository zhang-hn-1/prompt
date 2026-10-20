"""Stereo VO: feature arrays + fixed calibration, no per-frame truth pose input."""
import numpy as np
import cv2
from feature_tracks import match_descriptors,triangulate

def temporal_matches(q,qd,s,sd):
    if len(q)==0 or len(s)<2:return []
    distance=np.empty((len(q),len(s)),float)
    for i,row in enumerate(qd):distance[i]=np.unpackbits(np.bitwise_xor(row,sd),axis=1).sum(1)
    reverse=distance.argmin(0);out=[]
    for i,row in enumerate(distance):
        a,b=np.argsort(row,kind='stable')[:2]
        if row[a]<=64 and row[a]<.75*row[b] and reverse[a]==i:out.append((i,int(a)))
    return out

def pose_from_matches(points,pixels,K):
    points=np.asarray(points,float).reshape(-1,3);pixels=np.asarray(pixels,float).reshape(-1,2)
    result=dict(accepted=False,reason='insufficient_landmarks',transform=None,landmarks=len(points),inliers=[],reprojection_p90_px=None)
    if len(points)<8 or len(points)!=len(pixels):return result
    if not np.isfinite(points).all() or not np.isfinite(pixels).all():result['reason']='nonfinite';return result
    singular=np.linalg.svd(points-points.mean(0),compute_uv=False)
    if singular[1]<1e-6:result['reason']='collinear';return result
    cv2.setRNGSeed(3072)
    ok,rv,tv,ix=cv2.solvePnPRansac(points,pixels,K,None,iterationsCount=100,reprojectionError=1.5,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
    if not ok or ix is None:result['reason']='pnp_failed';return result
    ix=ix.ravel();result['inliers']=ix.tolist()
    if len(ix)<8 or len(ix)/len(points)<.5:result['reason']='insufficient_inliers';return result
    ok,rv,tv=cv2.solvePnP(points[ix],pixels[ix],K,None,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
    R=cv2.Rodrigues(rv)[0];cam=points[ix]@R.T+tv.ravel();projection=cam@K.T
    if not ok or not np.isfinite(cam).all() or (cam[:,2]<=0).any():result['reason']='invalid_refinement';return result
    error=np.linalg.norm(projection[:,:2]/projection[:,2,None]-pixels[ix],axis=1);p90=float(np.percentile(error,90));result['reprojection_p90_px']=p90
    if p90>1.5:result['reason']='reprojection';return result
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=tv.ravel();result.update(accepted=True,reason='accepted',transform=T.tolist());return result

def estimate_increment(previous,current,K,right_optical_in_left):
    times=[float(previous['time_s']),float(current['time_s'])]
    if not np.isfinite(times).all() or times[1]<=times[0]:raise ValueError('strictly increasing finite capture times required')
    # USD-like matrices are used only as fixed relative stereo calibration.
    usd=np.diag([1.,-1.,-1.,1.]);right=np.asarray(right_optical_in_left)@usd
    q,qd=previous['left_xy'],previous['left_desc'];s,sd=previous['right_xy'],previous['right_desc']
    landmarks={};stereo=match_descriptors(q,qd,s,sd,usd,right,K)
    for i,j in stereo:
        measured=triangulate(q[i],s[j],usd,right,K)
        if measured is not None and measured['sigma_m']<=.1:landmarks[i]=measured['world']
    temporal=temporal_matches(q,qd,current['left_xy'],current['left_desc'])
    matches=[(i,j) for i,j in temporal if i in landmarks]
    result=pose_from_matches([landmarks[i] for i,j in matches],[current['left_xy'][j] for i,j in matches],K)
    result.update(stereo_matches=len(stereo),qualified_stereo_landmarks=len(landmarks),temporal_matches=len(temporal),joint_matches=len(matches),matches=[list(m) for m in matches])
    if len(landmarks)<8:result['reason']='stereo_support_insufficient'
    elif len(matches)<8:result['reason']='temporal_support_insufficient'
    return result

class PoseChain:
    def __init__(self):self.pose=np.eye(4)
    def update(self,transform):
        if self.pose is None or transform is None:self.pose=None;return None
        self.pose=self.pose@np.linalg.inv(np.asarray(transform,float));return self.pose.copy()
