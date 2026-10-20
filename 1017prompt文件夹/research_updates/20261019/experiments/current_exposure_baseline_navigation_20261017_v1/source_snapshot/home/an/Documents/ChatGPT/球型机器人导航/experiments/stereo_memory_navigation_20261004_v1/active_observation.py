"""Acquisition-only known-map guard and finite-time boundary around frozen ORB.

Never changes the frozen feature predictor, PID, or unknown observation labels.
The map gate is a privileged 2D experiment precondition, not RGB safe navigation.
"""
import numpy as np
from feature_tracks import predict_features,FeatureMemory
from guarded_response import applicability
from route_geometry_audit import hazard_clearance
from primitive_planner import assess_support

def finite_time(time_s):
    if not np.isfinite(time_s):raise ValueError('finite time required')

def guarded_predict(reference,sources,K):
    finite_time(reference['time_s']);past=[]
    for s in sources:
        finite_time(s['time_s'])
        if s['time_s']<=reference['time_s']:past.append(s)
    return predict_features(reference,past,K)

class SafeFeatureMemory(FeatureMemory):
    def update(self,points,time_s,pose_sigma_m):
        finite_time(time_s)
        for p in points:
            finite_time(p['query'][3]);finite_time(p['source'][3])
        super().update(points,time_s,pose_sigma_m)
    def snapshot(self,time_s):
        finite_time(time_s);return super().snapshot(time_s)

def admit_observation(current,execution,entry,labels,origin,resolution):
    gate=applicability(current,execution,entry)
    yaw=current['yaw'];c,s=np.cos(yaw),np.sin(yaw)
    xy=np.asarray(entry['xy'])@np.array([[c,-s],[s,c]]).T+np.asarray(current['position'])[:2]
    # Body .5 + original margin .1 + frozen finite response error gate .1.
    radius=.7;required=radius+np.sqrt(2)*resolution/2
    support=assess_support(xy,labels,origin,resolution,radius)
    iy,ix=np.nonzero(np.asarray(labels)>0);hazards=np.asarray(origin)+np.c_[iy,ix]*resolution
    path=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>0]]
    clearance=hazard_clearance(path,hazards)
    admitted=gate['applicable'] and support['support_passed'] and (clearance is None or clearance>=required)
    return dict(admitted=bool(admitted),applicability=gate,world_xy=xy.tolist(),required_hazard_center_clearance_m=float(required),hazard_center_clearance_m=clearance,support=support,full_3d_certified=False,role='known-map controlled acquisition precondition, not visual map/control certificate')
