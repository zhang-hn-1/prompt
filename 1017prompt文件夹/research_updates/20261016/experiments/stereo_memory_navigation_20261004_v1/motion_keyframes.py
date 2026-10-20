"""Pose-only causal keyframes; no image/partner/depth confidence or labels."""
import numpy as np
def _pose(value):
    p=np.array(value,dtype=float,copy=True)
    if p.shape!=(4,4) or not np.isfinite(p).all() or not np.allclose(p[3],[0,0,0,1],atol=1e-6) or not np.allclose(p[:3,:3].T@p[:3,:3],np.eye(3),atol=1e-5) or abs(np.linalg.det(p[:3,:3])-1)>1e-5:raise ValueError('invalid rigid pose')
    return p
def select_keyframes(current_pose,time_s,past_rows):
    t=float(time_s)
    if not np.isfinite(t):raise ValueError('invalid current time')
    current=_pose(current_pose);valid=[];identities=set()
    for row in past_rows:
        old=float(row['time_s'])
        if not np.isfinite(old):raise ValueError('invalid past time')
        if old>=t or t-old>10.:continue
        frame=int(row['frame']);identity=(frame,old)
        if identity in identities:raise ValueError('duplicate keyframe')
        identities.add(identity);p=_pose(row['pose']);baseline=np.linalg.norm(p[:3,3]-current[:3,3]);angle=np.degrees(np.arccos(np.clip((np.trace(current[:3,:3].T@p[:3,:3])-1)/2,-1,1)))
        if .15<=baseline<=.60 and angle<=20:valid.append(dict(frame=frame,time_s=old,pose=p,baseline_m=float(baseline),rotation_deg=float(angle)))
    selected=[]
    for row in sorted(valid,key=lambda v:(v['time_s'],v['frame']),reverse=True):
        if all(np.linalg.norm(row['pose'][:3,3]-s['pose'][:3,3])>=.15 for s in selected):selected.append(row)
        if len(selected)==3:break
    return selected
