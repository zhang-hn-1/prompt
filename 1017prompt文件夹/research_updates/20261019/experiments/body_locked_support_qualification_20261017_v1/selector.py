"""Dual-frame point-set compatibility; not point correspondence or error bound.

An untrained advisory for NEW-capture admission. History includes actual source
clouds whether risk was admitted or not. It never clears history or writes free.
"""
import numpy as np
from scipy.spatial import cKDTree

class MotionCompatibility:
    def __init__(self):
        self.frame=-1;self.source_time=-np.inf;self.now=-np.inf;self.history=[];self.eyes=None

    def update(self,frame,source_time,now,body,clouds):
        if isinstance(frame,bool) or not isinstance(frame,(int,np.integer)) or frame<=self.frame or not np.isfinite([source_time,now]).all() or source_time>now or source_time<=self.source_time or now<self.now:
            raise ValueError('strict new causal capture before payload')
        T=np.asarray(body,float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-6,rtol=0) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6,rtol=0) or abs(np.linalg.det(T[:3,:3])-1)>1e-6:
            raise ValueError('finite rigid estimated-body transform required')
        if not isinstance(clouds,(list,tuple)) or not clouds or (self.eyes is not None and len(clouds)!=self.eyes):raise ValueError('fixed nonzero source-eye list')
        world=[];local=[]
        for cloud in clouds:
            w=np.asarray(cloud,float)
            if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():raise ValueError('finite Nx3 current source points')
            world.append(w.copy());local.append((w-T[:3,3])@T[:3,:3])
        recent=[h for h in self.history if source_time-h['time_s']<=10.]
        anchor=None
        for h in reversed(recent):
            if np.linalg.norm(T[:3,3]-h['body'][:3,3])>=.2:
                anchor=h;break
        dw=[];db=[];flags=[];eyes=[]
        for e,(w,v) in enumerate(zip(world,local)):
            ww=np.full(len(w),np.nan);bb=np.full(len(w),np.nan)
            if anchor is not None and anchor['world_trees'][e] is not None and len(w):
                ww=anchor['world_trees'][e].query(w)[0];bb=anchor['body_trees'][e].query(v)[0]
            dw.append(ww);db.append(bb);flags.append(np.isfinite(ww)&np.isfinite(bb)&(ww>=.2)&(bb<=.1));eyes.append(np.full(len(w),e,int))
        # No state changes before all inputs/queries succeeded.
        saved=dict(frame=int(frame),time_s=float(source_time),body=T.copy(),
            world_trees=[cKDTree(w,copy_data=True) if len(w) else None for w in world],
            body_trees=[cKDTree(v,copy_data=True) if len(v) else None for v in local])
        self.history=recent+[saved];self.frame=int(frame);self.source_time=float(source_time);self.now=float(now);self.eyes=len(world)
        return dict(point_witness=np.concatenate(flags),distance_world=np.concatenate(dw),distance_body=np.concatenate(db),point_eye=np.concatenate(eyes),
            anchor_frame=-1 if anchor is None else anchor['frame'],anchor_time_s=None if anchor is None else anchor['time_s'],
            anchor_translation_m=None if anchor is None else float(np.linalg.norm(T[:3,3]-anchor['body'][:3,3])))

def joint_actions(source_cell_row,point_witness,cell_count):
    inverse=np.asarray(source_cell_row);w=np.asarray(point_witness)
    if isinstance(cell_count,bool) or not isinstance(cell_count,(int,np.integer)) or cell_count<0 or inverse.ndim!=1 or not np.issubdtype(inverse.dtype,np.integer) or w.shape!=inverse.shape or w.dtype!=bool or (inverse<0).any() or (inverse>=cell_count).any():raise ValueError('original sourcepoint-to-jointcell index required')
    count=np.bincount(inverse,minlength=cell_count);wcount=np.bincount(inverse,weights=w.astype(int),minlength=cell_count)
    return ((count>0)&(wcount==count)).astype(float)
