"""Metric reconstruction from oriented 2D tangent lines, never scene geometry."""
import numpy as np

AX = np.diag([1., -1., -1.])

def solve_shape(records, family, pose_key, cfg):
    if len(records) < 2:
        return None, 'views'
    centres = np.array([r[pose_key][:3, 3] for r in records])
    if max(np.linalg.norm(centres-centres[i], axis=1).max() for i in range(len(centres))) < cfg['min_camera_baseline_m']:
        return None, 'baseline'
    aa=[];bb=[]
    for r in records:
        n = r['lines'] @ (r[pose_key][:3,:3] @ AX).T
        if family == 1:
            n = n[:,:2]
        norm = np.linalg.norm(n, axis=1)
        valid = norm > 1e-10
        n = n[valid] / norm[valid,None]
        aa.append(np.c_[n, -np.ones(len(n))])
        bb.append(n @ r[pose_key][:n.shape[1],3])
    A=np.concatenate(aa);b=np.concatenate(bb)
    if len(A) < A.shape[1] or np.linalg.matrix_rank(A) != A.shape[1]:
        return None, 'rank'
    cond=float(np.linalg.cond(A))
    if cond > cfg['max_condition_number']:
        return None, 'condition'
    value=np.linalg.lstsq(A,b,rcond=None)[0]
    residual=float(np.sqrt(np.mean((A@value-b)**2)))
    if not np.isfinite(value).all() or value[-1] <= 0 or residual > cfg['max_tangent_equation_rms_m']:
        return None, 'residual_or_radius'
    result=dict(family=family, centre=value[:-1], radius=value[-1], condition=cond, rms=residual)
    if family == 1:
        heights=[]
        for side in range(2):
            coeff=[]
            for r in records:
                rays=r['rims'][side]
                if not len(rays):continue
                direction=rays @ (r[pose_key][:3,:3]@AX).T
                direction=direction[abs(direction[:,2])>1e-8]
                if not len(direction):continue
                slope=direction[:,:2]/direction[:,2,None]
                origin=r[pose_key][:3,3]
                offset=origin[:2]-slope*origin[2]-value[:2]
                coeff.append(np.c_[np.sum(slope*slope,axis=1),2*np.sum(offset*slope,axis=1),np.sum(offset*offset,axis=1)-value[-1]**2])
            if not coeff:
                heights.append(None);continue
            q=np.concatenate(coeff)
            if len(q)<3 or np.linalg.matrix_rank(q[:,:2])<2 or np.linalg.cond(q[:,:2])>cfg['max_condition_number']:
                heights.append(None);continue
            sol=np.linalg.lstsq(q[:,:2],-q[:,2],rcond=None)[0]
            height=float(sol[1]);err=float(np.sqrt(np.mean((q[:,0]*height**2+q[:,1]*height+q[:,2])**2)))
            heights.append(height if np.isfinite(height) and err<=.02 else None)
        result['bottom'],result['top']=heights
    return result,'accepted'

def ray_depth(rays, T, shape):
    d=rays@(T[:3,:3]@AX).T;origin=T[:3,3]
    if shape['family']==0:
        v=origin-shape['centre'];qa=np.sum(d*d,axis=1);qb=d@v;qc=np.dot(v,v)-shape['radius']**2
    else:
        v=origin[:2]-shape['centre'];qa=np.sum(d[:,:2]**2,axis=1);qb=d[:,:2]@v;qc=np.dot(v,v)-shape['radius']**2
    disc=qb*qb-qa*qc
    z=np.full(len(rays),np.nan);good=(disc>=0)&(qa>1e-12)
    low=np.divide(-qb-np.sqrt(np.maximum(0,disc)),qa,out=np.full(len(rays),np.nan),where=qa>1e-12)
    high=np.divide(-qb+np.sqrt(np.maximum(0,disc)),qa,out=np.full(len(rays),np.nan),where=qa>1e-12)
    z[good]=np.where(low[good]>0,low[good],high[good])
    if shape['family']==1:
        zz=origin[2]+d[:,2]*z
        inside=np.ones(len(z),bool)
        if shape['bottom'] is not None:inside &= zz >= shape['bottom']-1e-9
        if shape['top'] is not None:inside &= zz <= shape['top']+1e-9
        z[~inside]=np.nan
        for cap in [shape['bottom'],shape['top']]:
            if cap is None:continue
            capz=np.divide(cap-origin[2],d[:,2],out=np.full(len(z),np.nan),where=abs(d[:,2])>1e-10)
            xy=origin[:2]+d[:,:2]*capz[:,None]
            valid=(capz>0)&(np.linalg.norm(xy-shape['centre'],axis=1)<=shape['radius'])
            choose=valid&(~np.isfinite(z)|(capz<z));z[choose]=capz[choose]
    z[z<=0]=np.nan
    return z
