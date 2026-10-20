"""Current per-eye metric depth prior -> points; no stereo correspondence or free cells."""
import numpy as np
F=584.8888888888889
AX=np.diag([1,-1,-1])
def world_points(z,T,mask,root,stride=2):
 y,x=np.indices(z.shape);valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask.astype(bool)&(y%stride==0)&(x%stride==0)
 yy,xx=np.where(valid);ray=np.column_stack([(xx-319.5)/F,(yy-239.5)/F,np.ones(len(xx))]);w=T[:3,3]+(ray*z[yy,xx,None])@(T[:3,:3]@AX).T
 keep=np.linalg.norm(w-root,axis=1)>=.55
 return w[keep],(yy*640+xx)[keep]
def per_eye(z,T,mask,root,stride=2):
 w,q=world_points(z,T,mask,root,stride);k=(w[:,2]>.05)&(w[:,2]<=1.5);return w[k],q[k]
def consistent(w,other_z,other_T,other_mask):
 c=(w-other_T[:3,3])@(other_T[:3,:3]@AX);positive=c[:,2]>0
 x=np.rint(F*c[:,0]/np.maximum(c[:,2],1e-9)+319.5).astype(int);y=np.rint(F*c[:,1]/np.maximum(c[:,2],1e-9)+239.5).astype(int);inside=positive&(x>=0)&(x<640)&(y>=0)&(y<480)
 k=np.where(inside)[0];usable=np.isfinite(other_z[y[k],x[k]])&~other_mask[y[k],x[k]].astype(bool)&(other_z[y[k],x[k]]>=.5)&(other_z[y[k],x[k]]<=3);k=k[usable]
 keep=np.ones(len(w),bool);keep[k]=np.abs(other_z[y[k],x[k]]-c[k,2])<=.15
 return keep,int(len(k)),int((~keep).sum())
def cells(w,origin):return np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
def project_pair(zs,pose,masks,origin):
 root=pose['root_position_world_m'];arms=[];conflicts=[]
 for j,eye in enumerate(['left','right']):
  w,q=per_eye(zs[j],pose[eye+'_camera_usd_world'],masks[eye],root);other=['right','left'][j];keep,overlap,conflict=consistent(w,zs[1-j],pose[other+'_camera_usd_world'],masks[other]);arms.append((w,q,keep));conflicts.append(dict(overlap_queries=overlap,conflict_queries=conflict))
 fused=np.concatenate([a[0][a[2]] for a in arms],axis=0)
 return [cells(a[0],origin) for a in arms]+[cells(fused,origin)],conflicts
