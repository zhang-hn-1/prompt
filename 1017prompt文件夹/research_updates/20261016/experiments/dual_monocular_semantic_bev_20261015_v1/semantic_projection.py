from projection import *
def project_semantic(zs,classes,pose,masks,origin):
 arms=[];conflicts=[]
 for j,e in enumerate(['left','right']):
  w,q=world_points(zs[j],pose[e+'_camera_usd_world'],masks[e],pose['root_position_world_m']);k=classes[j].reshape(-1)[q]==1;w=w[k];q=q[k]
  other=['right','left'][j];keep,n,c=consistent(w,zs[1-j],pose[other+'_camera_usd_world'],masks[other]);arms.append((w,q,keep));conflicts.append(dict(overlap_queries=n,conflict_queries=c))
 fused=np.concatenate([a[0][a[2]] for a in arms],axis=0)
 return [cells(a[0],origin) for a in arms]+[cells(fused,origin)],conflicts
