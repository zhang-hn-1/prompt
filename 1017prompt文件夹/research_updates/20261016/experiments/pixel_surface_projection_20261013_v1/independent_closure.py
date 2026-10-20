"""Independent saved-data review; no physics, fitting, or full relabelling."""
from common import *
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import maximum_filter,distance_transform_edt
from projection import backproject,root_from_camera
from category_projection import raster,point_packet
from category_memory import CategoryMemory
from surface_identity import categories
from rig_adapter import Rig,RigMask

def independent_metrics(y,p):
 totals=np.zeros(9,dtype=np.int64);dist=[]
 for a,b in zip(y,p):
  gt=a==2;pr=b==2;vis=a>0;ng=maximum_filter(gt,size=3,mode='constant');npred=maximum_filter(pr,size=3,mode='constant')
  totals+=np.array([pr.sum(),gt.sum(),(pr&ng).sum(),(gt&npred).sum(),(a==1).sum(),(pr&(a==1)).sum(),(pr&~maximum_filter(vis,size=3,mode='constant')).sum(),vis.sum(),(vis&maximum_filter(b>0,size=3,mode='constant')).sum()])
  if pr.any():dist.extend((distance_transform_edt(~gt)*.1)[pr] if gt.any() else [99.]*int(pr.sum()))
 n,g,mp,mg,ground,fg,uns,v,rv=map(int,totals)
 r=dict(precision=mp/max(1,n),recall=mg/max(1,g),ground_false_fraction=fg/max(1,ground),unsupported_fraction=uns/max(1,n),visible_surface_recall=rv/max(1,v),localization_p90_m=float(np.quantile(dist,.9)) if dist else None)
 r['pass_gate']=bool(n and r['precision']>=.9 and r['recall']>=.8 and r['ground_false_fraction']<=.05 and r['unsupported_fraction']<=.1 and r['visible_surface_recall']>=.7 and r['localization_p90_m']<=.15)
 return r

def main():
 status=json.loads((D/'qualify_status.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
 report=json.loads((D/'qualification.json').read_text());checked={}
 for section in ['input_sha256','output_sha256','source_sha256']:
  for p,h in report[section].items():assert sha(p)==h,(section,p)
  checked[section]=len(report[section])
 rows=[];cad=RigMask(Rig('forward',.4,.3,.05));total=0
 for name,(R,cp) in GROUPS.items():
  cfg=json.loads(cp.read_text());states=json.loads((R/'states.json').read_text());scene=json.loads((D/'scenes'/f'{name}.json').read_text());maps=np.load(D/f'{name}_maps.npz');init=np.load(R/'maps/frame_000.npz');memory=CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),name!='H');pe=[];counts=np.zeros(5,dtype=np.int64);bad=[];sample_checks=0
  files=sorted((D/'labels'/name).glob('*.npz'));assert len(files)==181
  for j,lp in enumerate(files):
   f=np.load(R/'frames'/lp.name);n=np.load(R/'known_poses'/lp.name);l=np.load(lp);i=int(lp.stem.split('_')[1]);assert i==j*5;assert float(l['time_s'])==float(f['time_s'])==float(n['time_s'])==float(states[i]['time_s']);assert int(l['frame'])==i
   root=f['root_position_world_m'];q=f['root_quaternion_world_wxyz'];rr=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();yaw=np.arctan2(rr[1,0],rr[0,0]);nr,nroot=root_from_camera(n['left_camera_usd_world']);nyaw=np.arctan2(nr[1,0],nr[0,0]);mask=cad.project(np.array(states[i]['joints']));tw=[];tc=[];nw=[];nc=[];eligible_count=labelled_count=0
   for eye in ['left','right']:
    z=f[eye+'_depth_m'];cl=l[eye+'_category'];el=l[eye+'_eligible'];assert cl.shape==el.shape==z.shape==(480,640);assert el.dtype==bool;assert set(np.unique(cl))<={0,1,2};assert not (cl[~el]>0).any()
    valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask[eye];yy,xx=np.where(valid);p=backproject(xx,yy,z[yy,xx],f[eye+'_camera_usd_world']);outside=np.linalg.norm(p-root,axis=1)>=.55;expected=np.zeros_like(el);expected[yy[outside],xx[outside]]=True;assert np.array_equal(el,expected)
    yy=yy[outside];xx=xx[outside];p=p[outside];lab=cl[yy,xx];tw.append(p);tc.append(lab);pn,cn,idx=point_packet(z,cl,n[eye+'_camera_usd_world'],nroot,mask[eye]);nw.append(pn);nc.append(cn);gtp=backproject(idx%640,idx//640,z.ravel()[idx],f[eye+'_camera_usd_world']);pe.append(np.linalg.norm(pn-gtp,axis=1));eligible_count+=len(p);labelled_count+=int((lab>0).sum());counts+=np.array([len(p),(lab>0).sum(),(lab==1).sum(),(lab==2).sum(),(lab==0).sum()])
    if i in [0,450,495,900]:
     sel=np.unique(np.concatenate([np.arange(0,len(p),max(1,len(p)//150)),np.flatnonzero(lab==0)[:100]]));a,_=categories(p[sel],scene);assert np.array_equal(a,lab[sel]);sample_checks+=len(sel)
   assert np.array_equal(raster(np.concatenate(tw),np.concatenate(tc),root,yaw),maps['target'][j]);assert np.array_equal(raster(np.concatenate(nw),np.concatenate(nc),nroot,nyaw),maps['native_oracle'][j])
   memory.update(np.concatenate(nw),np.concatenate(nc),i,float(f['time_s']),n['left_camera_usd_world'][:3,3])
   sf=i if i in [450,495,500] else 899 if i==895 else None
   if sf is not None:
    actual=np.load(D/f'{name}_memory_{sf:03d}.npz');now=float(actual['time_s']);exp=memory.snapshot(now)
    for k in ['labels','clock','origin','resolution','time_s']:assert np.array_equal(actual[k],exp[k]),(name,sf,k)
   if eligible_count>=1000 and labelled_count<.8*eligible_count:bad.append(i)
   total+=1
  m=independent_metrics(maps['target'],maps['native_oracle']);row=next(r for r in report['rows'] if r['group']==name)
  for k,v in m.items():assert row['metrics']['native_oracle'][k]==v,(name,k)
  fraction=int(counts[1])/max(1,int(counts[0]));assert fraction==row['label_coverage'];assert bad==row['label_coverage_failure_frames'];quant=np.quantile(np.concatenate(pe),[.5,.9,.95]);assert np.allclose(quant,row['world_point_error_p50_p90_p95'])
  gates=dict(label_coverage=fraction>=.95,per_frame_coverage=not bad,oracle_projection=m['pass_gate']);assert gates==row['gates'];assert all(gates.values())==row['qualification_pass']
  rows.append(dict(group=name,frames=181,pixel_masks_and_times_verified=362,full_target_native_map_identity=True,memory_snapshots_verified=[450,495,500,899],authored_label_sample_points=sample_checks,label_coverage=fraction,metrics=m,world_point_error_p50_p90_p95=quant.tolist(),qualification_pass=all(gates.values())))
  print('independent replay',name,'pass',flush=True)
 assert report['qualification_pass']==all(r['qualification_pass'] for r in rows)
 save('independent_closure.json',dict(offline_oracle_review_pass=True,qualification_pass=report['qualification_pass'],frames_verified=total,hash_counts=checked,rows=rows,label_recast_scope='Sampled frames 0/450/495/900 both eyes; uniform points plus up to100 unknown per eye. Full eligible masks/categories codes/counts/maps replayed all905 frames. No full authored-surface recast.',limits=['All groups seen development; no independent navigation or predictor performance.','Native root/camera gauge cancellation limits local raster localization; separate world-point errors are reported.','Authored compatibility approximates renderer surfaces; coverage gates retained.','Any-point memory admission versus four-count raster denominators remain distinct.'],fit=0,physics=0,review_source_sha256=sha(D/'independent_closure.py')))
if __name__=='__main__':main()
