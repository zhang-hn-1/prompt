"""OFFLINE distance-residual footprint feasibility. GT never an online source.
Original current query IDs retained; perfect range, optional perfect category.
No rematching, no filling holes, no training or closed-model reuse.
"""
from common import *
import numpy as np
from projection import backproject,root_from_camera
from category_projection import raster
from category_memory import CategoryMemory
from scipy.spatial import cKDTree
old=(W/'experiments/learned_visible_obstacles_20261013_v1/score.py').read_text();ns={}
exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+old[old.index('def metrics('):old.index('\nrows=[]')],ns);metrics=ns['metrics']
assert not (D/'result.json').exists();inputs={};outputs={};rows=[]
oracle=json.loads((Q/'qualification.json').read_text());assert oracle['qualification_pass']
for k in ['input_sha256','output_sha256','source_sha256']:
 for p,s in oracle[k].items():assert sha(p)==s,p
for name,(R,cp) in GROUPS.items():
 paths=sorted((R/'frames').glob('*.npz'));assert [int(p.stem.split('_')[1]) for p in paths]==list(range(0,901,5))
 init=np.load(R/'maps/frame_000.npz');mem={k:CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),name!='H') for k in ['range_height','range_category']}
 grids={k:[] for k in mem};frame_rows=[];query_total=finite_total=labelled_total=0;point_error=[];counts={k:[] for k in mem};packets={};native_views={}
 for j,p in enumerate(paths):
  i=int(p.stem.split('_')[1]);f=np.load(p);pp=R/'sensor_packets'/f'source_{i:03d}.npz';pkt=np.load(pp);npth=R/'known_poses'/p.name;n=np.load(npth);lp=Q/'labels'/name/p.name;lab=np.load(lp)
  t=float(f['time_s']);assert t==float(n['time_s'])==float(pkt['time_s'])==float(lab['time_s']) and int(pkt['frame'])==i
  ids=pkt['query_pixel_id'].astype(int);assert len(np.unique(ids))==len(ids) and ((ids>=0)&(ids<480*640)).all()
  z=f['left_depth_m'].ravel()[ids];eligible=np.isfinite(z)&(z>=.5)&(z<=3);idx=ids[eligible];depth=z[eligible];cat=lab['left_category'].ravel()[idx];labelled=cat>0
  nr,root=root_from_camera(n['left_camera_usd_world']);yaw=np.arctan2(nr[1,0],nr[0,0]);world=backproject(idx%640,idx//640,depth,n['left_camera_usd_world']);outside=np.linalg.norm(world-root,axis=1)>=.55
  world=world[outside];idx=idx[outside];cat=cat[outside];depth=depth[outside]
  truth=backproject(idx%640,idx//640,depth,f['left_camera_usd_world']);point_error.append(np.linalg.norm(world-truth,axis=1))
  # Zero residual uncertainty is deliberately optimistic; original fixed height band.
  height=(world[:,2]>.05)&(world[:,2]<=1.5);classes={'range_height':np.where(height,2,0).astype(np.uint8),'range_category':cat}
  for method,cl in classes.items():
   grids[method].append(raster(world,cl,root,yaw));packets.setdefault(method,{})[i]=(world,cl,t,n['left_camera_usd_world'][:3,3])
  query_total+=len(ids);finite_total+=int(eligible.sum());labelled_total+=int(labelled.sum())
  frame_rows.append(dict(frame=i,queries=len(ids),finite_range=int(eligible.sum()),labelled=int(labelled.sum()),nohit=int((~np.isfinite(z)).sum()),finite_outside_range=int((np.isfinite(z)&~eligible).sum())))
  for path in [p,pp,npth,lp]:inputs[str(path)]=sha(path)
  if j%90==0:print(name,j+1,'/181 oracle',flush=True)
 arr={k:np.stack(v) for k,v in grids.items()};target=np.load(Q/f'{name}_maps.npz')['target'];met={k:metrics(target,v) for k,v in arr.items()};gate={}
 for method,m in met.items():gate[method]=dict(precision=m['precision']>=.9,recall=m['recall']>=.8,ground_false=m['ground_false_fraction']<=.05,unsupported=m['unsupported_fraction']<=.1,localization=m['localization_p90_m'] is not None and m['localization_p90_m']<=.15)
 out=D/f'{name}_local_maps.npz';np.savez_compressed(out,target=target,**arr);outputs[str(out)]=sha(out)
 keys=iter(range(0,901,5));nextkey=next(keys,None)
 for i in range(900):
  mp=R/'maps'/f'frame_{i:03d}.npz';a=np.load(mp);t=float(a['time_s'])
  while nextkey is not None and packets['range_height'][nextkey][2]<=t:
   for method,memory in mem.items():
    world,cl,epoch,view=packets[method][nextkey];memory.update(world,cl,nextkey,epoch,view)
   nextkey=next(keys,None)
  for method,memory in mem.items():
   s=memory.snapshot(t);counts[method].append(int((s['labels']==1).sum()))
   if i in [450,495,500,899]:
    out=D/f'{name}_{method}_memory_{i:03d}.npz';np.savez_compressed(out,**s);outputs[str(out)]=sha(out)
  inputs[str(mp)]=sha(mp)
 occ=np.load(json.loads(cp.read_text())['occupancy']);ref=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(ref);memory_metrics={}
 for method in mem:
  a=np.load(D/f'{name}_{method}_memory_899.npz');cells=np.argwhere(a['labels']==1);xy=a['origin']+cells*float(a['resolution']);bad=tree.query(xy)[0]>.15;age=float(a['time_s'])-a['clock'][tuple(cells.T)];memory_metrics[method]=dict(risk=len(cells),far015=int(bad.sum()),older10s=int((age>10).sum()),old_far015=int((bad&(age>10)).sum()))
 pe=np.concatenate(point_error);rows.append(dict(group=name,epochs=181,queries=query_total,finite_correctable=finite_total,labelled=labelled_total,finite_fraction=finite_total/max(1,query_total),point_error_p50_p90_p95=np.quantile(pe,[.5,.9,.95]).tolist(),metrics=met,gates=gate,qualification_pass=all(gate['range_height'].values()),optimistic_semantic_pass=all(gate['range_category'].values()),frames=frame_rows,map_counts=counts,memory_metrics=memory_metrics));print('RESULT',name,gate,flush=True)
 inputs[str(cp)]=sha(cp);occpath=Path(json.loads(cp.read_text())['occupancy']);inputs[str(occpath)]=sha(occpath)
save('result.json',dict(rows=rows,qualified=all(r['qualification_pass'] for r in rows),optimistic_qualified=all(r['optimistic_semantic_pass'] for r in rows),input_sha256=inputs,output_sha256=outputs,source_sha256={str(p):sha(p) for p in [D/'protocol.json',D/'common.py',D/'qualify.py',Q/'projection.py',Q/'category_projection.py',Q/'category_memory.py']},fit=0,physical=0,scope='All seen development. Offline perfect finite range on original sensor query footprint, no hole filling. Additional ideal-category arm is optimistic diagnostic, not online. Memory only positive risk replay, never original risk clearing. Fixed-height local proxy omits original texture/uncertainty filters and is optimistic, not exact original end-to-end consumer upper bound.'))
