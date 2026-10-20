"""Independent full fixed-query oracle replay; no training or physics."""
from common import *
import numpy as np,importlib.util
from projection import backproject,root_from_camera
from category_projection import raster
from category_memory import CategoryMemory
from scipy.spatial import cKDTree
spec=importlib.util.spec_from_file_location('independent_metrics',Q/'independent_closure.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
s=json.loads((D/'qualify_status.json').read_text());assert s['status']=='finished' and s['child_returncode']==0
r=json.loads((D/'result.json').read_text());review=json.loads((D/'preflight_review.json').read_text());assert review['admitted']
for n,h in review['source_sha256'].items():assert sha(D/n)==h
for k in ['input_sha256','output_sha256','source_sha256']:
 for p,h in r[k].items():assert sha(p)==h
rows=[]
for name,(R,cp) in GROUPS.items():
 row=next(x for x in r['rows'] if x['group']==name);actual=np.load(D/f'{name}_local_maps.npz');target=np.load(Q/f'{name}_maps.npz')['target'];assert np.array_equal(target,actual['target']);frames=[];pe=[];tot=np.zeros(3,np.int64);post=np.zeros(2,np.int64);packets={};counts={k:[] for k in ['range_height','range_category']};init=np.load(R/'maps/frame_000.npz');mem={k:CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),name!='H') for k in counts}
 for j,p in enumerate(sorted((R/'frames').glob('*.npz'))):
  i=int(p.stem.split('_')[1]);assert i==j*5;f=np.load(p);n=np.load(R/'known_poses'/p.name);pkt=np.load(R/'sensor_packets'/f'source_{i:03d}.npz');lab=np.load(Q/'labels'/name/p.name);t=float(f['time_s']);assert t==float(n['time_s'])==float(pkt['time_s'])==float(lab['time_s']);ids=pkt['query_pixel_id'].astype(int);assert len(ids)==len(np.unique(ids));z=f['left_depth_m'].ravel()[ids];valid=np.isfinite(z)&(z>=.5)&(z<=3);idx=ids[valid];cl=lab['left_category'].ravel()[idx];tot+=np.array([len(ids),valid.sum(),(cl>0).sum()]);nr,root=root_from_camera(n['left_camera_usd_world']);yaw=np.arctan2(nr[1,0],nr[0,0]);world=backproject(idx%640,idx//640,z[valid],n['left_camera_usd_world']);keep=np.linalg.norm(world-root,axis=1)>=.55;depth=z[valid][keep];idx=idx[keep];world=world[keep];cl=cl[keep];post+=np.array([len(world),(cl>0).sum()]);truth=backproject(idx%640,idx//640,depth,f['left_camera_usd_world']);pe.append(np.linalg.norm(world-truth,axis=1));classes={'range_height':np.where((world[:,2]>.05)&(world[:,2]<=1.5),2,0).astype(np.uint8),'range_category':cl}
  for k,c in classes.items():assert np.array_equal(raster(world,c,root,yaw),actual[k][j]);packets.setdefault(k,{})[i]=(world,c,t,n['left_camera_usd_world'][:3,3])
  frames.append(dict(frame=i,queries=len(ids),finite_range=int(valid.sum()),labelled=int((lab['left_category'].ravel()[ids[valid]]>0).sum()),nohit=int((~np.isfinite(z)).sum()),finite_outside_range=int((np.isfinite(z)&~valid).sum())))
 assert frames==row['frames'];assert tot.tolist()==[row['queries'],row['finite_correctable'],row['labelled']];assert np.allclose(np.quantile(np.concatenate(pe),[.5,.9,.95]),row['point_error_p50_p90_p95'])
 gate={}
 for k in counts:
  met=m.independent_metrics(target,actual[k])
  for a in ['precision','recall','ground_false_fraction','unsupported_fraction','localization_p90_m']:assert met[a]==row['metrics'][k][a]
  gate[k]=dict(precision=met['precision']>=.9,recall=met['recall']>=.8,ground_false=met['ground_false_fraction']<=.05,unsupported=met['unsupported_fraction']<=.1,localization=met['localization_p90_m'] is not None and met['localization_p90_m']<=.15)
 assert gate==row['gates']
 keys=iter(range(0,901,5));key=next(keys,None);admitted=[]
 for i in range(900):
  a=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(a['time_s'])
  while key is not None and packets['range_height'][key][2]<=t:
   for k,memory in mem.items():w,c,epoch,view=packets[k][key];memory.update(w,c,key,epoch,view)
   admitted.append(key);key=next(keys,None)
  for k,memory in mem.items():
   snapshot=memory.snapshot(t);counts[k].append(int((snapshot['labels']==1).sum()))
   if i in [450,495,500,899]:
    old=np.load(D/f'{name}_{k}_memory_{i:03d}.npz')
    for a in ['labels','clock','origin','resolution','time_s']:assert np.array_equal(snapshot[a],old[a]),(name,k,i,a)
 assert counts==row['map_counts'];assert admitted==list(range(0,900,5))
 occpath=Path(json.loads(cp.read_text())['occupancy']);assert str(occpath) in r['input_sha256'];occ=np.load(occpath);ref=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(ref)
 for k in counts:
  a=np.load(D/f'{name}_{k}_memory_899.npz');cells=np.argwhere(a['labels']==1);xy=a['origin']+cells*float(a['resolution']);bad=tree.query(xy)[0]>.15;age=float(a['time_s'])-a['clock'][tuple(cells.T)];mm=dict(risk=len(cells),far015=int(bad.sum()),older10s=int((age>10).sum()),old_far015=int((bad&(age>10)).sum()));assert mm==row['memory_metrics'][k]
 rows.append(dict(group=name,source_frames=181,local_arms=2,local_maps_exact=362,memory_snapshots_exact=8,map_clocks_exact=900,source_epoch900_not_consumed=True,pre_body_queries=int(tot[0]),pre_body_finite=int(tot[1]),pre_body_labelled=int(tot[2]),post_body_finite=int(post[0]),post_body_labelled=int(post[1]),gates=gate,qualified=all(gate['range_height'].values()),optimistic_qualified=all(gate['range_category'].values())));print('independent fixed query',name,'verified',flush=True)
assert r['qualified']==all(x['qualified'] for x in rows);assert r['optimistic_qualified']==all(x['optimistic_qualified'] for x in rows)
save('independent_closure.json',dict(independent_review_pass=True,qualified=r['qualified'],optimistic_qualified=r['optimistic_qualified'],rows=rows,full_source_frames=905,local_maps_replayed=1810,memory_snapshots_replayed=40,map_clocks_replayed=4500,provenance_verified=True,fit=0,physical=0,minor_correction_review='Added only direct occupancy path-content SHA to result inputs; author-updated preflight SHA is explicitly later correction, not earlier reviewed source identity.',scope='Offline perfect finite-range fixed left-query proxy, not exact original whole pipeline or theoretical bound. Primary height gate and ideal-category optimistic arm distinct. Counts pre-body and post-body reported separately; no missing query filling, free or clearing.',review_source_sha256=sha(D/'independent_closure.py')))
