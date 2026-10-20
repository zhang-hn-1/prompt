"""Independent saved prediction replay; never train, generate predictions, or run physics."""
from common import *
import numpy as np
from category_projection import raster,point_packet
from category_memory import CategoryMemory
from projection import root_from_camera
from rig_adapter import Rig,RigMask
from dynamic_self_mask import joints_from_proprio
import importlib.util
spec=importlib.util.spec_from_file_location('oracle_independent',Q/'independent_closure.py');review=importlib.util.module_from_spec(spec);spec.loader.exec_module(review)

def main():
 statuses={}
 for stage in ['prepare','train_predict','score']:
  s=json.loads((D/(stage+'_status.json')).read_text());assert s['status']=='finished' and s['child_returncode']==0;statuses[stage]=s
 result=json.loads((D/'result.json').read_text());cov=json.loads((D/'coverage.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());fit=json.loads((D/'fit_started.json').read_text());ledger=json.loads((D/'ledger.json').read_text());trace=json.loads((D/'train_trace.json').read_text());frozen=json.loads((D/'prefit_review.json').read_text())
 assert fit['fit']==ledger['fit_used']==1 and ledger['physical_used']==0;assert [x['epoch'] for x in trace]==list(range(1,31));assert fit['coverage_sha256']==sha(D/'coverage.json');assert fit['protocol_sha256']==sha(D/'protocol.json')
 for n,h in frozen['source_sha256'].items():assert sha(D/n)==h
 verify_oracle()
 for k in ['input_sha256','output_sha256']:verify_hashes(cov[k])
 for k in ['outputs_sha256','inputs_sha256','source_sha256']:verify_hashes(seal[k])
 assert sha(D/'model_final.pt')==seal['model_sha256'];verify_hashes(result['output_sha256']);assert sha(D/'prediction_seal.json')==result['prediction_seal_sha256']
 sums=np.zeros(10);sq=np.zeros(10);count=0;cc=np.zeros(3,np.int64)
 for group in ['H','E0']:
  for p in sorted((D/'features'/group).glob('*.npz')):
   a=np.load(p)['x'].astype(np.float64).reshape(10,-1);sums+=a.sum(1);sq+=(a*a).sum(1);count+=a.shape[1];s=np.load(D/'supervision'/group/p.name);cc+=np.bincount(s['category'].ravel(),minlength=3)
 mean=sums/count;std=np.sqrt(np.maximum(sq/count-mean*mean,1e-6));w=1/np.sqrt(cc);w/=w.mean();assert np.array_equal(mean,cov['mean']) and np.allclose(std,cov['std'],rtol=0,atol=1e-12) and np.array_equal(w,cov['class_weights']);assert np.array_equal(cc,cov['train_class_pixels'])
 rows=[];cad=RigMask(Rig('forward',.4,.3,.05))
 for group,R in GROUPS.items():
  maps=np.load(D/f'{group}_maps.npz')['prediction'];target=np.load(Q/f'{group}_maps.npz')['target'];init=np.load(R/'maps/frame_000.npz');mem=CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),group!='H');eall=[];oall=[];frames=[];kept=total=obkept=obtotal=0;files=sorted((D/'predictions'/group).glob('*.npz'));assert len(files)==181
  for j,p in enumerate(files):
   i=int(p.stem.split('_')[1]);assert i==j*5;a=np.load(p);f=np.load(R/'frames'/p.name);n=np.load(R/'known_poses'/p.name);h=np.load(R/'history'/p.name);lab=np.load(Q/'labels'/group/p.name);assert float(a['time_s'])==float(f['time_s'])==float(n['time_s'])==float(lab['time_s']);assert a['category'].shape==a['depth_m'].shape==a['confidence'].shape==(2,480,640);assert set(np.unique(a['category']))<={0,1,2};assert np.isfinite(a['depth_m']).all() and (a['depth_m']>=.5).all() and (a['depth_m']<=3).all();assert not (a['category'][a['confidence']<.9]>0).any()
   nr,root=root_from_camera(n['left_camera_usd_world']);yaw=np.arctan2(nr[1,0],nr[0,0]);mask=cad.project(joints_from_proprio(h['proprio_history']));points=[];classes=[];fe=[]
   for ei,eye in enumerate(['left','right']):
    cl=a['category'][ei];z=a['depth_m'][ei];pw,pc,_=point_packet(z,cl,n[eye+'_camera_usd_world'],root,mask[eye]);points.append(pw);classes.append(pc);valid=lab[eye+'_eligible']&(lab[eye+'_category']>0);take=valid&(cl>0);ob=valid&(lab[eye+'_category']==2);obt=ob&(cl==2);err=np.abs(z[take]-f[eye+'_depth_m'][take]);oe=np.abs(z[obt]-f[eye+'_depth_m'][obt]);eall.append(err);oall.append(oe);fe.append(err);kept+=int(take.sum());total+=int(valid.sum());obkept+=int(obt.sum());obtotal+=int(ob.sum())
   points=np.concatenate(points);classes=np.concatenate(classes);assert np.array_equal(raster(points,classes,root,yaw),maps[j]);mem.update(points,classes,i,float(a['time_s']),n['left_camera_usd_world'][:3,3]);sf=i if i in [450,495,500] else 899 if i==895 else None
   if sf is not None:
    actual=np.load(D/f'{group}_memory_{sf:03d}.npz');exp=mem.snapshot(float(actual['time_s']))
    for k in ['labels','clock','origin','resolution','time_s']:assert np.array_equal(actual[k],exp[k]),(group,sf,k)
   err=np.concatenate(fe);frames.append(dict(frame=i,scored=len(err),p90_m=float(np.quantile(err,.9)) if len(err) else None))
  m=review.independent_metrics(target,maps);row=next(x for x in result['rows'] if x['group']==group)
  for k,v in m.items():assert row['metrics'][k]==v
  err=np.concatenate(eall);oe=np.concatenate(oall);quant=np.quantile(err,[.5,.9,.95]).tolist() if len(err) else [None]*3;oq=np.quantile(oe,[.5,.9,.95]).tolist() if len(oe) else [None]*3;assert quant==row['range_p50_p90_p95_m'] and oq==row['obstacle_range_p50_p90_p95_m'];assert frames==row['frames'];assert [kept,total,obkept,obtotal]==[row[x] for x in ['retained_pixels','label_pixels','retained_obstacle_pixels','obstacle_label_pixels']]
  gates=dict(local_map=m['pass_gate'],range_p90=quant[1] is not None and quant[1]<=.15,obstacle_range_p90=oq[1] is not None and oq[1]<=.15,per_frame_range=all(x['p90_m']<=.3 for x in frames if x['scored']>=1000),retained_label_pixels=kept/max(1,total)>=.7,retained_obstacle_pixels=obkept/max(1,obtotal)>=.8);assert gates==row['gates'];assert all(gates.values())==row['pass_gate'];rows.append(dict(group=group,frames=181,maps_exact=True,memory_exact=[450,495,500,899],retained_label_fraction=kept/max(1,total),retained_obstacle_fraction=obkept/max(1,obtotal),range_quantiles=quant,obstacle_range_quantiles=oq,gates=gates,pass_gate=all(gates.values())));print('independent learned replay',group,'verified',flush=True)
 assert result['source_qualified']==all(r['pass_gate'] for r in rows)
 save('independent_closure.json',dict(independent_review_pass=True,source_qualified=result['source_qualified'],frames_verified=905,rows=rows,train_only_mean_weights_counts_exact=True,train_only_std_tolerance=1e-12,fit=1,epochs=30,physical=0,statuses=statuses,provenance_verified=True,review_source_sha256=sha(D/'independent_closure.py'),limits=['All groups seen development.','Learned category/range are priors; no binocular measurement certification.','Local native root/camera gauge cancels translation/yaw; maps do not prove world registration.','Ground and unknown are never free; no actual new navigation.','Range errors are conditional on retained eligible labels with explicit retention gates; not full-image depth accuracy.']))
if __name__=='__main__':main()
