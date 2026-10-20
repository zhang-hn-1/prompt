"""Complete current-RGB prediction stream; no renderer/GT reads."""
from common import *
from adapter import Model
from semantic_projection import project_semantic
from category_memory import CategoryMemory
import numpy as np,time,torch
assert json.loads((D/'oracle_qualification.json').read_text())['pass_gate'];assert (D/'semantic_identity.json').exists();out=D/'predictions';out.mkdir(exist_ok=False);model=Model();torch.cuda.reset_peak_memory_stats();inputs={};outputs={};rows=[];start=time.monotonic()
for name,(R,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());maskpath=Path(cfg['static_masks']);masks=dict(np.load(maskpath));init=np.load(R/'maps/frame_000.npz');origin=init['origin'];memory=CategoryMemory(origin,init['labels'].shape,.1,name!='H');gm=out/name;gm.mkdir();paths=sorted((R/'source_inputs').glob('*.npz'));assert len(paths)==181;timings=[];counts=[];mr=[]
 for f in [cp,maskpath,R/'maps/frame_000.npz']:inputs[str(f)]=sha(f)
 for j,p in enumerate(paths):
  a=np.load(p);pp=R/'known_poses'/p.name;pose=dict(np.load(pp));i=int(p.stem.split('_')[1]);t=float(a['time_s']);assert t==float(pose['time_s']);torch.cuda.synchronize();begin=time.monotonic();pair=model.pair(a['left_rgb'],a['right_rgb']);torch.cuda.synchronize();infer=time.monotonic()-begin
  zs=[x['depth_m'] for x in pair];classes=[x['category'] for x in pair];grids,conflicts=project_semantic(zs,classes,pose,masks,origin);elapsed=time.monotonic()-begin;timings.append(dict(frame=i,inference_sync_s=infer,pair_projection_wall_s=elapsed));c=grids[2];world=np.column_stack([origin+c*.1,np.full(len(c),.5)]);memory.update(world,np.full(len(c),2),i,t,pose['left_camera_usd_world'][:3,3]);unused_snapshot=memory.snapshot(t)
  pf=gm/p.name;arr={e+'_'+k:v for e,b in zip(['left','right'],pair) for k,v in b.items()};np.savez_compressed(pf,**arr,left_cells=grids[0],right_cells=grids[1],fused_cells=grids[2],origin=origin,time_s=t,frame=i,inference_sync_s=infer,pair_projection_wall_s=elapsed);inputs[str(p)]=sha(p);inputs[str(pp)]=sha(pp);outputs[str(pf)]=sha(pf);counts.append(dict(frame=i,conflicts=conflicts,cells=[len(c) for c in grids]))
  for ii in range(i,min(i+5,900)):
   mp=R/'maps'/f'frame_{ii:03d}.npz';qt=float(np.load(mp)['time_s']);inputs[str(mp)]=sha(mp);snapshot=memory.snapshot(qt);mr.append(dict(frame=ii,time_s=qt,risk=int((snapshot['labels']==1).sum())))
   if ii in [450,495,500,899]:
    mf=gm/f'memory_{ii:03d}.npz';np.savez_compressed(mf,**snapshot);outputs[str(mf)]=sha(mf)
  if j%30==0:print(name,j+1,'/181',round(time.monotonic()-start),flush=True)
 rows.append(dict(group=name,timings=timings,counts=counts,memory_counts=mr));print(name,'complete',flush=True)
 save('generation_progress.json',dict(completed=[r['group'] for r in rows],elapsed_s=time.monotonic()-start))
sourcefiles=[D/n for n in ['protocol.json','common.py','adapter.py','projection.py','semantic_projection.py','category_memory.py','generate.py','semantic_identity.json']]+list((D/'semantic_weights').glob('*'))+[D.parent/'dual_monocular_bev_20261015_v1/weights/metric_hypersim_vits.pth'];sourcefiles+=list((D.parent/'dual_monocular_bev_20261015_v1/vendor/Depth-Anything-V2/metric_depth/depth_anything_v2').rglob('*.py'))
save('prediction_seal.json',dict(inputs=inputs,outputs=outputs,sources={str(p):sha(p) for p in sourcefiles},rows=rows,truth_read=False,fit=0,physical=0,elapsed_s=time.monotonic()-start,peak_gpu_bytes=torch.cuda.max_memory_allocated()))
print('ALL 1267 PAIRS SEALED BEFORE GT',flush=True)
