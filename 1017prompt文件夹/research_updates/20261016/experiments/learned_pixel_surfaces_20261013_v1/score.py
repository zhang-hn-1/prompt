"""Reads truth only after all905 learned outputs are sealed."""
from common import *
import numpy as np
from category_projection import point_packet,raster
from category_memory import CategoryMemory
from projection import root_from_camera
from rig_adapter import Rig,RigMask
from dynamic_self_mask import joints_from_proprio
from scipy.spatial import cKDTree
old=(W/'experiments/learned_visible_obstacles_20261013_v1/score.py').read_text();ns={}
exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+old[old.index('def metrics('):old.index('\nrows=[]')],ns);metrics=ns['metrics']
seal=json.loads((D/'prediction_seal.json').read_text());assert seal['pairs']==905
verify_oracle()
cov=json.loads((D/'coverage.json').read_text());assert cov['qualified'];verify_hashes(cov['input_sha256']);verify_hashes(cov['output_sha256'])
for key in ['outputs_sha256','inputs_sha256','source_sha256']:verify_hashes(seal[key])
assert sha(D/'model_final.pt')==seal['model_sha256']
cad=RigMask(Rig('forward',.4,.3,.05));rows=[];outputs={}
for group,R in GROUPS.items():
 predmaps=[];errors=[];oberrors=[];frames=[];kept=total=obkept=obtotal=0
 init=np.load(R/'maps/frame_000.npz');mem=CategoryMemory(init['origin'],init['labels'].shape,float(init['resolution']),group!='H')
 for j,p in enumerate(sorted((D/'predictions'/group).glob('*.npz'))):
  frame=int(p.stem.split('_')[1]);a=np.load(p);f=np.load(R/'frames'/p.name);n=np.load(R/'known_poses'/p.name);h=np.load(R/'history'/p.name);lab=np.load(Q/'labels'/group/p.name)
  assert float(a['time_s'])==float(n['time_s'])==float(f['time_s']);mask=cad.project(joints_from_proprio(h['proprio_history']))
  nr,root=root_from_camera(n['left_camera_usd_world']);yaw=np.arctan2(nr[1,0],nr[0,0]);cloud=[];classes=[];fe=[];fcount=0
  for ei,eye in enumerate(['left','right']):
   z=a['depth_m'][ei];cl=a['category'][ei];pw,pc,_=point_packet(z,cl,n[eye+'_camera_usd_world'],root,mask[eye]);cloud.append(pw);classes.append(pc)
   valid=lab[eye+'_eligible']&(lab[eye+'_category']>0);take=valid&(cl>0);ob=valid&(lab[eye+'_category']==2);obt=ob&(cl==2)
   e=np.abs(z[take]-f[eye+'_depth_m'][take]);errors.append(e);fe.append(e);oberrors.append(np.abs(z[obt]-f[eye+'_depth_m'][obt]));kept+=int(take.sum());total+=int(valid.sum());obkept+=int(obt.sum());obtotal+=int(ob.sum());fcount+=len(e)
  world=np.concatenate(cloud);category=np.concatenate(classes);predmaps.append(raster(world,category,root,yaw));mem.update(world,category,frame,float(a['time_s']),n['left_camera_usd_world'][:3,3])
  if frame in [450,495,500,895]:
   k=899 if frame==895 else frame;now=float(np.load(R/'maps/frame_899.npz')['time_s']) if k==899 else float(a['time_s']);out=D/f'{group}_memory_{k:03d}.npz';np.savez_compressed(out,**mem.snapshot(now));outputs[str(out)]=sha(out)
  vals=np.concatenate(fe);frames.append(dict(frame=frame,scored=fcount,p90_m=float(np.quantile(vals,.9)) if len(vals) else None))
  if j%90==0:print('score',group,j+1,flush=True)
 arr=np.stack(predmaps);out=D/f'{group}_maps.npz';np.savez_compressed(out,prediction=arr);outputs[str(out)]=sha(out);target=np.load(Q/f'{group}_maps.npz')['target'];m=metrics(target,arr)
 e=np.concatenate(errors);oe=np.concatenate(oberrors);quant=np.quantile(e,[.5,.9,.95]).tolist() if len(e) else [None]*3;oq=np.quantile(oe,[.5,.9,.95]).tolist() if len(oe) else [None]*3
 gates=dict(local_map=m['pass_gate'],range_p90=quant[1] is not None and quant[1]<=.15,obstacle_range_p90=oq[1] is not None and oq[1]<=.15,per_frame_range=all(f['p90_m']<=.3 for f in frames if f['scored']>=1000),retained_label_pixels=kept/max(1,total)>=.7,retained_obstacle_pixels=obkept/max(1,obtotal)>=.8)
 rows.append(dict(group=group,role='train' if group in ['H','E0'] else 'fit_excluded_seen_development',metrics=m,range_p50_p90_p95_m=quant,obstacle_range_p50_p90_p95_m=oq,retained_pixels=kept,label_pixels=total,retained_obstacle_pixels=obkept,obstacle_label_pixels=obtotal,frames=frames,gates=gates,pass_gate=all(gates.values())))
 print('RESULT',group,gates,quant,m,flush=True)
save('result.json',dict(rows=rows,source_qualified=all(r['pass_gate'] for r in rows),fit=1,physical=0,output_sha256=outputs,prediction_seal_sha256=sha(D/'prediction_seal.json'),scope='All known data are development. Prior inferred ranges/classes, not confirmed stereo. Ground/unknown never free. GT only labels/scoring. Local gauge cancellation not world registration proof.'))
