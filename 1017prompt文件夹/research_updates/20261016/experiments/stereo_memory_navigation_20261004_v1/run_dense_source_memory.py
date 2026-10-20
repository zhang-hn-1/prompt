"""ONE archive-only causal consumer. No labels, future states, actions or inference."""
from pathlib import Path
import datetime,json,hashlib,time
import numpy as np
from stereo_geometry import restore_native
from dense_source_points import point_packet,records
from point_support_shadow import PointSupportMap,swept_indices,support_result
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
p=json.loads((H/'dense_source_memory_protocol.json').read_text())
for kind in ['source_sha256','input_sha256']:
 for f,d in p[kind].items():assert sha(f)==d,f
out=H/'dense_source_memory_runs';out.mkdir(exist_ok=False);masks=np.load(p['mask']);origin=np.array(p['origin']);shape=tuple(p['shape']);res=.1;rows=[];started=now();status=H/'dense_source_memory_status.json';status.write_text(json.dumps(dict(start_utc=started,status='running'),indent=2)+'\n')
for model in ('b0','adapter'):
 history=PointSupportMap(origin,shape,res);folder=out/model;folder.mkdir()
 for frame in range(0,321,5):
  pose=np.load(H/'dense_source_memory_knownposes'/f'frame_{frame:03d}.npz');assert set(pose.files)=={'left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s'}
  raw=np.load(H/'dense_adapter_v2_predictions'/model/f'frame_{frame:03d}.npz');t=float(pose['time_s']);pred=dict(depth_m=raw['depth_m'],confidence=raw['confidence'],disparity_native_px=restore_native(raw['disparity_px']),frame=frame,time_s=t)
  packet=point_packet(pred,{k:pose[k] for k in pose.files},masks['left'],masks['right']);pts=records(packet);pp=folder/f'packet_{frame:03d}.npz';np.savez_compressed(pp,**packet)
  current=PointSupportMap(origin,shape,res);current.update(pts,t,0.);history.update(pts,t,0.);disk=swept_indices([pose['root_position_world_m'][:2]],origin,res,.7);arrays={};row=dict(model=model,frame=frame,time_s=t,points=len(pts),packet_sha256=sha(pp),knownpose_layer=True,arms={})
  for arm,mapper in [('current',current),('history',history)]:
   s=mapper.snapshot(t);arrays.update({arm+'_'+k:s[k] for k in ['labels','provenance','floor_time','hazard_time']});counts={k:int((s['provenance']==i).sum()) for i,k in [(1,'current_floor'),(2,'historical_floor'),(3,'current_hazard'),(4,'historical_hazard')]}
   row['arms'][arm]=dict(records=len(s['records']),ignored=s['ignored_records'],conflicts=s['conflict_cells'],counts=counts,source_time_min=None if not s['records'] else min(r['query'][3] for r in s['records']),source_time_max=None if not s['records'] else max(r['query'][3] for r in s['records']),current_disk=support_result(s['labels'],disk),certified_free=False,action_authorized=False,model_imputed_regions=0)
  arrays['current_disk_indices']=disk;mp=folder/f'maps_{frame:03d}.npz';np.savez_compressed(mp,**arrays);row['maps_sha256']=sha(mp);rp=folder/f'frame_{frame:03d}.json';rp.write_text(json.dumps(row,indent=2)+'\n');rows.append(dict(model=model,frame=frame,time_s=t,path=str(rp),sha256=sha(rp),packet_path=str(pp),packet_sha256=sha(pp),maps_path=str(mp),maps_sha256=sha(mp)))
  if frame%50==0:print('MEM',model,frame,len(pts),row['arms']['history']['records'],row['arms']['history']['current_disk']['unknown_cells'],flush=True)
assert len(rows)==130
(out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),rows=rows,labels_not_loaded=True,future_trajectory_not_loaded=True,models_source_rejected=True,maps=260,scope='knownpose/archive causalmapdiagnostic ONLY'),indent=2)+'\n');status.write_text(json.dumps(dict(start_utc=started,end_utc=now(),status='finished',rows=130,maps=260),indent=2)+'\n')
