"""Generate current-pair outputs only. This process has no GT reads."""
from common import *
import numpy as np,time,torch
from census import CensusStereo
from rig_adapter import Rig,RigMask,paired
from height_interval import HeightIntervalSurfaceMemory
from stereo_geometry import restore_native
from tentative_point_memory import TentativePointMemory
from viewpoint_confirmation import ViewpointConfirmationGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
original_savez=np.savez_compressed
def preserved_savez(path,**arrays):
 if Path(path).exists():
  old=np.load(path);assert set(old.files)==set(arrays),path
  for k,v in arrays.items():assert np.array_equal(old[k],np.asarray(v),equal_nan=True),(path,k)
 else:original_savez(path,**arrays)
np.savez_compressed=preserved_savez
recovery=json.loads((D/'recovery_registration.json').read_text())
assert sha(D/'census.py')==recovery['census_sha'] and sha(D/'protocol.json')==recovery['protocol_sha']
for p,digest in recovery['preserved_prefix'].items():assert sha(p)==digest,p
out=D/'predictions';out.mkdir(exist_ok=True);inputs={};outputs={};rows=[];model=CensusStereo('cuda');cad=RigMask(Rig('forward',.4,.3));start=time.monotonic()
for name,(R,configpath) in GROUPS.items():
 term=json.loads((R/'termination.json').read_text());assert term['termination'] in ['timeout90','collision','success','arrival','goal_reached'];paths=sorted((R/'source_inputs').glob('*.npz'));assert [int(p.stem.split('_')[1]) for p in paths]==list(range(0,901,5))
 cfg=json.loads(configpath.read_text());masks=dict(np.load(cfg['static_masks']));inputs[str(configpath)]=sha(configpath);inputs[cfg['static_masks']]=sha(cfg['static_masks']);inputs[str(R/'termination.json')]=sha(R/'termination.json')
 gm=out/name;gm.mkdir(exist_ok=True);surface=HeightIntervalSurfaceMemory();old_surface=HeightIntervalSurfaceMemory();memory=TentativePointMemory();views={};surfaces={};gstart=time.monotonic();projection_exact=0
 for j,p in enumerate(paths):
  a=dict(np.load(p));i=int(p.stem.split('_')[1]);t=float(a['time_s']);assert abs(t-i*.1)<1e-5;posepath=R/'known_poses'/p.name;histpath=R/'history'/p.name;pose=dict(np.load(posepath));hist=np.load(histpath)['proprio_history'].copy();hist[:,:9]=0.;assert float(pose['time_s'])==t
  for f in [p,posepath,histpath]:inputs[str(f)]=sha(f)
  begin=time.monotonic();predfile=gm/p.name
  if predfile.exists():
   pred=dict(np.load(predfile));assert int(pred.pop('frame'))==i and float(pred.pop('time_s'))==t;pred.pop('compute_wall_s')
  else:
   pred=model.predict(a['left_rgb'],a['right_rgb'],masks['left'],masks['right']);np.savez_compressed(predfile,**pred,frame=i,time_s=t,compute_wall_s=time.monotonic()-begin)
  pkt=dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t)
  _,q,_=paired(pkt,pose,hist,cad,t);pf=gm/f'packet_{i:03d}.npz';np.savez_compressed(pf,**q)
  hashes={str(f):sha(f) for f in [p,posepath,histpath,predfile,pf]};s=surface.update(q,a['left_rgb'],a['right_rgb'],t,hashes,pkt['disparity_native_px']);sf=gm/f'surface_{i:03d}.npz';np.savez_compressed(sf,**{k:v for k,v in s.items() if k!='input_hashes'})
  surfaces[i]=dict(frame=i,time_s=t,world=s['world'],query_pixel_id=s['query_pixel_id'],partner_pixel_id=s['partner_pixel_id'],input_sha256=hashes);views[i]=dict(time_s=t,position_world=pose['left_camera_usd_world'][:3,3])
  # Original source arithmetic is a separate identity check, never input to new matcher.
  op=R/'source_predictions'/p.name;old=dict(np.load(op));bpkt=dict(depth_m=old['depth_m'],confidence=old['confidence'],disparity_native_px=restore_native(old['disparity_px']),frame=i,time_s=t)
  _,bq,_=paired(bpkt,pose,hist,cad,t);savedq=np.load(R/'sensor_packets'/f'source_{i:03d}.npz')
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:assert np.array_equal(bq[k],savedq[k]),(name,i,'packet',k)
  bs=old_surface.update(bq,a['left_rgb'],a['right_rgb'],t,hashes,bpkt['disparity_native_px']);saveds=np.load(R/'partial_observations'/p.name)
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:assert np.array_equal(bs[k],saveds[k]),(name,i,'surface',k)
  projection_exact+=1
  for f in [op,R/'sensor_packets'/f'source_{i:03d}.npz',R/'partial_observations'/p.name]:inputs[str(f)]=sha(f)
  for f in [predfile,pf,sf]:outputs[str(f)]=sha(f)
  if j%20==0:print(name,j+1,'/181 generation, elapsed',round(time.monotonic()-start),flush=True)
 # Replay unchanged risk consumer for complete recorded trajectory. Neither ground truth nor planning queried.
 origin=np.load(R/'maps/frame_000.npz')['origin'];Grid=RepeatedExposureMemoryGrid if name=='H' else ViewpointConfirmationGrid
 grids=[Grid(origin,10,static_scene_assumption=True) for _ in range(2)];npath=R/'native_predictions.json';poses={round(v['time_s'],7):v['position_world'] for v in json.loads(npath.read_text()) if v.get('available')};inputs[str(npath)]=sha(npath)
 framekeys=iter(sorted(surfaces));nextkey=next(framekeys,None);counts=[]
 for i in range(900):
  mp=R/'maps'/f'frame_{i:03d}.npz';a=np.load(mp);t=float(a['time_s']);root=np.asarray(poses[round(t,7)]);rp=R/'raw_point_memory_snapshots'/f'frame_{i:03d}.json';rs=json.loads(rp.read_text())['records']
  while nextkey is not None and surfaces[nextkey]['time_s']<=t:
   memory.update(surfaces[nextkey],t);nextkey=next(framekeys,None)
  for g,records in zip(grids,[rs,memory.at(t)['records']]):
   if name=='H':g.consume_records(records,t,root)
   else:g.consume_records(records,t,root,views)
  assert np.array_equal(grids[0].labels,a['labels']),(name,i,'original map not exact')
  assert np.array_equal(grids[0].hazard_acquisition_time,a['hazard_acquisition_time']),(name,i,'original map clock not exact')
  inputs[str(mp)]=sha(mp);inputs[str(rp)]=sha(rp)
  counts.append(dict(frame=i,time_s=t,baseline=int((grids[0].labels==1).sum()),candidate=int((grids[1].labels==1).sum())))
  if i in [450,495,500,899]:
   for label,g in zip(['baseline','candidate'],grids):
    f=gm/f'map_{label}_{i:03d}.npz';np.savez_compressed(f,labels=g.labels,clock=g.hazard_acquisition_time,origin=g.origin,resolution=g.resolution,time_s=t);outputs[str(f)]=sha(f)
 row=dict(group=name,epochs=181,original_packet_and_surface_exact=projection_exact,original_maps_exact=900,termination=term,elapsed_s=time.monotonic()-gstart,map_counts=counts);rows.append(row);print(name,'complete, original 181 sources/900 maps exact',flush=True)
 save('generation_progress.json',dict(completed=[r['group'] for r in rows],elapsed_s=time.monotonic()-start))
sourcefiles=[D/n for n in ['protocol.json','census.py','common.py','generate.py']]+[H/n for n in ['dense_source_points.py','paired_texture_surface.py','generic_height_surface.py','partial_surface_memory.py','tentative_point_memory.py','repeated_exposure_memory_v1.py','partial_point_risk_grid.py','dynamic_self_mask.py']]+[V/'rig_adapter.py',V/'viewpoint_confirmation.py',W/'experiments/risk_height_navigation_20261012_v1/height_interval.py']
save('prediction_seal.json',dict(inputs=inputs,outputs=outputs,sources={str(p):sha(p) for p in sourcefiles},rows=rows,truth_read=False,fit=0,physical=0,elapsed_s=time.monotonic()-start))
print('ALL 905 PREDICTIONS, 4500 REPLAY COUNTS AND SELECTED MAP ARRAYS SEALED BEFORE GT',flush=True)
