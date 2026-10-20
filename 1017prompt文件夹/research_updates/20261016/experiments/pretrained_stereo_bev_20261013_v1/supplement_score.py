"""Offline sealed-output evaluation; never changes predictions/consumer."""
from common import *
import numpy as np
from scipy.spatial import cKDTree
from tentative_point_memory import TentativePointMemory
from viewpoint_confirmation import ViewpointConfirmationGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
assert (D/'prediction_seal.json').exists()
results={}
for name,(R,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());occ=np.load(cfg['occupancy']);tree=cKDTree(occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']));gm=D/'predictions'/name
 poses={round(v['time_s'],7):v['position_world'] for v in json.loads((R/'native_predictions.json').read_text()) if v.get('available')};origin=np.load(R/'maps/frame_000.npz')['origin'];Grid=RepeatedExposureMemoryGrid if name=='H' else ViewpointConfirmationGrid;g=Grid(origin,10,static_scene_assumption=True);memory=TentativePointMemory();views={};surfs={};timings=[]
 for p in sorted((R/'source_inputs').glob('*.npz')):
  i=int(p.stem.split('_')[1]);a=dict(np.load(gm/f'surface_{i:03d}.npz'));pose=np.load(R/'known_poses'/p.name);surfs[i]=dict(frame=i,time_s=float(np.load(gm/p.name)['time_s']),world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256={str(gm/f'surface_{i:03d}.npz'):sha(gm/f'surface_{i:03d}.npz')});views[i]=dict(time_s=float(np.load(gm/p.name)['time_s']),position_world=pose['left_camera_usd_world'][:3,3]);timings.append(float(np.load(gm/p.name)['compute_wall_s']))
 keys=iter(sorted(surfs));n=next(keys,None);rows=[]
 for i in range(900):
  old=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(old['time_s']);root=np.asarray(poses[round(t,7)])
  while n is not None and surfs[n]['time_s']<=t:memory.update(surfs[n],t);n=next(keys,None)
  if name=='H':g.consume_records(memory.at(t)['records'],t,root)
  else:g.consume_records(memory.at(t)['records'],t,root,views)
  arms=[]
  for labels,clock in [(old['labels'],old['hazard_acquisition_time']),(g.labels,g.hazard_acquisition_time)]:
   c=np.argwhere(labels==1);xy=origin+c*.1;dist=tree.query(xy)[0] if len(c) else np.empty(0);age=t-clock[tuple(c.T)];clear=float(np.linalg.norm(xy-root[:2],axis=1).min()) if len(c) else None
   arms.append(dict(risk=len(c),wrong=int((dist>.15).sum()),old=int((age>10).sum()),old_wrong=int(((dist>.15)&(age>10)).sum()),root_to_risk_min_m=clear,root_blocked=clear is not None and clear<.8707106781186548))
  if i in [450,495,500,899]:
   s=np.load(gm/f'map_candidate_{i:03d}.npz');assert np.array_equal(g.labels,s['labels']);assert np.array_equal(g.hazard_acquisition_time,s['clock'])
  rows.append(dict(frame=i,time_s=t,arms=arms))
 results[name]=dict(rows=rows,paired_wall_seconds_p50_p90_max=np.quantile(timings,[.5,.9,1]).tolist(),root_blocked_ticks=[sum(r['arms'][j]['root_blocked'] for r in rows) for j in range(2)],final=rows[-1])
 print(name,results[name]['root_blocked_ticks'],results[name]['paired_wall_seconds_p50_p90_max'],flush=True)
R,_=GROUPS['C65'];block=[]
for i in [220,225]:
 p=f'frame_{i:03d}.npz';a=np.load(R/'partial_observations'/p);origin=np.load(R/'maps/frame_000.npz')['origin'];c=np.floor((a['world'][:,:2]-origin)/.1+.5).astype(int);xy=origin+c*.1;q=a['query_pixel_id'][np.linalg.norm(xy-[-1.1,-.1],axis=1)<1e-5].astype(int);f=np.load(R/'frames'/p);gt=f['left_depth_m'].reshape(-1)[q];pred=np.load(D/'predictions/C65'/p);z=pred['depth_m'].reshape(-1)[q];valid=pred['confidence'].reshape(-1)[q].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)
 block.append(dict(frame=i,original_queries=q.tolist(),GT_m=gt.tolist(),pretrained_depth_m=[float(v) if np.isfinite(v) else None for v in z],accepted=valid.tolist(),error_m=[float(v) if np.isfinite(v) else None for v in np.abs(z-gt)],finite_GT=int(np.isfinite(gt).sum())))
save('supplement_result.json',dict(groups=results,original_16_blocker_sources=block,scope='Complete 6300 same-trajectory map times, original consumer unchanged; GT bodyband reference .15m only scoring. Root clearance is risk-cell centre distance, not proof of physical collision or route feasibility. Paired wall includes synchronized GPU CPU transfer, not pure kernel time; all groups development.'))
