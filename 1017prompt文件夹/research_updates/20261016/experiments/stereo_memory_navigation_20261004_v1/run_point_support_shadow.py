"""Historical causal replay: no truth access, actions, simulator or model inference."""
from pathlib import Path
import json,datetime,hashlib,time
import numpy as np
from point_support_shadow import PointSupportMap,swept_indices,support_result
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'point_support_protocol.json').read_text())
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 out=H/'point_support_runs';out.mkdir(exist_ok=False)
 ref=np.load(p['reference_file'])['world_xy'];origin=np.array(p['grid']['origin']);shape=tuple(p['grid']['shape']);res=p['grid']['resolution']
 phases=dict(start=ref[:1],turn=ref[:301],stop=ref[300:],whole=ref)
 cells={k:swept_indices(v,origin,res,.7) for k,v in phases.items()}
 np.savez_compressed(out/'reference_and_sweep.npz',world_xy=ref,**{k+'_cells':v for k,v in cells.items()})
 history=PointSupportMap(origin,shape,res);rows=[]
 for i in range(181):
  raw=json.loads((H/'active_feature_runs'/f'prediction_{i:03d}.json').read_text());t=raw['time_s'];points=raw['causal']['points']
  packet=dict(frame=i,time_s=t,points=[{k:pt[k] for k in ['world','sigma_m','query','source']} for pt in points],pose_sigma_m=raw['pose_sigma_m'])
  packetfile=out/f'packet_{i:03d}.json';packetfile.write_text(json.dumps(packet,indent=2)+'\n')
  current=PointSupportMap(origin,shape,res);current.update(packet['points'],t,packet['pose_sigma_m']);history.update(packet['points'],t,packet['pose_sigma_m'])
  arrays={};frame=dict(frame=i,time_s=t,arms={},same_causal_query_points_both_arms=True,source_gate_passed=False,labels_not_loaded=True)
  for arm,m in [('current',current),('history',history)]:
   start=time.perf_counter();s=m.snapshot(t)
   for key in ['labels','provenance','floor_time','hazard_time']:arrays[arm+'_'+key]=s[key]
   support={k:support_result(s['labels'],c) for k,c in cells.items()}
   frame['arms'][arm]=dict(records=s['records'],ignored_records=s['ignored_records'],conflict_cells=s['conflict_cells'],current_floor_cells=int((s['provenance']==1).sum()),historical_floor_cells=int((s['provenance']==2).sum()),current_hazard_cells=int((s['provenance']==3).sum()),historical_hazard_cells=int((s['provenance']==4).sum()),unknown_cells=int((s['labels']<0).sum()),source_from_past_records=sum(pt['source'][3]<pt['query'][3] for pt in s['records']),support=support,source_model_admitted=False,certified_free=False,full_3d_certified=False,model_imputed_regions=0,action_authorized=False,seconds=time.perf_counter()-start)
  mapsfile=out/f'maps_{i:03d}.npz';np.savez_compressed(mapsfile,**arrays)
  recordfile=out/f'frame_{i:03d}.json';recordfile.write_text(json.dumps(frame,indent=2)+'\n')
  rows.append(dict(frame=i,time_s=t,packet_sha256=sha(packetfile),maps_sha256=sha(mapsfile),record_sha256=sha(recordfile)))
  if i%30==0:print('SUPPORT',i,[(a,frame['arms'][a]['support']['whole']['unknown_cells']) for a in ['current','history']],flush=True)
 manifest=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,labels_not_loaded=True,source_sha256=p['source_sha256'],reference_and_sweep_sha256=sha(out/'reference_and_sweep.npz'),maps=362,scope='same frozen causal query points, current-frame-only versus10s endpoint map; historical diagnostic; no3D/space interpolation/truth/action/admission')
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('POINT_SUPPORT_COMPLETE',manifest['saved_utc'],flush=True)
if __name__=='__main__':main()
