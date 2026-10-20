from pathlib import Path
import json,datetime,hashlib,numpy as np
from dense_source_points import records
from point_support_shadow import PointSupportMap,replay_prefix
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'dense_source_memory_protocol.json').read_text());out=H/'dense_source_memory_runs';manifest=json.loads((out/'manifest.json').read_text());a=json.loads((out/'assessment.json').read_text());checks=0
for key in ('source_sha256','input_sha256'):
 for f,d in p[key].items():assert sha(f)==d,f;checks+=1
assert p['utc']<manifest['saved_utc']<a['utc'];assert len(manifest['rows'])==130 and manifest['maps']==260 and manifest['labels_not_loaded'] and manifest['future_trajectory_not_loaded'];assert not a['consumer_supported']
for r in manifest['rows']:
 for k,d in [('path','sha256'),('packet_path','packet_sha256'),('maps_path','maps_sha256')]:assert sha(r[k])==r[d];checks+=1
origin=p['origin'];shape=p['shape'];causal_checks=0;rebuilt=0
class Trap(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('read future payload')
  return super().__getitem__(k)
for model in ('b0','adapter'):
 history=PointSupportMap(origin,shape,.1);packets=[]
 for frame in range(0,321,5):
  raw=np.load(out/model/f'packet_{frame:03d}.npz');packet={k:raw[k] for k in raw.files};packet['frame']=int(packet['frame']);packet['time_s']=float(packet['time_s']);pts=records(packet);t=float(packet['time_s']);packets.append(dict(points=pts,time_s=t,pose_sigma_m=0.));current=PointSupportMap(origin,shape,.1);current.update(pts,t,0.);history.update(pts,t,0.);saved=np.load(out/model/f'maps_{frame:03d}.npz')
  for arm,m in [('current',current),('history',history)]:
   snap=m.snapshot(t)
   for key in ['labels','provenance','floor_time','hazard_time']:assert np.array_equal(snap[key],saved[arm+'_'+key]),(model,frame,key)
   assert not snap['certified_free'] and not snap['source_model_admitted'] and not snap['model_imputed_regions'];rebuilt+=1
 for t in (0.,6.,16.):
  earlier=[q for q in packets if q['time_s']<=t];trapped=earlier+[Trap(time_s=q['time_s']) for q in packets if q['time_s']>t];before=replay_prefix(earlier,t,origin,shape,.1).snapshot(t);after=replay_prefix(trapped,t,origin,shape,.1).snapshot(t)
  for key in ['labels','provenance','floor_time','hazard_time']:assert np.array_equal(before[key],after[key])
  causal_checks+=1
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),passed=True,identity_checks=checks,maps_rebuilt=rebuilt,actual_future_payload_replacement_and_truncation_checks=causal_checks,source_and_consumer_rejected=True,online_runtime_not_validated=True,whole_research_complete=False)
(H/'dense_source_memory_author_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
