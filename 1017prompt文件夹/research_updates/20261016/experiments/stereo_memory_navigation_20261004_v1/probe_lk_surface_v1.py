"""Six frozen causal packets; explicit diagnostic, target100 gate unreachable with old queries."""
from pathlib import Path
import numpy as np,json,hashlib
from lk_surface_v1 import track_pairs
from feature_tracks import extract,triangulate
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'lk_surface_development_v1';O.mkdir(exist_ok=False);K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
cad=TimedStaticBaseSelfMask();cache={};inputs={};rows=[];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def packet(i):
 if i not in cache:
  paths=[D/'source_inputs'/f'frame_{i:03d}.npz',D/'known_poses'/f'frame_{i:03d}.npz',D/'history'/f'frame_{i:03d}.npz']
  for path in paths:inputs[str(path)]=sha(path)
  rgb=np.load(paths[0]);pose=np.load(paths[1]);history=np.load(paths[2])['proprio_history'].copy();history[:,:9]=0;mask=cad.project(joints_from_proprio(history))['left'];xy,desc,size=extract(rgb['left_rgb'],mask)
  cache[i]=dict(frame=i,time_s=float(pose['time_s']),pose=pose['left_camera_usd_world'],rgb=rgb['left_rgb'],mask=mask,xy=xy)
 return cache[i]
def predict(q,sources):
 pool=[[] for _ in q['xy']];stages=[]
 for s in sources:
  if s['time_s']>q['time_s']:continue
  if s['time_s']>=q['time_s']:raise ValueError('only strictly earlier LEFT')
  pairs=track_pairs(q['rgb'],s['rgb'],q['xy'],s['mask']);geometries=[]
  for pair in pairs:
   i=pair['query_index'];r=triangulate(q['xy'][i],pair['source_xy'],q['pose'],s['pose'],K)
   geometries.append(dict(query_index=i,source_xy=pair['source_xy'],result=r))
   if r is not None and r['sigma_m']<=.1:r.update(query=[q['frame'],'left',i,q['time_s']],source=[s['frame'],'left',pair['source_xy'],s['time_s']]);pool[i].append(r)
  stages.append(dict(source_frame=s['frame'],geometries=geometries))
 accepted=[];conflicts=0
 for p in pool:
  if not p:continue
  if any(np.linalg.norm(np.array(a['world'])-b['world'])>.1 for a in p for b in p):conflicts+=1;continue
  accepted.append(max(p,key=lambda x:x['parallax_deg']))
 return dict(query_xy=q['xy'].tolist(),points=accepted,stages=stages,conflicting_queries=conflicts)
for f in [215,220,225,325,330,335]:
 q=packet(f);sources=[packet(i) for i in range(f-20,f,5)];a=predict(q,sources);assert a==predict(q,sources+[dict(time_s=q['time_s']+.1,FORBIDDEN_FUTURE=True)]);rows.append(dict(frame=f,prediction=a,future_payload_ignored=True))
p=O/'predictions.json';p.write_text(json.dumps(rows,indent=2)+'\n');(O/'prediction_seal.json').write_text(json.dumps(dict(output_sha256={str(p):sha(p)},input_sha256=inputs,source_sha256={str(x):sha(x) for x in [Path(__file__),H/'lk_surface_v1.py',H/'feature_tracks.py',H/'lk_surface_protocol_v1.json',H/'lk_surface_protocol_v2.json']},no_GT_or_future_inputs=True,development_only=True),indent=2)+'\n');print([(r['frame'],len(r['prediction']['points'])) for r in rows])
