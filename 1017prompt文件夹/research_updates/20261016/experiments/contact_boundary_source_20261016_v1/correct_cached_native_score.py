"""One scoring correction over unchanged saved labels; no new data or label search."""
from pathlib import Path
import json,os,hashlib,time
import numpy as np
from scipy.spatial import cKDTree
from qualify_data import D,AX,NAMES,project,metric,st

start=time.monotonic();r=json.loads((D/'e0_qualification.json').read_text())
root=Path(r['data_root']);out=D/'e0_recovery_qualification'
seal=json.loads((root/'capture_seal.json').read_text())
local=[np.array(v['local_matrix']) for v in seal['initial_local_identity']]
F=seal['initial_local_identity'][0]['optics'][0]/18*640
distances=[];point_errors=[];totals=dict(prediction_cells=0,target_cells=0,correct=0,recovered=0)
category={m:{k:dict(target=0,recovered=0) for k in ['base_on_floor','floor_intersecting','suspended','unidentified']} for m in ['oracle','native']}
changed=0;maximum_projection_difference=0.
for row in r['rows']:
 a=np.load(root/'frames'/row['frame']);p=out/row['frame']
 with np.load(p) as old:s={k:old[k].copy() for k in old.files}
 origin=s['origin'];cell=lambda w:np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
 union=set()
 for j,eye in enumerate(NAMES):
  uv=s[eye+'_contact_uv'];N=a[eye+'_camera_usd_world_estimated'];B=N@np.linalg.inv(local[j])
  pred,k=project(uv,N,F,B[:3,3]);cells=cell(pred);s[eye+'_native_cells']=cells;union|=st(cells)
  # Independent homogeneous camera-ray calculation and body reassembly.
  assert np.max(abs(B@local[j]-N))<1e-12
  ray=np.c_[(uv-[319.5,239.5])/F,np.ones(len(uv))]
  homogeneous=np.c_[ray[:,0],-ray[:,1],-ray[:,2],np.zeros(len(ray))]
  direction=(N@homogeneous.T).T[:,:3]
  depth=np.divide(-N[2,3],direction[:,2],out=np.full(len(uv),np.inf),where=abs(direction[:,2])>1e-12)
  with np.errstate(invalid='ignore'):wp=N[:3,3]+direction*depth[:,None]
  independent=(depth>=.5)&(depth<=3)&np.isfinite(wp).all(1)&(np.linalg.norm(wp-B[:3,3],axis=1)>=.55)
  assert np.array_equal(independent,k)
  if len(pred):maximum_projection_difference=max(maximum_projection_difference,float(np.max(abs(wp[k]-pred))))
  assert np.allclose(wp[k],pred,atol=1e-12,rtol=0)
  T=a[eye+'_camera_usd_world_label'];z=a[eye+'_depth_label'][uv[:,1],uv[:,0]]
  truepoint=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@AX).T
  point_errors.extend(np.linalg.norm(pred[:,:2]-truepoint[k,:2],axis=1).tolist())
 original=st(s['native_union']);changed+=int(original!=union)
 s['native_union']=np.array(sorted(union),np.int64).reshape(-1,2)
 score,dist=metric(union,st(s['target_union']),origin);row['native']=score;distances.extend(dist.tolist())
 for key in totals:totals[key]+=score[key]
 for method in ['oracle','native']:
  prediction=s[method+'_union']*.1+origin;tree=cKDTree(prediction) if len(prediction) else None
  for key in category[method]:
   target=s[key+'_cells']*.1+origin
   recovered=int((tree.query(target)[0]<=.15).sum()) if tree is not None and len(target) else 0
   category[method][key]['target']+=len(target);category[method][key]['recovered']+=recovered
 tmp=p.with_suffix('.tmp')
 with tmp.open('wb') as f:np.savez_compressed(f,**s)
 os.replace(tmp,p)  # Original hard-linked outputs remain preserved in initial_native_root_score_error.
precision=totals['correct']/totals['prediction_cells'] if totals['prediction_cells'] else None
recall=totals['recovered']/totals['target_cells'] if totals['target_cells'] else None
dist=np.array(distances);p90=float(np.quantile(dist,.9)) if len(dist) and np.isfinite(dist).all() else None
r['aggregate']['native']=dict(**totals,precision=precision,recall=recall,position_p90_m=p90,
 contact_point_position_p90_m=float(np.quantile(point_errors,.9)) if point_errors else None)
r['gates']['native'].update(precision=precision is not None and precision>=.95,
 recall=recall is not None and recall>=.8,position=p90 is not None and p90<=.15,
 every_frame_recall=all(row['native']['target_cells']==0 or (row['native']['recall'] is not None and row['native']['recall']>=.8) for row in r['rows']),
 every_frame_precision=all(row['native']['prediction_cells']==0 or row['native']['precision']>=.95 for row in r['rows']))
for method in ['oracle','native']:
 r['gates'][method]['every_frame_position']=all(row[method]['prediction_cells']==0 or (row[method]['position_p90_m'] is not None and row[method]['position_p90_m']<=.3) for row in r['rows'])
r['all_required_gates_pass']=all(all(g.values()) for g in r['gates'].values())
r['decision']='PASS_NECESSARY_DATA_AND_CONSUMER_GATE_ONLY' if r['all_required_gates_pass'] else 'REJECT_BEFORE_FIT'
for method in category:
 for key,a in category[method].items():a['recall']=a['recovered']/a['target'] if a['target'] else None
r['visible_category_coverage']=category
r['scoring_correction']=dict(issue='Native root exclusion formerly used USD camera rotation; now B=N@inverse(local). Added originally registered per-frame position gate.',
 unchanged='All physical data, contact UV labels, oracle outputs, target domains, thresholds and category memberships.',
 original_source_sha256=r['source_sha256'],corrected_driver_sha256=hashlib.sha256((D/'qualify_data.py').read_bytes()).hexdigest(),
 native_reprojection_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 frames_changed_native_cells=changed,maximum_independent_homogeneous_difference_m=maximum_projection_difference,
 wall_s=time.monotonic()-start)
(D/'e0_qualification.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(r['decision'],r['aggregate'],r['gates'],r['visible_category_coverage'],r['scoring_correction'],flush=True)
