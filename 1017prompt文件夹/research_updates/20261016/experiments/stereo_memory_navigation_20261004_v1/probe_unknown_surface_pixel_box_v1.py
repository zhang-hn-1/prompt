from pathlib import Path
import numpy as np,json,hashlib,datetime,time
from pixel_box_polytope_v1 import envelope
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';L=H/'lk_surface_development_v1';O=H/'unknown_surface_pixel_box_development_v1';O.mkdir(exist_ok=False);sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=H/'unknown_surface_pixel_box_protocol_v1.json';protocol=json.loads(p.read_text());assert all(sha(f)==s for f,s in protocol['source_sha256'].items());rows=json.loads((L/'predictions.json').read_text());assert [r['frame'] for r in rows]==protocol['frames'];K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]]);inputs={};results=[];start=time.perf_counter()
for row in rows:
 f=row['frame'];cp=D/'known_poses'/f'frame_{f:03d}.npz';inputs[str(cp)]=sha(cp);c=dict(np.load(cp));q=np.asarray(row['prediction']['query_xy']);out=[]
 for stage in row['prediction']['stages']:
  past=stage['source_frame'];pp=D/'known_poses'/f'frame_{past:03d}.npz';inputs[str(pp)]=sha(pp);s=dict(np.load(pp));assert past<f and abs(float(c['time_s'])-f*.1)<1e-5 and abs(float(s['time_s'])-past*.1)<1e-5
  for r in stage['geometries']:
   if r['result'] is None:continue
   j=r['query_index'];a=envelope(q[j],r['source_xy'],c['left_camera_usd_world'],s['left_camera_usd_world'],K,float(c['time_s']),float(s['time_s']));a.update(query_index=j,query_xy=q[j].tolist(),query_frame=f,query_time_s=float(c['time_s']),source_frame=past,source_time_s=float(s['time_s']),source_xy=r['source_xy']);out.append(a)
 results.append(dict(frame=f,geometries=out))
pred=O/'predictions.json';pred.write_text(json.dumps(results,indent=2)+'\n');(O/'prediction_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),wall_s=time.perf_counter()-start,input_sha256=inputs,protocol_sha256=sha(p),output_sha256={str(pred):sha(pred)},code_sha256={str(H/n):sha(H/n) for n in ['pixel_box_polytope_v1.py','probe_unknown_surface_pixel_box_v1.py']},source_GT_pose_diagnostic_only=True,no_depth_labels_or_future_inputs=True),indent=2)+'\n');print([(r['frame'],len(r['geometries']),{s:sum(g['status']==s for g in r['geometries']) for s in ['bounded','empty','unbounded','solver_failure']}) for r in results])
