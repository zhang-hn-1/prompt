from pathlib import Path
import json,hashlib,datetime,numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;cad=RobotSelfMask();frames=[('prior_rgb_history_run',0),('prior_rgb_history_run',135),('rgb_complete_goal_current_run',200),('rgb_complete_goal_current_run',350)];O=H/'dynamic_mask_development';O.mkdir(exist_ok=False);sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();paths=[H/'dynamic_self_mask.py',H/'run_dynamic_mask_development.py',H.parent.parent/'docs/superpowers/specs/2026-10-07-dynamic-self-occlusion-design.md']
for run,i in frames:paths.extend([H/run/'history'/f'frame_{i:03d}.npz',H/run/'sensor_packets'/f'source_{i:03d}.npz'])
pre=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),inputs_sha256={str(f):sha(f) for f in paths},frames=frames,scope='four already-used development frames; nofreshadmission orphysics');(O/'protocol.json').write_text(json.dumps(pre,indent=2)+'\n');rows=[]
for run,i in frames:
 hist=np.load(H/run/'history'/f'frame_{i:03d}.npz');q=joints_from_proprio(hist['proprio_history']);m=cad.project(q);p=np.load(H/run/'sensor_packets'/f'source_{i:03d}.npz');query=p['query_pixel_id'];partner=p['partner_pixel_id'];keep=~m['left'].ravel()[query]&~m['right'].ravel()[partner];np.savez_compressed(O/f'{run}_{i:03d}.npz',**m,joints=q,source_keep=keep,time_s=float(p['time_s']));row=dict(run=run,frame=i,current_measured_joints=q.tolist(),mask_fraction={e:float(m[e].mean()) for e in m},extra_mask_fraction={e:float((m[e]&~cad.frozen[e]).mean()) for e in m},original_source_points=len(query),remaining_source_points=int(keep.sum()),noGT_or_future_mask_inputs=True)
 if run=='prior_rgb_history_run' and i==135:
  bad=json.loads((H/'rgb_fresh_blocking_cells_diagnosis_v2.json').read_text())['query_pixels'];row['twelve_development_selfsurface_points_masked']=int(m['left'].ravel()[bad].sum())
 rows.append(row)
for f,d in pre['inputs_sha256'].items():assert sha(f)==d,f
result=dict(rows=rows,scope='CADprojection development only; FK/rendereridentity/falseocclusion gates notyetvalidated',candidate_admitted=False,whole_research_complete=False,post_artifact_sha256={str(f):sha(f) for f in O.glob('*') if f.is_file()});(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows,indent=2))
