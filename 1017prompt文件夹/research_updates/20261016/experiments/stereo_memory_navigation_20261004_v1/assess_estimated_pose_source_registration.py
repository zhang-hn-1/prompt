from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;D=H/'estimated_pose_source_registration';old=H/'cad_complete_baseline_current_run'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
protocol=json.loads((D/'protocol.json').read_text());c=json.loads((D/'prediction_completion.json').read_text())
for rows in [protocol['source_sha256'],c['outputs_sha256']]:
 for f,s in rows.items():assert sha(f)==s,f
rows=[];distances=[];same=True
for f in sorted(D.glob('source_*.npz')):
 a=np.load(f);b=np.load(old/'sensor_packets'/f.name);equal={k:bool(np.array_equal(a[k],b[k])) for k in ['query_pixel_id','partner_pixel_id','sigma_m','frame','time_s']};same= same and all(equal.values())
 if equal['query_pixel_id'] and equal['partner_pixel_id']:
  distance=np.linalg.norm(a['world']-b['world'],axis=1);distances.extend(distance.tolist());rows.append(dict(frame=int(a['frame']),count=len(distance),payload_equal=equal,point_displacement_p90_m=float(np.percentile(distance,90)) if len(distance) else None,point_displacement_peak_m=float(distance.max()) if len(distance) else None))
 else:rows.append(dict(frame=int(a['frame']),count=len(a['world']),payload_equal=equal))
out=dict(historical_not_fresh=True,not_online=True,sources=len(rows),all_payload_equal=same,endpoint_displacement_p90_m=float(np.percentile(distances,90)),endpoint_displacement_peak_m=float(max(distances)),endpoint_count=len(distances),pose_sigma_assumption_zero_not_certified=True,initial_gauge_privileged=True,depth_source_gate_unchanged_not_admitted=True,rows=rows)
(D/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='rows'},indent=2))
