from pathlib import Path
import json,numpy as np,hashlib
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;rows=[]
for name in ('standby_handoff_e0_run_v1','standby_handoff_e1_transfer_run_v1','sgbm_baseline35_e2_run_v1','sgbm_baseline35_e0_coverage_run_v1'):
 D=H/name;states=json.loads((D/'states.json').read_text());native=json.loads((D/'native_predictions.json').read_text());lookup={round(float(r['time_s']),6):r for r in native if r['available']};xy=[];angle=[]
 for s in states:
  n=lookup[round(float(s['time_s']),6)];xy.append(float(np.linalg.norm(np.array(n['position_world'])[:2]-np.array(s['position'])[:2])))
  R=Rotation.from_quat(np.array(s['quaternion'])[[1,2,3,0]]).as_matrix();angle.append(float(Rotation.from_matrix(R.T@np.array(n['rotation_world'])).magnitude()))
 rows.append(dict(run=name,captures=len(states),xy_final_m=xy[-1],xy_p90_m=float(np.percentile(xy,90)),orientation_final_rad=angle[-1],orientation_p90_rad=float(np.percentile(angle,90)),role='offline diagnostic of already-collected causal native outputs; not fresh validation or pose admission',input_sha256={str(D/f):hashlib.sha256((D/f).read_bytes()).hexdigest() for f in ('states.json','native_predictions.json','initial_gauge.json')}))
(H/'native_pose_available_baseline_diagnostics_v1.json').write_text(json.dumps(dict(rows=rows,old_pose_rejections_preserved=True,used_for_protocol_design=True,not_fresh_heldout=True,no_fit=True),indent=2)+'\n')
print(json.dumps(rows))
