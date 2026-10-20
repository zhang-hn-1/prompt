"""Historical identity audit only; no policy recall, fit, or action modification."""
from pathlib import Path
import hashlib,json
import numpy as np
H=Path(__file__).resolve().parent; D=H/'flat_ground_pose_D3_run_v1'; O=H/'persistent_context_D3_historical_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
records=json.loads((D/'policy_records.json').read_text()); identity=json.loads((D/'identity.json').read_text())
rows=[]; previous=None; groups=[]
for path in sorted((D/'local_plans').glob('frame_*.npz')):
 frame=int(path.stem.split('_')[-1]); a=np.load(path)
 digest=hashlib.sha256()
 for key in ['pose','direction','command']:
  v=a[key];digest.update(key.encode());digest.update(str(v.dtype).encode());digest.update(str(v.shape).encode());digest.update(v.tobytes())
 geom=digest.hexdigest();local=records[frame]['local']; decision=local['reference_decision']
 row=dict(frame=frame,logged_check_time_s=float(a['time_s']),geometry_sha256=geom,version=int(a['version']),created_this_call=local['local_plan_created'],installed_time_s=local['reference_installed_time_s'],decision=decision['decision'],decision_measurement_time_s=decision['measured_time_s'],npz_sha256=sha(path))
 assert row['version']==local['local_reference_version']
 if geom!=previous:
  groups.append(dict(first_frame=frame,last_frame=frame,geometry_sha256=geom,version=row['version'],installed_time_s=row['installed_time_s'],logged_check_time_s=row['logged_check_time_s']))
 else:groups[-1]['last_frame']=frame
 previous=geom;rows.append(row)
checks={}
for p in [H/'run_flat_ground_pose_D3_v1.py',H/'tracking_tube_planner.py',H/'guarded_reference.py',H/'typed_hypothesis_grid.py']:
 recorded=identity['source_sha256'].get(str(p)); current=sha(p)
 checks[str(p)]=dict(recorded_sha256=recorded,current_sha256=current,exact=recorded==current)
r=dict(scope='HISTORICAL_REFERENCE_IDENTITY_NOT_NEW_CONTROL_OR_NAV',plan_files=len(rows),geometry_runs=groups,selected_rows=[x for x in rows if x['frame'] in [60,100,200,300,400]],source_identity=checks,all_rows=rows,inputs={str(D/'policy_records.json'):sha(D/'policy_records.json'),str(D/'identity.json'):sha(D/'identity.json')},code_sha256=sha(__file__),no_policy_recall=True,no_future_reference_fill=True)
(O/'reference_identity_audit_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='all_rows'},indent=2))
