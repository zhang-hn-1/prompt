"""Native teacher raster adapter only; original sealed predictions and gates unchanged."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'paired_current_A2_run_v1';O=H/'continuous_point_model_consumer_historical_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((O/'prediction_seal.json').read_text());assert all(sha(p)==v for p,v in seal['outputs'].items());p=D/'offline_occupancy_label.npz';truth=np.load(p);xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);metrics=[]
for f in [20,25,30,35,40]:
 v=np.load(O/f'model_{f:03d}.npz')['possible_xy'];dist=cKDTree(v).query(xy)[0] if len(v) else np.full(len(xy),np.inf);n=int((dist<=np.sqrt(2)*.05+1e-9).sum());metrics.append(dict(frame=f,MODEL_xy_centres=len(v),physical_cells=len(xy),covered_cells=n,coverage=n/len(xy),necessary_gate=n/len(xy)>=.95))
r=dict(decision='PASS_HISTORICAL_NECESSARY_ONLY' if all(v['necessary_gate'] for v in metrics) else 'REJECT_UNIQUE_CONTINUOUS_POINT_MODEL_CONSUMER',metrics=metrics,native_teacher_resolution_m=float(truth['resolution']),MODEL_resolution_m=.1,tolerance_m=np.sqrt(2)*.05,historical_not_fresh=True,source_fullpoint_rejection_preserved=True,no_actor_or_free_or_formal_nav=True,v1_failed_adapter_preserved=True,predictions_unmodified=True,teacher_sha256=sha(p),code_sha256=sha(__file__));(O/'assessment_v2_native_truth.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
