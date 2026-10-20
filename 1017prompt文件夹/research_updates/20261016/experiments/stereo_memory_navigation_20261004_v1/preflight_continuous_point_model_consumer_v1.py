"""Candidate outputs sealed before labels; historical source-consumer necessaryscreen."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial import cKDTree
from paired_prior_memory_v1 import PairedPriorMemory
from partial_cue_source_v2 import inflate_xy
H=Path(__file__).resolve().parent;D=H/'paired_current_A2_run_v1';O=H/'continuous_point_model_consumer_historical_v1';O.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();m=PairedPriorMemory();inputs={};counts=[]
for f in range(0,41,5):
 packets=[]
 for kind in ['eligible','unknown']:
  p=D/'current_evidence'/f'{kind}_packet_{f:03d}.npz';z=np.load(p);v={k:z[k].copy() for k in z.files};v['input_sha256']={str(p):sha(p)};inputs[str(p)]=sha(p);packets.append(v)
 t=float(packets[0]['time_s']);m.update(*packets,t)
 if f<20:continue
 s=m.at(t);points=np.concatenate([r['world'] for r in s['provisional']['records']],axis=0);haz=(points[:,2]>.05)&(points[:,2]<=1.5);xy=inflate_xy(points[haz,:2],.2);np.savez_compressed(O/f'model_{f:03d}.npz',possible_xy=xy,time_s=t,frame=f,model_hypothesis=True,measured_geometry=False,certified_free=False,source_model_admitted=False);counts.append(dict(frame=f,historical_hazard_points=int(haz.sum()),MODEL_xy_centres=len(xy)))
(O/'prediction_seal.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in O.glob('model_*.npz')},producer_sha256=sha(__file__),protocol_sha256=sha(H/'continuous_point_model_consumer_protocol_v1.json')),indent=2)+'\n')
# Only now open offline occupancy for necessary whole-footprint coverage.
p=D/'offline_occupancy_label.npz';truth=np.load(p);xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);assert abs(float(truth['resolution'])-.1)<1e-8;metrics=[]
for r in counts:
 v=np.load(O/f"model_{r['frame']:03d}.npz")['possible_xy'];dist=cKDTree(v).query(xy)[0] if len(v) else np.full(len(xy),np.inf);n=int((dist<=np.sqrt(2)*.05+1e-9).sum());metrics.append(dict(**r,physical_cells=len(xy),covered_cells=n,coverage=n/len(xy),necessary_gate=n/len(xy)>=.95))
r=dict(decision='PASS_HISTORICAL_NECESSARY_ONLY' if all(v['necessary_gate'] for v in metrics) else 'REJECT_UNIQUE_CONTINUOUS_POINT_MODEL_CONSUMER',metrics=metrics,historical_not_fresh=True,source_fullpoint_rejection_preserved=True,no_actor_or_free_or_formal_nav=True,occupancy_teacher_read_only_after_prediction_seal=True,teacher_sha256=sha(p));(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
