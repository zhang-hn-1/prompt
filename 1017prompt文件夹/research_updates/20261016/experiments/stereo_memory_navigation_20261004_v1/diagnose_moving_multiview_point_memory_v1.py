"""Five frozen historical causal prefixes, no new controller candidate or grade."""
from pathlib import Path
import hashlib,json,numpy as np
from scipy.spatial import cKDTree
from paired_current_evidence_v1 import qualify_packet
from paired_prior_memory_v1 import PairedPriorMemory
from partial_cue_source_v2 import inflate_xy
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'moving_multiview_point_memory_diagnostic_v1';O.mkdir();m=PairedPriorMemory();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};rows=[]
for f in range(0,401,5):
 fs=[D/'sensor_packets'/f'source_{f:03d}.npz',D/'online_inputs'/f'frame_{f:03d}.npz',D/'masks'/f'frame_{f:03d}.npz'];inputs.update({str(p):sha(p) for p in fs});p=dict(np.load(fs[0]));rgb=np.load(fs[1]);mask=np.load(fs[2]);t=float(p['time_s']);q=qualify_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],float(rgb['time_s']),t,f);a=q['eligible_packet'];b=q['unknown_packet'];a['input_sha256']=b['input_sha256']={str(v):sha(v) for v in fs};m.update(a,b,t)
 if f not in [20,100,200,300,400]:continue
 s=m.at(t);w=np.concatenate([r['world'] for r in s['provisional']['records']],axis=0);haz=(w[:,2]>.05)&(w[:,2]<=1.5);xy=inflate_xy(w[haz,:2],.2);np.savez_compressed(O/f'model_{f:03d}.npz',possible_xy=xy,time_s=t,frame=f,model_hypothesis=True,measured_geometry=False,certified_free=False);rows.append(dict(frame=f,current_supported=len(a['world']),history_supported=len(w)-len(a['world']),hazard_support=int(haz.sum()),support_epochs=len(s['provisional']['records']),MODEL_centres=len(xy)));print('PREFIX',f,len(xy),flush=True)
(O/'prediction_seal.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in O.glob('model_*.npz')},code_sha256=sha(__file__)),indent=2)+'\n')
g=np.load(D/'offline_occupancy_label.npz');truth=g['origin']+np.argwhere(g['labels']>0)*float(g['resolution'])
for r in rows:
 xy=np.load(O/f"model_{r['frame']:03d}.npz")['possible_xy'];dist=cKDTree(xy).query(truth)[0] if len(xy) else np.full(len(truth),np.inf);r.update(physical_cells=len(truth),covered_cells=int((dist<=np.sqrt(2)*.05+1e-9).sum()),coverage=float((dist<=np.sqrt(2)*.05+1e-9).mean()))
a=dict(scope='HISTORICAL_MOVING_MULTIVIEW_DIAGNOSIS_NOT_ACCEPTANCE',rows=rows,old_source_static_candidate_and_nav_reject_preserved=True,no_actor_admission_or_new_success=True,causal_prefix_only=True,whole_memory_ttl10_not_all_future_union=True,teacher_read_after_prediction_seal=True,code_sha256=sha(__file__));(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
