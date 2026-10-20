from pathlib import Path
import json,hashlib,datetime,numpy as np
from anchor_evidence_v1 import qualify
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';B=H/'unknown_surface_pixel_box_development_v2_checked';O=H/'anchor_evidence_development_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();protocol=json.loads((H/'anchor_evidence_protocol_v1.json').read_text());assert all(sha(f)==s for f,s in protocol['source_sha256'].items());seal=json.loads((B/'prediction_pretruth_seal.json').read_text());assert all(sha(f)==s for f,s in seal['output_sha256'].items());O.mkdir(exist_ok=False);cad=TimedStaticBaseSelfMask();cache={};inputs={};out=[]
def packet(i):
 if i not in cache:
  rgbfile=D/'source_inputs'/f'frame_{i:03d}.npz';histfile=D/'history'/f'frame_{i:03d}.npz';inputs.update({str(p):sha(p) for p in [rgbfile,histfile]});rgb=dict(np.load(rgbfile));assert rgb['left_rgb'].shape==(480,640,3);history=np.load(histfile)['proprio_history'].copy();history[:,:9]=0;mask=cad.project(joints_from_proprio(history))['left'];np.savez_compressed(O/f'CAD_{i:03d}.npz',mask=mask);cache[i]=(rgb['left_rgb'],mask,float(rgb['time_s']))
 return cache[i]
rows=json.loads((B/'predictions.json').read_text());assert [r['frame'] for r in rows]==protocol['frames'];assert sum(len(r['geometries']) for r in rows)==355
for row in rows:
 f=row['frame'];a,m,t=packet(f);new=[]
 for r in row['geometries']:
  s,n,st=packet(r['source_frame']);assert abs(t-r['query_time_s'])<1e-6 and abs(st-r['source_time_s'])<1e-6;decision=qualify(a,s,r['query_xy'],r['source_xy'],m,n,t,st);new.append(dict(original=r,evidence=decision))
 out.append(dict(frame=f,geometries=new))
p=O/'predictions.json';p.write_text(json.dumps(out,indent=2)+'\n');(O/'predictions_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,parent_prediction_sha256=sha(B/'predictions.json'),output_sha256={str(f):sha(f) for f in O.iterdir() if f.is_file()},source_sha256={str(H/n):sha(H/n) for n in ['anchor_evidence_v1.py','probe_anchor_evidence_v1.py','anchor_evidence_protocol_v1.json','timed_static_base_self_mask.py','static_base_cached_self_mask_v2.py']},GT_depth_not_input=True,source_GTpose_only_parent_diagnostic=True,original_355_pairs_preserved=True),indent=2)+'\n');print([(r['frame'],len(r['geometries']),sum(g['evidence']['eligible_anchor'] for g in r['geometries'])) for r in out])
