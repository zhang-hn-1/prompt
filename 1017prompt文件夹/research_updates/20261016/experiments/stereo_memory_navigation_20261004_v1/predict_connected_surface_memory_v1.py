"""Single development graph consumer; no labels, action or unobserved geometry."""
from pathlib import Path
import numpy as np, cv2, json, hashlib, datetime
from scipy.spatial import cKDTree
from temporal_ray_sweep import project_depth
from connected_surface_growth import select_connected
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';S=H/'temporal_ray_sweep_diagnostic_v1';D=H/'connected_surface_memory_v1'
assert not D.exists();D.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((S/'prediction_seal.json').read_text());assert all(sha(p)==s for p,s in seal['output_sha256'].items())
inputs={}
def load(p):
 inputs[str(p.resolve())]=sha(p);return np.load(p)
seed=load(O/'sensor_packets/source_180.npz');sw=seed['world'];haz=(sw[:,2]>.05)&(sw[:,2]<=1.5);sw=sw[haz];st=float(seed['time_s']);hist={arm:[(st,sw)] for arm in ['F0','F1']};rows=[]
K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
off=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)])
records=json.loads((S/'records.json').read_text())
for frame in [235,245,255]:
 packets=[];valid=[]
 for eye in ['left','right']:
  r=next(r for r in records if r['frame']==frame and r['eye']==eye);f=load(S/r['file']);uv=f['current_uv'];n=len(uv);patch=uv[:,None,:]+off[None];pu=patch.reshape(-1,2);z=np.repeat(f['selected_depth_m'],49);C=load(O/'known_poses'/f'frame_{frame:03d}.npz');cm=load(O/'masks'/f'frame_{frame:03d}.npz')[eye]
  x,y=pu.astype(int).T;v=(~cm[y,x]).reshape(n,49).all(1)
  for j in f['source_frames']:
   B=load(O/'known_poses'/f'frame_{j:03d}.npz');m=load(O/'masks'/f'frame_{j:03d}.npz')[eye]
   assert float(B['time_s'])<float(C['time_s'])
   xy,pz=project_depth(pu,z,C[eye+'_camera_usd_world'],B[eye+'_camera_usd_world'],K)
   inside=np.isfinite(xy).all(1)&(xy[:,0]>=0)&(xy[:,0]<=639)&(xy[:,1]>=0)&(xy[:,1]<=479)&(pz>0)
   mx=xy[:,0].reshape(n,49).astype(np.float32);my=xy[:,1].reshape(n,49).astype(np.float32)
   support=cv2.remap((~m).astype(np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)>.999
   v&=(inside.reshape(n,49)&support).all(1)
  packets.append((r,f));valid.append(v)
 t=float(packets[0][1]['current_time_s']);assert t==float(packets[1][1]['current_time_s'])
 w=np.concatenate([f['world'] for r,f in packets]);v=np.concatenate(valid);chosen={};counts=[]
 for arm in ['F0','F1']:
  hist[arm]=[(age,a) for age,a in hist[arm] if 0<=t-age<=10.]
  anchors=np.concatenate([a for age,a in hist[arm]]) if hist[arm] else np.empty((0,3));q=w[v]
  keep=cKDTree(anchors).query(q)[0]<=.1 if arm=='F0' and len(anchors) else select_connected(anchors,q)
  accept=np.zeros(len(w),bool);accept[v]=keep;chosen[arm]=accept;counts.append(len(anchors));hist[arm].append((t,w[accept].copy()))
 offset=0
 for (r,f),support in zip(packets,valid):
  n=len(f['world']);name=r['file'];np.savez_compressed(D/name,**{k:f[k] for k in f.files},bilinear_patch_contract_valid=support,F0_direct_accepted=chosen['F0'][offset:offset+n],F1_connected_accepted=chosen['F1'][offset:offset+n],previous_anchor_counts=np.array(counts),not_sensor_admitted=True)
  rows.append(dict(**r,input_contract_rejected=int((~support).sum()),F0_points=int(chosen['F0'][offset:offset+n].sum()),F1_points=int(chosen['F1'][offset:offset+n].sum())));offset+=n
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n')
plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-source-connected-surface-memory-diagnostic.md'
(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256={str(p.resolve()):sha(p) for p in D.iterdir() if p.is_file()},plan_sha256=sha(plan),code_sha256=sha(__file__),module_sha256=sha(H/'connected_surface_growth.py'),source_contract_module_sha256=sha(H/'temporal_ray_sweep.py'),full_suite_sha256=sha(H/'connected_surface_growth_full_suite_v1.log'),only_original_proposal_nodes=True,no_truth_actions_or_RGB_inference_read=True,knownpose_privileged=True,development_not_fresh=True,no_old_geometry_time_refresh=True,no_free_or_control_admission=True),indent=2)+'\n')
print(rows)
