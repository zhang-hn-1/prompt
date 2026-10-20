"""EXPLICIT ORACLE component ablation after predictive stress REJECT. Not deployable."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from dense_source_points import point_packet
from stereo_geometry import restore_native
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'online_RGB_memory_fresh_run_v1';D=H/'RGB_memory_frozen_pose_stress_v1';OUT=D/'oracle_component_audit_v1';assert not OUT.exists();OUT.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();samples=np.load(D/'all_pose_samples.npz');states=json.loads((O/'states.json').read_text());inputs={};results={}
for arm in ['true_position','true_orientation','full_true_pose']:
 out=OUT/arm;out.mkdir();memory=PastPointRGBMemory();rows=[]
 for f in sorted((O/'source_predictions').glob('*.npz')):
  pred=np.load(f);i=int(pred['frame']);t=float(pred['time_s']);est=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');real=np.load(O/'known_poses'/f'frame_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz');rp=np.array(states[i]['position']);ep=est['root_position_world_m'];known=dict(root_position_world_m=rp if arm!='true_orientation' else ep,time_s=t)
  for eye in ['left','right']:
   k=eye+'_camera_usd_world';T=est[k].copy()
   if arm=='true_position':T[:3,3]+=rp-ep
   elif arm=='true_orientation':T=real[k].copy();T[:3,3]+=ep-rp
   else:T=real[k].copy()
   known[k]=T
  prediction={k:pred[k] for k in ['depth_m','confidence','frame','time_s']};prediction['disparity_native_px']=restore_native(pred['disparity_px']);src=point_packet(prediction,known,mask['left'],mask['right']);haz=int(((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum());q=18-1e-6<=t<24-1e-6 and i>=4 and all(np.linalg.norm(v[:2])<.05 for v in samples['velocity'][i-4:i+1]) and haz>=100
  if not memory.seeded and q:memory.seed(src,rgb,t,qualified=True)
  proof=memory.query(rgb,known,mask,t);w=proof['past_world'][proof['appearance_corroborated']];np.savez_compressed(out/f'frame_{i:03d}.npz',**proof,frame=i,time_s=t);rows.append(dict(frame=i,time_s=t,seed_frame=memory.source_frame,support=int(proof['appearance_corroborated'].sum()),bins=len(np.unique(np.floor(w[:,:2]/.1).astype(int),axis=0)),span_xy_m=np.ptp(w[:,:2],axis=0).tolist() if len(w) else None))
  for p in [f,D/'estimated_poses'/f'frame_{i:03d}.npz',O/'known_poses'/f'frame_{i:03d}.npz',O/'online_inputs'/f'frame_{i:03d}.npz',O/'masks'/f'frame_{i:03d}.npz']:inputs[str(p.resolve())]=sha(p)
 results[arm]=rows
(OUT/'records.json').write_text(json.dumps(results,indent=2)+'\n');(OUT/'completion.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),code_sha256=sha(__file__),inputs_sha256=inputs,outputs_sha256={str(f.resolve()):sha(f) for f in OUT.rglob('*.npz')},explicit_oracle_current_pose_components=True,NOT_model_prediction_or_online=True,old_model_reject_NOT_rescored=True,labels_not_read_in_this_audit=True),indent=2)+'\n');print('Explicit oracle component outputs complete', {k:next(r for r in v if r['frame']==190) for k,v in results.items()})
