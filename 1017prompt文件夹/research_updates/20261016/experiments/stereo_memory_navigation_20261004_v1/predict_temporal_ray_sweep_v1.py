"""SIX frozen source-only known-pose development cases. Seal BEFORE labels."""
from pathlib import Path
import numpy as np,json,hashlib,datetime,time
from temporal_ray_sweep import TemporalRaySweep,project_depth,OPTICAL
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_ray_sweep_diagnostic_v1';assert not D.exists();D.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-temporal-ray-sweep-diagnostic.md';K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
(D/'protocol.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),plan_sha256=sha(plan),code_sha256=sha(__file__),module_sha256=sha(H/'temporal_ray_sweep.py'),CPU_suite_log_sha256=sha(H/'temporal_ray_sweep_full_suite_v1.log'),knownpose_privileged=True,development_not_fresh=True,six_fixed_cases=True,no_runtime_control=True),indent=2)+'\n');inputs={};rows=[]
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
for i in [235,245,255]:
 packets={}
 for j in [i,i-10,i-20]:
  rgb=load(O/'online_inputs'/f'frame_{j:03d}.npz');pose=load(O/'known_poses'/f'frame_{j:03d}.npz');mask=load(O/'masks'/f'frame_{j:03d}.npz');assert float(rgb['time_s'])==float(pose['time_s']);packets[j]=(rgb,pose,mask)
 for eye in ['left','right']:
  def packet(j):
   rgb,pose,mask=packets[j];return dict(time_s=float(rgb['time_s']),rgb=rgb[eye+'_rgb'],pose=pose[eye+'_camera_usd_world'],mask=mask[eye])
  c=packet(i);sources=[packet(i-10),packet(i-20)];start=time.perf_counter();out=TemporalRaySweep(K).predict(c,sources,c['time_s']);elapsed=time.perf_counter()-start;depth=out['depth_m'];yy,xx=np.indices(depth.shape);sel=np.isfinite(depth)&(yy%2==0)&(xx%2==0);uv=np.c_[xx[sel],yy[sel]];z=depth[sel];rays=np.c_[uv,np.ones(len(uv))]@np.linalg.inv(K).T;T=c['pose'];world=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@OPTICAL).T
  Rbody=(T[:3,:3]@OPTICAL)@np.array([[0,0,1],[0,-1,0],[1,0,0]]).T;root=T[:3,3]-Rbody@np.array([.2,.3 if eye=='left' else -.3,.05]);h=(world[:,2]>.05)&(world[:,2]<=1.5)&(np.linalg.norm(world-root,axis=1)>.55);uv=uv[h];world=world[h];z=z[h];past_uv=[project_depth(uv,z,T,s['pose'],K)[0] for s in sources];name=f'frame_{i:03d}_{eye}.npz'
  np.savez_compressed(D/name,**out,world=world,current_uv=uv,past_uv=np.array(past_uv),eye=eye,current_frame=i,source_frames=np.array([i-10,i-20]),query_pixel_id=uv[:,1]*640+uv[:,0],selected_depth_m=z,geometry_available_time_s=c['time_s'],certified_free=False,source_model_admitted=False,derived_root_position_world_m=root)
  rows.append(dict(frame=i,eye=eye,time_s=c['time_s'],latency_s=elapsed,all_finite_dense_depth=int(np.isfinite(depth).sum()),hazard_queries=len(world),file=name));print(rows[-1],flush=True)
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n');outputs=[p for p in D.iterdir() if p.is_file()];(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256={str(p.resolve()):sha(p) for p in outputs},all_six_predictions_before_GT=True,no_current_or_future_truth_or_actions_read=True,knownpose_privileged=True,no_controller_or_free_labels=True),indent=2)+'\n');print('SIX_SOURCE_ONLY_PREDICTIONS_SEALED',flush=True)
