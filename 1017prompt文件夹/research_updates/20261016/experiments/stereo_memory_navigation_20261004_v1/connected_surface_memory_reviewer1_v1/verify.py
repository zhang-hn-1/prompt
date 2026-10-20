from pathlib import Path
import hashlib,json,numpy as np,cv2,sys
H=Path(__file__).resolve().parents[1];sys.path.insert(0,str(H));from temporal_ray_sweep import project_depth
D=H/'connected_surface_memory_v1';R=H/'connected_surface_memory_reviewer1_v1';O=H/'qualified_hazard_navigation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'prediction_seal.json').read_text());results={}
for group in ['input_sha256','output_sha256']:
 assert all(sha(p)==s for p,s in seal[group].items());results[group+'_verified']=len(seal[group])
for key,path in [('code_sha256',H/'predict_connected_surface_memory_v1.py'),('module_sha256',H/'connected_surface_growth.py'),('source_contract_module_sha256',H/'temporal_ray_sweep.py'),('full_suite_sha256',H/'connected_surface_growth_full_suite_v1.log'),('plan_sha256',H.parent.parent/'docs/superpowers/plans/2026-10-07-source-connected-surface-memory-diagnostic.md')]:assert seal[key]==sha(path)
results['code_plan_log_seals_verified']=True
A=json.loads((D/'assessment_v1.json').read_text());B=json.loads((R/'replayed_outputs/assessment_v1.json').read_text());A.pop('utc');B.pop('utc');assert A==B;results['exact_assessment_except_utc']=True
results['packet_rows']=[];off=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)]);K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
for r in json.loads((D/'records.json').read_text()):
 f=np.load(D/r['file']);re=np.load(R/'replayed_outputs'/r['file']);src=np.load(H/'temporal_ray_sweep_diagnostic_v1'/r['file']);assert set(f.files)==set(re.files)
 for k in f.files:np.testing.assert_equal(f[k],re[k])
 for k in src.files:np.testing.assert_equal(src[k],f[k])
 t=float(f['current_time_s']);assert t==float(f['geometry_available_time_s']);assert np.all(f['source_time_s']<t);assert np.all(t-f['source_time_s']<=2.+1e-9)
 pu=(f['current_uv'][:,None,:]+off[None]).reshape(-1,2);z=np.repeat(f['selected_depth_m'],49);C=np.load(O/'known_poses'/f"frame_{r['frame']:03d}.npz");m=np.load(O/'masks'/f"frame_{r['frame']:03d}.npz")[r['eye']];assert np.all((pu[:,0]>=0)&(pu[:,0]<640)&(pu[:,1]>=0)&(pu[:,1]<480));x,y=pu.T;exact=(~m[y,x]).reshape(-1,49).all(1)
 for j in f['source_frames']:
  P=np.load(O/'known_poses'/f'frame_{j:03d}.npz');m=np.load(O/'masks'/f'frame_{j:03d}.npz')[r['eye']];xy,pz=project_depth(pu,z,C[r['eye']+'_camera_usd_world'],P[r['eye']+'_camera_usd_world'],K)
  inside=np.isfinite(xy).all(1)&(xy[:,0]>=0)&(xy[:,0]<=639)&(xy[:,1]>=0)&(xy[:,1]<=479)&(pz>0)
  vals=cv2.remap((~m).astype(np.float32),xy[:,0].reshape(-1,49).astype(np.float32),xy[:,1].reshape(-1,49).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
  exact&=(inside.reshape(-1,49)&(vals==1)).all(1)
 loose=f['bilinear_patch_contract_valid'];diff=loose&~exact
 results['packet_rows'].append(dict(file=r['file'],all_arrays_reproduced=True,original_fields_retained=True,strict_all49_contract_differences=int(diff.sum()),F0_accepted_differences=int((diff&f['F0_direct_accepted']).sum()),F1_accepted_differences=int((diff&f['F1_connected_accepted']).sum()),anchors=f['previous_anchor_counts'].tolist()))
# literal demonstration for threshold vs actual nonzero interpolation support
mask=np.ones((2,2),np.float32);mask[1,1]=0;value=float(cv2.remap(mask,np.array([[1/32]],np.float32),np.array([[1/32]],np.float32),cv2.INTER_LINEAR)[0,0]);results['bilinear_tiny_weight_fixture']=dict(value=value,accepted_by_gt999=value>.999,all_taps_unmasked=False)
(R/'verification.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
