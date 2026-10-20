from pathlib import Path
import json,datetime,hashlib,numpy as np
from scipy.spatial import cKDTree
from partial_cue_source_v2 import inflate_xy
H=Path(__file__).resolve().parent;D=H/'bank_rock_observation_run_v1';S=H/'bilateral_core_memory_preflight_v2';O=H/'free_ray_contradiction_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((S/'outputs_pretruth_seal.json').read_text());assert all(sha(f)==s for f,s in seal.items());pool=np.load(S/'pool.npz');cores=pool['raw_world_voxel_centres'];proof=[[] for _ in cores];end=float(pool['time_s'])
for frame in range(0,141,5):
 q=np.load(D/'sensor_packets'/f'source_{frame:03d}.npz');pose=np.load(D/'estimated_poses'/f'frame_{frame:03d}.npz');t=float(q['time_s']);assert abs(t-float(pose['time_s']))<=1e-9 and t<=end
 if not len(q['world']):continue
 T=pose['left_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);local=(cores-T[:3,3])@R;metric_local=(q['world']-T[:3,3])@R
 with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
 valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480)
 pix=np.column_stack([q['query_pixel_id']%640,q['query_pixel_id']//640]);idx=np.flatnonzero(valid);d,j=cKDTree(pix).query(uv[idx]);delta=metric_local[j,2]-local[idx,2];take=(d<=np.sqrt(2)+1e-9)&(delta>.2)
 for k,pointid,dp,dz in zip(idx[take],j[take],d[take],delta[take]):proof[k].append(dict(source_frame=frame,time_s=t,query_pixel_id=int(q['query_pixel_id'][pointid]),partner_pixel_id=int(q['partner_pixel_id'][pointid]),pixel_distance=float(dp),core_opticalZ_m=float(local[k,2]),endpoint_opticalZ_m=float(metric_local[pointid,2]),endpoint_minus_core_Z_m=float(dz),metric_endpoint_world=q['world'][pointid].tolist()))
veto=np.array([bool(p) for p in proof]);np.savez_compressed(O/'diagnostic_flags.npz',cores=cores,contradicted=veto,time_s=end);(O/'proofs.json').write_text(json.dumps([dict(core_xyz=c.tolist(),past_ray_proofs=p) for c,p in zip(cores,proof)],indent=2)+'\n');(O/'pretruth_seal.json').write_text(json.dumps({str(f):sha(f) for f in [O/'diagnostic_flags.npz',O/'proofs.json']},indent=2)+'\n')
# Diagnostic ground truth only after above immutable evidence.
r=np.linalg.norm(cores[:,:2],axis=1);true_volume=(r<=.35)&(cores[:,2]>=0)&(cores[:,2]<=1);remaining=inflate_xy(cores[~veto,:2],.2);truth=np.load(D/'offline_occupancy_label.npz');xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(remaining).query(xy)[0]<=np.sqrt(2)*.05+1e-9).mean()) if len(remaining) else 0.
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),raw_core_count=len(cores),contradicted_cores=int(veto.sum()),true_occupied_cores=int(true_volume.sum()),true_occupied_contradicted=int((veto&true_volume).sum()),remaining_coverage=coverage,remaining_possible_area_m2=len(remaining)*.01,no_actor_change=True,diagnostic_only_not_admission=True,not_measured_free=True,historical_not_fresh=True,all_old_rejects_preserved=True,whole_research_complete=False,original35_not_triggered=True,proof_seal_sha256=sha(O/'pretruth_seal.json'),code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
