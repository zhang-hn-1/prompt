from pathlib import Path
import sys,json,hashlib,collections,numpy as np
H=Path(__file__).resolve().parent
DEP=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');sys.path.insert(0,str(DEP))
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
from memory_temporal_ray_labels_v1 import label_ray
if __name__=='__main__':
 d=H/'sgbm_baseline35_e1_coverage_run_v1';seal=json.loads((d/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];files=[d/'partial_observations/frame_000.npz',d/'frames/frame_005.npz',d/'history/frame_005.npz'];identity={}
 for p in files:
  digest=hashlib.sha256(p.read_bytes()).hexdigest();assert seal[str(p)]==digest;identity[str(p)]=digest
 source=np.load(files[0]);capture=np.load(files[1]);history=np.load(files[2]);assert float(source['acquisition_time_s'])==0. and abs(float(capture['time_s'])-.5)<1e-6
 cad=TimedStaticBaseSelfMask();q=joints_from_proprio(history['proprio_history']);mask=cad.project(q)['left']
 results=[label_ray(point_world=point,camera_usd_world=capture['left_camera_usd_world'],depth_m=capture['left_depth_m'],self_mask=mask,past_time_s=float(source['acquisition_time_s']),current_time_s=float(capture['time_s'])) for point in source['world'][:64]]
 out=dict(design_sha256=hashlib.sha256((H/'memory_temporal_ray_label_probe_design_v1.json').read_bytes()).hexdigest(),inputs_sha256=identity,source_query_pixel_ids=source['query_pixel_id'][:64].tolist(),past_frame=0,current_frame=5,points=len(results),current_measured_joints=q.tolist(),projected_mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),masked_pixels=int(mask.sum()),reason_counts=dict(collections.Counter(r['reason'] for r in results)),label_counts=dict(collections.Counter(r['label'] for r in results)),results=results,new_physics=0,new_fit=0,partial_ground_surface_not_free_volume=True,whole_mission_complete=False)
 p=H/'memory_temporal_ray_label_probe_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print({k:v for k,v in out.items() if k not in ('inputs_sha256','results','source_query_pixel_ids')})
