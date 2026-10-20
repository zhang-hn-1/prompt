"""Postsealed privileged footprint-near visibility, not production instance selection."""
from pathlib import Path
import json,numpy as np,hashlib
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'moving_multiview_point_memory_diagnostic_v1';g=np.load(D/'offline_occupancy_label.npz');xy=g['origin']+np.argwhere(g['labels']>0)*float(g['resolution']);tree=cKDTree(xy);xx,yy=np.meshgrid(np.arange(640),np.arange(480));ray=np.stack(((xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones_like(xx)),axis=-1);rows=[]
for f in [20,100,200,300,400]:
 gt=np.load(D/'offline_truth'/f'frame_{f:03d}.npz');mask=np.load(D/'masks'/f'frame_{f:03d}.npz');source=np.load(D/'sensor_packets'/f'source_{f:03d}.npz');eyes={}
 for eye in ['left','right']:
  z=gt[eye+'_depth_m'];valid=np.isfinite(z)&(z>0);T=gt[eye+'_camera_usd_world'];w=(T[:3,:3]@np.diag([1,-1,-1])@(ray[valid]*z[valid,None]).T).T+T[:3,3];haz=(w[:,2]>.05)&(w[:,2]<=1.5);near=tree.query(w[:,:2])[0]<=.05+1e-9;hits=haz&near;eyes[eye]=dict(physical_footprint_hazard_pixels=int(hits.sum()),not_CAD=int((hits&~mask[eye][valid]).sum()),interpretation='postsealed teacher diagnostic, not production selection or exact instance segmentation')
 rows.append(dict(frame=f,raw_points=len(source['world']),raw_predicted_hazard=int(((source['world'][:,2]>.05)&(source['world'][:,2]<=1.5)).sum()),eyes=eyes))
r=dict(scope='POSTSEAL_PRIVILEGED_VISIBILITY_CAUSE_DIAGNOSIS_ONLY',rows=rows,no_actor_change=True,footprint_nearness_not_exact_instance=True,prediction_seal_sha256=hashlib.sha256((O/'prediction_seal.json').read_bytes()).hexdigest());existing=json.loads((O/'postseal_view_loss_diagnosis_v1.json').read_text());assert existing==r
(O/'postseal_visibility_reproduction_v1.json').write_text(json.dumps(dict(original_inline_output_exact=True,saved_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),source_saved_after_inline_execution=True),indent=2)+'\n');print('Saved standalone reproduction exactly matches original inline diagnosis')
