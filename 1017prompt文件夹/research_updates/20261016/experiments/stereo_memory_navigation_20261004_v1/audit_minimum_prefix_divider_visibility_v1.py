"""Six frozen postseal exposures: missing physical divider coverage, not actor filtering."""
from pathlib import Path
import json,hashlib,numpy as np
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1'
frames=[0,80,115,210,215,330]
protocol=dict(frames=frames,target='/E1Detour/Divider/divider_06',maximum_exposures=6,GT_postseal_diagnostic_only=True,no_actor_or_threshold_change=True,no_new_physics_or_NN=True)
(H/'minimum_prefix_divider_visibility_protocol_v1.json').write_text(json.dumps(protocol,indent=2)+'\n')
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
obj=next(x for x in json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text()) if x['path']==protocol['target']);c=np.array(obj['position']);radius=obj['radius'];half=obj['height']/2
Y,X=np.indices((480,640));rays=np.stack(((X-319.5)/FOCAL,(Y-239.5)/FOCAL,np.ones_like(X)),axis=-1);out=[]
for i in frames:
 name=f'frame_{i:03d}.npz';gt=np.load(D/'frames'/name);pose=np.load(D/'known_poses'/name);pred=np.load(D/'source_predictions'/name);counts={}
 for eye in ['left','right']:
  T=pose[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);depth=gt[eye+'_depth_m'];world=T[:3,3]+(rays*depth[...,None])@R.T
  mask=np.isfinite(depth)&(depth>0)&np.isfinite(world).all(-1)&(np.linalg.norm(world[...,:2]-c[:2],axis=-1)<=radius+.001)&(np.abs(world[...,2]-c[2])<=half+.001)
  counts[eye+'_GT_visible_before_CAD_pixels']=int(mask.sum())
  if eye=='left':
   counts['left_GT_pixels_with_NN_confidence']=int((mask&pred['confidence'].astype(bool)).sum())
   packet=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');partial=np.load(D/'partial_observations'/name);flat=mask.ravel()
   counts['GT_target_query_pixels_after_pair_and_CAD']=int(flat[packet['query_pixel_id']].sum())
   counts['GT_target_query_pixels_retained_texture']=int(flat[partial['query_pixel_id']].sum())
   measured=partial['world'];counts['predicted_partial_points_near_target_shape']=int(((np.linalg.norm(measured[:,:2]-c[:2],axis=1)<=radius+.001)&(np.abs(measured[:,2]-c[2])<=half+.001)).sum())
 out.append(dict(frame=i,**counts))
r=dict(protocol=protocol,rows=out,source_point_precision_not_volume_recall=True,GT_visible_before_CAD_not_full_visibility_certificate=True,pair_or_CAD_stage_not_separated=True,no_truth_to_actor=True,no_new_fit=True,whole_research_complete=False)
(H/'minimum_prefix_divider_visibility_audit_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
