"""Compare independent actual arms, never invent a counterfactual trajectory."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;A=H/'live_generic_height_navigation_run_v1';B=H/'live_paired_texture_navigation_run_v1';O=H/'paired_texture_live_common_prefix_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for d in (A,B):
 for f,s in json.loads((d/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f
ar=np.load(A/'requests.npz')['requests'];br=np.load(B/'requests.npz')['requests'];n=min(len(ar),len(br));diff=np.flatnonzero(np.any(ar[:n]!=br[:n],axis=1));first=None if not len(diff) else int(diff[0]);end=5*n+1 if first is None else first*5+1
az=np.load(A/'physics.npz');bz=np.load(B/'physics.npz');fields=['root_position_world_m','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s','smoothed_target','applied_action','position_world_xy','simulation_time_s'];physical={k:bool(np.array_equal(az[k][:end],bz[k][:end])) for k in fields};rgb=[];pred=[];raw=[]
for i in range(0,n if first is None else first+1,5):
 name=f'frame_{i:03d}.npz'
 for folder,keys,out in [('source_inputs',('left_rgb','right_rgb'),rgb),('source_predictions',('depth_m','confidence','disparity_px'),pred)]:
  a=np.load(A/folder/name);b=np.load(B/folder/name);out.append(dict(frame=i,exact=all(np.array_equal(a[k],b[k]) for k in keys)))
 a=np.load(A/'sensor_packets'/f'source_{i:03d}.npz');b=np.load(B/'sensor_packets'/f'source_{i:03d}.npz');raw.append(dict(frame=i,exact=all(np.array_equal(a[k],b[k]) for k in ('world','query_pixel_id','partner_pixel_id'))))
firstbad=lambda rows:next((r['frame'] for r in rows if not r['exact']),None)
r=dict(first_request_difference_frame=first,first_effect_physics_index=None if first is None else end,physical_prefix_exact=physical,observer_requests140_exact=bool(np.array_equal(ar[:140],br[:140])),first_RGB_difference_frame=firstbad(rgb),first_NN_difference_frame=firstbad(pred),first_raw_source_difference_frame=firstbad(raw),same_perception_before_first_action_difference=all(x['exact'] for rows in (rgb,pred,raw) for x in rows),not_matched_perception_causal_AB=not all(x['exact'] for rows in (rgb,pred,raw) for x in rows),one_arm_each_not_statistical=True,not_a_claim_of_memory_benefit=True,old_generic_rejected_preserved=True,not_original35=True,whole_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(O/'RGB_NN_raw_prefix_rows.json').write_text(json.dumps(dict(rgb=rgb,nn=pred,raw=raw),indent=2)+'\n');print(json.dumps(r,indent=2))
