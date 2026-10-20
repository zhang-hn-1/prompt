"""Whole-sequence frozen reliability transfer, current/past only, no labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np,torch
from learned_cell_memory import CellMemory
from memory_reliability_head import ReliabilityHead
from point_support_shadow import PointSupportMap
from dense_source_points import records
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from packet_scalars import normalize_npz_packet
H=Path(__file__).resolve().parent;D=H/'live_generic_height_navigation_run_v1';O=H/'frozen_head_generic_domain_predictions_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for name in ['live_generic_height_navigation_pose_assessment_v1','live_generic_height_navigation_source_assessment_v1']:
 a=json.loads((H/name/'assessment.json').read_text());assert all(a.get('gates',a.get('checks',{})).values()) and a['decision'].startswith('PASS')
assert json.loads((H/'live_generic_height_navigation_trajectory_v1/assessment.json').read_text())['termination']['termination']=='success'
for n in ['live_generic_height_navigation_runtime_verification_v1.json','live_generic_height_memory_consumption_assessment_v1.json','live_generic_height_policy_causal_assessment_v2.json']:assert (H/n).exists()
seal=json.loads((H/'learned_memory_fit/fitted_weights_prevalidation_sha256.json').read_text());weights=H/'learned_memory_fit/Mmem1.pt';assert sha(weights)==seal['weight_sha256']
O.mkdir(exist_ok=False);(O/'frames').mkdir();torch.set_num_threads(1);s=torch.load(weights,weights_only=True,map_location='cpu');net=ReliabilityHead(s['mean'].numpy(),s['scale'].numpy());net.load_state_dict(s)
initial=np.load(D/'maps/frame_000.npz');origin=initial['origin'];shape=initial['labels'].shape;res=float(initial['resolution']);m=CellMemory(origin,shape,res);base=PointSupportMap(origin,shape,res);cad=RobotSelfMask();inputs={str(weights):sha(weights)};summary=[]
for row in json.loads((D/'source_rows.json').read_text()):
 i=row['frame'];t=row['time_s'];name=f'frame_{i:03d}.npz';files=[D/'partial_observations'/name,D/'sensor_packets'/f'source_{i:03d}.npz',D/'source_inputs'/name,D/'estimated_poses'/name,D/'history'/name]
 for f in files:inputs[str(f)]=sha(f)
 a=np.load(files[0]);raw=normalize_npz_packet(dict(np.load(files[1])));packet={k:raw[k] for k in ['frame','time_s','frame_width','pose_sigma_m','source_model_admitted','certified_free']}
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:packet[k]=a[k]
 rgb=np.load(files[2]);pose=np.load(files[3]);hist=np.load(files[4]);assert float(rgb['time_s'])==float(pose['time_s'])==t
 masks=cad.project(joints_from_proprio(hist['proprio_history']));ctx=dict(time_s=t,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],left_mask=masks['left'],left_camera_position=pose['left_camera_usd_world'][:3,3],root_position=pose['root_position_world_m'],previous_request=hist['action_history'][-1]);m.ingest(packet,ctx,t);z=m.snapshot(t);base.update(records(packet),t,float(packet['pose_sigma_m']));b=base.snapshot(t);cells=z['cells'];scores=net.predict(z['features'].astype(np.float32));bl=b['labels'][cells[:,0],cells[:,1]];assert (z['last_query_times']<=t).all()
 np.savez_compressed(O/'frames'/name,cells=cells,features=z['features'],scores=scores,time_s=t,last_query_times=z['last_query_times'],baseline_history_labels=bl,origin=origin,resolution=res,model_outputs_are_observed=False)
 summary.append(dict(frame=i,time_s=t,cells=len(cells),scores_ge_half=int((scores>=.5).sum())))
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(O/'all_predictions_pretruth_sha256.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),inputs=inputs,outputs={str(p):sha(p) for p in O.rglob('*') if p.is_file()},source_sha256={str(p):sha(p) for p in [Path(__file__).resolve(),H/'learned_cell_memory.py',H/'memory_reliability_head.py',H/'point_support_shadow.py']},whole_sequence=True,labels_not_read=True,no_refit=True,old_rejects_preserved=True),indent=2)+'\n');print('ALL PREDICTIONS SEALED',len(summary),sum(r['cells'] for r in summary))
