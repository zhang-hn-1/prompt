"""Build held causal features after weight freeze; no held GT labels read."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from learned_cell_memory import CellMemory
from packet_scalars import normalize_npz_packet
from point_support_shadow import PointSupportMap
from dense_source_points import records
from grid_only_snapshot import grid_arrays_snapshot
H=Path(__file__).resolve().parent;O=H/'frozen_learned_memory_D3_features_v1';O.mkdir(exist_ok=False);cases=['flat_ground_pose_D3_run_v1'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();inputs=[Path(__file__).resolve(),H/'packet_scalars.py',H/'learned_cell_memory.py',H/'point_support_shadow.py',H/'dense_source_points.py',H/'grid_only_snapshot.py'];paths={}
weight_seal=json.loads((H/'learned_memory_fit/fitted_weights_prevalidation_sha256.json').read_text());assert sha(H/'learned_memory_fit/Mmem1.pt')==weight_seal['weight_sha256'];assert json.loads((H/'flat_ground_pose_stage_status_v1.json').read_text())['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION';assert json.loads((H/'flat_ground_pose_actual_assessment_v1.json').read_text())['decision']=='PASS_CONDITIONAL_ESTIMATED_POSE_TASK_ONLY'
for case in cases:
 o=H/case;paths[case]=sorted((o/'sensor_packets').glob('*.npz'));inputs.extend([o/'maps/frame_000.npz'])
 for p in paths[case]:
  i=int(p.stem.split('_')[-1]);inputs.append(p)
  for folder in ['online_inputs','estimated_poses','history','masks']:inputs.append(o/folder/f'frame_{i:03d}.npz')
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs},held_cases=cases,historical_D3_not_fresh=True,designer_has_seen_navigation_and_source_labels=True,processor_occupancy_labels_not_read=True,weights_predate_D3=True,GT_or_occupancy_labels_not_opened=True,feature_definition='frozen21features and source-time-only snapshots; no absoluteXY/cell IDs as features',fitting_already_frozen=True,no_refit=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');summary=[]
for case in cases:
 o=H/case;dest=O/case;dest.mkdir();initial=np.load(o/'maps/frame_000.npz');m=CellMemory(initial['origin'],initial['labels'].shape,float(initial['resolution']));baseline=PointSupportMap(initial['origin'],initial['labels'].shape,float(initial['resolution']));indices=[]
 for p in paths[case]:
  d=np.load(p);packet=normalize_npz_packet({k:d[k] for k in d.files});i=int(d['frame']);t=float(d['time_s']);rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(o/'estimated_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');mask=np.load(o/'masks'/f'frame_{i:03d}.npz');assert float(rgb['time_s'])==float(pose['time_s'])==float(mask['time_s'])==t
  context=dict(time_s=t,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],left_mask=mask['left'],left_camera_position=pose['left_camera_usd_world'][:3,3],root_position=pose['root_position_world_m'],previous_request=hist['action_history'][-1]);m.ingest(packet,context,t);s=m.snapshot(t);baseline.update(records(packet),t,float(packet['pose_sigma_m']));b=grid_arrays_snapshot(baseline,t);cells=s['cells'];labels=b['labels'][cells[:,0],cells[:,1]] if len(cells) else np.zeros(0,np.int8)
  np.savez_compressed(dest/f'frame_{i:03d}.npz',features=s['features'],cells=cells,time_s=t,last_query_times=s['last_query_times'],baseline_history_labels=labels,origin=m.origin,resolution=m.resolution,model_outputs_are_observed=False,source_model_admitted=False)
  indices.append(dict(frame=i,time_s=t,eligible_cells=len(cells),raw_queries=len(d['world']),provenance_original_inputs=str(p),provenance_reconstructible=True));print('HELD_FEATURES',case,i,len(cells),flush=True)
 (dest/'index.json').write_text(json.dumps(indices,indent=2)+'\n');summary.append(dict(case=case,frames=len(indices),cell_time_rows=sum(x['eligible_cells'] for x in indices)))
seal={str(p):sha(p) for p in O.rglob('*') if p.is_file()};(O/'feature_tables_prelabel_sha256.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=seal,GT_labels_not_read=True,fitting_already_frozen=True,no_refit=True),indent=2)+'\n');(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print('HELD_FEATURES_SEALED_NO_LABELS',summary)
