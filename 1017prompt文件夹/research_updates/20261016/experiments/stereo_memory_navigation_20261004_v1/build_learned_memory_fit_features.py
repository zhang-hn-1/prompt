"""Build fit-only causal features; no fitting or heldout path access/GT labels."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from learned_cell_memory import CellMemory
from point_support_shadow import PointSupportMap
from dense_source_points import records
from grid_only_snapshot import grid_arrays_snapshot
H=Path(__file__).resolve().parent;O=H/'learned_memory_fit_features';O.mkdir(exist_ok=False);cases=['fresh_rolling_prior_shadow_current_run','occluded_obstacle_memory_navigation_history_run','short_obstacle_current_run','short_obstacle_history_run','retreat_observation_run'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();inputs=[Path(__file__).resolve(),H/'learned_cell_memory.py',H/'point_support_shadow.py',H/'dense_source_points.py',H/'grid_only_snapshot.py'];paths={}
for case in cases:
 o=H/case;paths[case]=sorted((o/'sensor_packets').glob('*.npz'));inputs.extend([o/'states.json',o/'maps/frame_000.npz'])
 for p in paths[case]:
  i=int(p.stem.split('_')[-1]);inputs.append(p)
  for folder in ['online_inputs','known_poses','history','masks']:inputs.append(o/folder/f'frame_{i:03d}.npz')
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs},fit_cases=cases,heldout_paths_not_opened=True,GT_or_occupancy_labels_not_opened=True,feature_definition='frozen21features and source-time-only snapshots; no absoluteXY/cell IDs as features',fitting_not_started=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');summary=[]
for case in cases:
 o=H/case;dest=O/case;dest.mkdir();states=json.loads((o/'states.json').read_text());initial=np.load(o/'maps/frame_000.npz');m=CellMemory(initial['origin'],initial['labels'].shape,float(initial['resolution']));baseline=PointSupportMap(initial['origin'],initial['labels'].shape,float(initial['resolution']));indices=[]
 for p in paths[case]:
  d=np.load(p);packet={k:d[k] for k in d.files};i=int(d['frame']);t=float(d['time_s']);rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');mask=np.load(o/'masks'/f'frame_{i:03d}.npz');assert float(rgb['time_s'])==float(pose['time_s'])==float(mask['time_s'])==t
  context=dict(time_s=t,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],left_mask=mask['left'],left_camera_position=pose['left_camera_usd_world'][:3,3],root_position=np.array(states[i]['position']),previous_request=hist['action_history'][-1]);m.ingest(packet,context,t);s=m.snapshot(t);baseline.update(records(packet),t,float(packet['pose_sigma_m']));b=grid_arrays_snapshot(baseline,t);cells=s['cells'];labels=b['labels'][cells[:,0],cells[:,1]] if len(cells) else np.zeros(0,np.int8)
  np.savez_compressed(dest/f'frame_{i:03d}.npz',features=s['features'],cells=cells,time_s=t,last_query_times=s['last_query_times'],baseline_history_labels=labels,origin=m.origin,resolution=m.resolution,model_outputs_are_observed=False,source_model_admitted=False)
  indices.append(dict(frame=i,time_s=t,eligible_cells=len(cells),raw_queries=len(d['world']),provenance_original_inputs=str(p),provenance_reconstructible=True));print('FIT_FEATURES',case,i,len(cells),flush=True)
 (dest/'index.json').write_text(json.dumps(indices,indent=2)+'\n');summary.append(dict(case=case,frames=len(indices),cell_time_rows=sum(x['eligible_cells'] for x in indices)))
seal={str(p):sha(p) for p in O.rglob('*') if p.is_file()};(O/'feature_tables_prelabel_sha256.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=seal,GT_labels_not_read=True,fitting_not_started=True),indent=2)+'\n');(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print('FIT_ONLY_FEATURES_SEALED',summary)
