"""Same-time saved interface audit; no inference, weights, GT occupancy or physics."""
from pathlib import Path
import numpy as np,json,hashlib
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';F=H/'frozen_learned_memory_D3_features_v1'/D.name;P=H/'frozen_learned_memory_D3_predictions_v1'/D.name;O=H/'learning_memory_domain_bridge_development_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();protocol=H/'learning_memory_domain_bridge_protocol_v1.json';proto=json.loads(protocol.read_text());inputs={str(protocol):sha(protocol)};rows=[]
for f in proto['frames']:
 paths=[D/'maps'/f'frame_{f:03d}.npz',F/f'frame_{f:03d}.npz',P/f'frame_{f:03d}.npz']
 for path in paths:inputs[str(path)]=sha(path)
 m=np.load(paths[0]);feature=np.load(paths[1]);pred=np.load(paths[2]);assert abs(float(m['time_s'])-float(pred['time_s']))<1e-8;assert np.array_equal(m['origin'],pred['origin']) and float(m['resolution'])==float(pred['resolution']);assert np.array_equal(pred['cells'],feature['cells'])
 cells=pred['cells'];selected=cells[pred['scores']>=.5];eligible=np.zeros(m['labels'].shape,bool);positive=eligible.copy()
 if len(cells):eligible[cells[:,0],cells[:,1]]=True
 if len(selected):positive[selected[:,0],selected[:,1]]=True
 model=(m['provenance']==6)|(m['provenance']==7);actoroccupied=m['labels']==1
 rows.append(dict(frame=f,time_s=float(m['time_s']),raw_eligible_cells=int(eligible.sum()),frozen_head_positive_cells=int(positive.sum()),actor_occupied_cells=int(actoroccupied.sum()),actor_MODEL_provenance_cells=int(model.sum()),MODEL_cells_with_raw_features=int((model&eligible).sum()),MODEL_cells_with_frozen_head_positive=int((model&positive).sum()),actor_occupied_with_raw_features=int((actoroccupied&eligible).sum()),actor_occupied_with_frozen_head_positive=int((actoroccupied&positive).sum()),provenance_histogram={str(int(k)):int(v) for k,v in zip(*np.unique(m['provenance'],return_counts=True))},actor_measured_hazard_times=int(np.isfinite(m['hazard_time']).sum()),actor_measured_floor_times=int(np.isfinite(m['floor_time']).sum()),actor_birth_times=np.unique(m['hypothesis_birth_time'][np.isfinite(m['hypothesis_birth_time'])]).tolist(),actor_model_barrier_not_observation=bool(m['model_barrier_not_observation']),snapshot_role='two_different_interfaces_not_matched_control'))
result=dict(rows=rows,old_head_inference_not_repeated=True,no_offline_GT_labels_read=True,no_future_frame_input=True,no_actor_change=True,no_query_ID_or_measured_depth_fabrication=True,working_conditional_baseline_not_general_source_admission=True,learning_teacher_FROM_model_MAP_not_observation=True,old_D3_not_fresh=True,whole_research_complete=False)
p=O/'assessment.json';p.write_text(json.dumps(result,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(inputs_sha256=inputs,source_sha256=sha(Path(__file__)),output_sha256={str(p):sha(p)}),indent=2)+'\n');print(json.dumps(rows,indent=2))
