from pathlib import Path
import json,hashlib,datetime,numpy as np
from tentative_point_memory import TentativePointMemory
from live_refined_surface_shadow import refined_surface
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';O=H/'fixed_appearance_memory_actual_verification_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();protocol=json.loads((H/'native_fixed_appearance_probe_protocol_v1.json').read_text())
for group in ('source_sha256','input_sha256'):
 for file,digest in protocol[group].items():assert sha(file)==digest,file
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for file,digest in manifest.items():assert sha(file)==digest,file
rows=json.loads((D/'memory_shadow_rows.json').read_text());parent=json.loads((D/'source_rows.json').read_text());assert len(rows)==len(parent)==29
m=TentativePointMemory();cad=RobotSelfMask();inputs=[];summaries=[]
for row,raw in zip(rows,parent):
 i=row['frame'];name=f'frame_{i:03d}.npz';surface=dict(np.load(D/'partial_observations'/name));rgb=np.load(D/'source_inputs'/name);pose=np.load(D/'estimated_poses'/name);classes=np.load(D/'partial_class_masks'/name);pred=np.load(D/'source_predictions'/name);hist=np.load(D/'history'/name);mask=cad.project(joints_from_proprio(hist['proprio_history']));t=row['time_s']
 assert float(rgb['time_s'])==float(pose['time_s'])==float(classes['time_s'])==float(pred['time_s'])==t
 assert i==int(surface['frame']) and raw['provenance']==row['input_sha256']
 for file,digest in row['input_sha256'].items():assert sha(file)==digest
 p=refined_surface(surface,rgb['left_rgb'],rgb['right_rgb'],mask,classes['right'],restore_native(pred['disparity_px']),pose['left_camera_usd_world'],t,row['input_sha256']);saved=np.load(D/'refined_shadow_points'/name)
 for k in ('world','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(p[k],saved[k])
 m.update(p,t);memory=m.at(t);serial={k:v for k,v in memory.items() if k!='records'};serial['records']=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in r.items()} for r in memory['records']];assert serial==json.loads((D/'tentative_memory_snapshots'/f'frame_{i:03d}.json').read_text())
 assert row['physics_clock_and_execution_unchanged'] and row['no_action_or_parent_count_change'] and not row['source_admitted']
 inputs.append(p);summaries.append({k:serial[k] for k in ('current_count','historical_count','distinct_XY_cells')})
# Every prefix exact; altered future input does not enter prefix processing.
for cutoff in range(len(inputs)):
 for poison in (False,True):
  stream=[dict(p,world=p['world'].copy()) for p in inputs]
  if poison:
   for p in stream[cutoff+1:]:p['world'][:]=1e9
  replay=TentativePointMemory()
  for j,p in enumerate(stream[:cutoff+1]):
   replay.update(p,p['time_s']);s=replay.at(p['time_s']);assert {k:s[k] for k in summaries[j]}==summaries[j]
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_ONLINE_READONLY_DIAGNOSTIC_MEMORY_INTERFACE_ONLY',actual29_reconstruction_exact=True,all29_future_replacement_and_truncation_prefixes_exact=True,original_parent_observer_reconstruction='see runtime verification; no strict cross-run physicalAB claimed',point_records=sum(r['current_count'] for r in rows),current_count_peak=max(r['current_count'] for r in rows),historical_count_peak=max(r['historical_count'] for r in rows),final_memory=summaries[-1],wall_shadow_p90_s=float(np.percentile([r['wall_s'] for r in rows],90)),source_admission=False,no_navigation_or_world_model_or_formal35=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
