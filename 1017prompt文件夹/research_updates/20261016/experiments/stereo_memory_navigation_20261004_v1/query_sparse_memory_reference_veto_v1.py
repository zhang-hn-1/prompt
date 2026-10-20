"""Historical current-known-reference positive-point query; no action API."""
from pathlib import Path
import json,hashlib,datetime,copy,numpy as np
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';M=H/'whole_historical_moving_memory_v1';O=H/'sparse_memory_reference_veto_query_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((M/'actor_seal.json').read_text())
for group in ['input_sha256','output_sha256']:
 for f,s in seal[group].items():assert sha(f)==s
policies=json.loads((D/'policy_records.json').read_text());O.mkdir(exist_ok=False);rows=[];inputs={str(D/'policy_records.json'):sha(D/'policy_records.json')}
def query(frame,records,mem,pose,origin,shape,resolution):
 r=records[frame];assert r['local']['reference_snapshot_valid']
 now=float(pose['time_s']);assert now==r['time_s'];path=np.asarray(r['local']['installed_reference_xy_after_act'],float)
 points=[];current=[]
 for v in mem['records']:
  if v['time_s']>now:continue # Never read future point payload or identifiers.
  assert now-v['time_s']<=10.+1e-10
  w=np.asarray(v['world'],float).reshape(-1,3);assert np.isfinite(w).all();points.extend(w.tolist())
  if v['time_s']==now:current.extend(w.tolist())
 out={}
 for name,world in [('current_only',current),('current_plus_past',points)]:
  world=np.asarray(world,float).reshape(-1,3);labels=np.full(shape,-1,np.int8);cells=np.rint((world[:,:2]-origin)/resolution).astype(int);inside=(cells>=0).all(1)&(cells<np.asarray(shape)).all(1);valid=cells[inside];labels[valid[:,0],valid[:,1]]=1
  q=reference_geometry_status(pose['root_position_world_m'][:2],path,labels,origin,resolution)
  out[name]={k:q[k] for k in ['suffix_clearance_m','bridge_clearance_m','continuous_clearance_pass','raster_clearance_pass','observed_hazard_count','projection_distance_m','remaining_arc_m','retention_necessary_screen_pass','reject_reasons']}
  # Legacy function field is explicitly relabelled: this query has no admitted observation.
  out[name]['tentative_positive_hazard_cells']=out[name].pop('observed_hazard_count');out[name]['point_records']=len(world);out[name]['empty_does_not_certify_free_or_safe']=not len(world);out[name]['action_authorized']=False;out[name]['full_volume_coverage_certified']=False
 return out
for i in range(140,600,5):
 name=f'frame_{i:03d}';files=[M/'memory'/(name+'.json'),D/'estimated_poses'/(name+'.npz'),D/'maps'/(name+'.npz')]
 for f in files:inputs[str(f)]=sha(f)
 memory=json.loads(files[0].read_text());pose=np.load(files[1]);grid=np.load(files[2]);origin=grid['origin'];shape=grid['labels'].shape;res=float(grid['resolution'])
 base=query(i,policies,memory,pose,origin,shape,res);assert query(i,policies[:i+1],memory,pose,origin,shape,res)==base
 poison=copy.deepcopy(memory);poison['records'].append(dict(time_s=float(pose['time_s'])+.5,world='INVALID_FUTURE_PAYLOAD_NEVER_READ'))
 records=policies[:i+1]+[None]*(len(policies)-i-1)
 assert query(i,records,poison,pose,origin,shape,res)==base
 rows.append(dict(frame=i,time_s=float(pose['time_s']),reference_version=policies[i]['local']['local_reference_version'],same_known_reference=True,queries=base,future_log_replacement_and_truncation_exact=True,hazard_verdict_changed=any(base['current_only'][k]!=base['current_plus_past'][k] for k in ['continuous_clearance_pass','raster_clearance_pass'])))
a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='HISTORICAL_READONLY_REJECTED_POINT_SOURCE_REFERENCE_QUERY',epochs=len(rows),epochs_history_adds_hazard_cells=sum(r['queries']['current_plus_past']['tentative_positive_hazard_cells']>r['queries']['current_only']['tentative_positive_hazard_cells'] for r in rows),epochs_hazard_verdict_changes=sum(r['hazard_verdict_changed'] for r in rows),epochs_empty_both=sum(r['queries']['current_plus_past']['empty_does_not_certify_free_or_safe'] for r in rows),all_future_prefix_tests_exact=all(r['future_log_replacement_and_truncation_exact'] for r in rows),no_new_actor_or_safety_or_source_admission=True,no_observed_floor_or_free_added=True,no_parent_MODEL_in_query_grid=True,no_truth_label_reads=True,no_navigation_or_worldmodel_or_formal35=True,whole_research_complete=False,code_sha256=sha(__file__),input_sha256=inputs,rows=rows)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k not in ['rows','input_sha256']},indent=2))
