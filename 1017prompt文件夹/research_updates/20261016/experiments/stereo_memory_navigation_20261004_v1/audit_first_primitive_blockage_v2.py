from pathlib import Path
import json,hashlib,types,inspect,sys,numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark.drive_model import DriveModel
import execution_domain_four_point_consumer_v2 as local
import minimum_prefix_transition_planning_v1 as search
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
D=H/'E1_continuous_tube_navigation_run_v1';rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];drive=DriveModel.from_json(P/'configs/benchmark_v1/drive_dynamics_v1.json');out=[]
for i,map_frame in [(506,506),(506,501),(511,511)]:
 r=rows[i];state=states[i];m=np.load(D/'maps'/f'frame_{map_frame:03d}.npz');g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();original=g.labels.copy();riskworld=g.origin+np.argwhere(g.labels==1)*g.resolution;trace=[];query=[]
 def collision(samples,**kw):
  passed=search._primitive_is_collision_free_unchecked(samples,**kw);frame=inspect.currentframe().f_back;v=frame.f_locals;cells=np.floor((samples[:,:2]-kw['origin_xy'])/kw['resolution_m']+.5).astype(int);inside=(cells>=0).all(axis=1)&(cells<np.asarray(kw['grid'].shape)).all(axis=1);bad=~inside;bad[inside]|=kw['grid'][cells[inside,0],cells[inside,1]]==kw['occupied'];nearest=[]
  for j in np.flatnonzero(bad):
   center=kw['origin_xy']+cells[j]*kw['resolution_m'];dist=np.linalg.norm(riskworld-center,axis=1);k=int(np.argmin(dist));nearest.append(dict(sample_index=int(j),sample_xy=samples[j,:2].tolist(),occupied_cell_xy=center.tolist(),closest_observed_risk_xy=riskworld[k].tolist(),grid_clearance_m=float(dist[k])))
  trace.append(dict(expansion=int(v['expanded']),search_state=list(v['current']),main=float(v['main']),aux=float(v['auxiliary']),direction=v['direction'],samples=samples.tolist(),passed=passed,blocking_samples=nearest));return passed
 traced=types.FunctionType(search.plan_kinodynamic_path.__code__,dict(search.plan_kinodynamic_path.__globals__,_primitive_is_collision_free_unchecked=collision),argdefs=search.plan_kinodynamic_path.__defaults__);traced.__kwdefaults__=search.plan_kinodynamic_path.__kwdefaults__
 def planner(req,drive,**kw):
  query.append(dict(start_xy=req.start_xy,start_yaw=req.start_yaw_rad,goal_xy=req.goal_xy,origin=req.origin_xy,shape=list(req.occupancy_grid.shape),first_direction=r['first_primitive_direction_query']['direction'],initial_direction=kw['initial_direction']));return traced(search.KinodynamicPlanRequest(**vars(req)),drive,first_primitive_direction=r['first_primitive_direction_query']['direction'],**kw)
 fn=types.FunctionType(local.local_curve_for_direction_tube.__code__,dict(local.local_curve_for_direction_tube.__globals__,plan_kinodynamic_path=planner));fn.__kwdefaults__=local.local_curve_for_direction_tube.__kwdefaults__
 R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));plan,meta=fn(g,np.asarray(state['position'][:2]),yaw,np.asarray(episode['goal_position_world_m'][:2]),drive,initial_direction=r['reference_decision']['direction_cost_input']);assert meta['status']==r['plan']['status'] and meta['expanded_nodes']==r['plan']['expanded_nodes'];assert np.array_equal(g.labels,original)
 if plan:
  saved=json.loads((D/'local_plans'/f'frame_{i:03d}.json').read_text())['waypoints'];assert [vars(w) for w in plan.waypoints]==saved
 snapshot=D/'execution'/f'frame_{i:03d}.json';out.append(dict(frame=i,map_frame=map_frame,query=query[0],status=meta['status'],expanded_nodes=meta['expanded_nodes'],requests_evaluated=len(trace),trace=trace,original_result_exact=True,labels_unchanged=True,current_execution_snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),physical_response_not_predicted=True))
result=dict(protocol=json.loads((H/'first_primitive_blockage_protocol_v1.json').read_text()),states=out);(D/'first_primitive_blockage_audit_v2.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps([dict(frame=r['frame'],status=r['status'],expanded=r['expanded_nodes'],checks=r['requests_evaluated'],first_expansions=[dict(expansion=a['expansion'],main=a['main'],aux=a['aux'],passed=a['passed'],block=a['blocking_samples'][:1]) for a in r['trace'][:12]]) for r in out]))
