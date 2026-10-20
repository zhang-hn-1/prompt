"""Exactly one grid and two nominal Hybrid searches on one sealed snapshot."""
from pathlib import Path
import json,hashlib,datetime,time,sys
from dataclasses import asdict
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark import kinodynamic_planning as old
from benchmark.drive_model import DriveModel
from benchmark.policies import DEFAULT_DRIVE_PROFILE
from direction_root_cost import snapshot_problem,prepare_request,objective_summary
from direction_root_planning import plan_kinodynamic_path
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def arrays(plan):
 return dict(pose=np.array([[p.x_m,p.y_m,p.yaw_rad] for p in plan.waypoints],float).reshape(-1,3),direction=np.array([1 if p.drive_direction=='forward' else -1 for p in plan.waypoints],int),command=np.array([[p.main_command_rad_s,p.auxiliary_angle_rad] for p in plan.waypoints],float).reshape(-1,2))
def main():
 started=datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'direction_root_cost_protocol.json').read_text());assert p['sealed_utc']<started and not (H/'direction_root_cost_assessment.json').exists() and not (H/'direction_root_cost_status.json').exists()
 for d in [p['source_sha256'],p['input_sha256']]:
  for path,digest in d.items():assert sha(path)==digest,path
 status=dict(started_utc=started,status='running',actual_grid_searches=0,actual_hybrid_searches=0,new_simulations=0)
 (H/'direction_root_cost_status.json').write_text(json.dumps(status,indent=2)+'\n')
 packed=np.load(H/'direction_root_cost_snapshot.npz');maps={k:packed[k] for k in ['labels','provenance','floor_time','hazard_time','origin','resolution','time_s']};current={k:packed[k] for k in ['frame','position_world_xy','yaw_world_rad','pointgoal_base_xy']};current.update(time_s=float(packed['current_time_s']),accepted_direction=str(packed['accepted_direction']))
 s=snapshot_problem(maps,current);request,meta=prepare_request(s);status['actual_grid_searches']=1;assert request is not None
 provenance=json.loads((H/'direction_root_cost_provenance.json').read_text());np.testing.assert_equal(meta['local_target'],provenance['expected_saved_meta']['local_target']);np.testing.assert_equal(meta['route_xy'],provenance['expected_saved_route'])
 model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE);results={};saved={}
 for name,initial in [('baseline',None),('candidate',p['initial_direction'])]:
  tick=time.perf_counter()
  result=old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**vars(request)),model) if name=='baseline' else plan_kinodynamic_path(request,model,initial_direction=initial)
  status['actual_hybrid_searches']+=1;elapsed=time.perf_counter()-tick;saved[name]=arrays(result)
  np.savez_compressed(H/f'direction_root_cost_{name}_reference.npz',**saved[name])
  summary=objective_summary(result,request,p['initial_direction'])
  results[name]=dict(status=result.status,expanded_nodes=result.expanded_nodes,path_length_m=result.path_length_m,direction_changes=result.direction_changes,seconds=elapsed,current_direction_objective=summary,selected_under_current_root_cost=name=='candidate')
  if name=='baseline':
   expected=np.load(H/'direction_root_cost_expected_baseline.npz')
   for key in ['pose','direction','command']:np.testing.assert_equal(saved[name][key],expected[key])
   assert result.expanded_nodes==provenance['expected_saved_meta']['expanded_nodes']==1821
 np.savez_compressed(H/'direction_root_cost_request.npz',occupancy_grid=request.occupancy_grid,origin_xy=request.origin_xy,resolution_m=request.resolution_m,start_xy=request.start_xy,start_yaw_rad=request.start_yaw_rad,goal_xy=request.goal_xy)
 both=all(r['status']=='success' for r in results.values());changed=bool(both and saved['baseline']['direction'][0]!=saved['candidate']['direction'][0]);equal=all(np.array_equal(saved['baseline'][k],saved['candidate'][k]) for k in saved['baseline'])
 result=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),started_utc=started,protocol_sha256=sha(H/'direction_root_cost_protocol.json'),frame=p['frame'],problem_meta=meta,results=results,baseline_exact_to_archive=True,actual_grid_searches=1,actual_hybrid_searches=2,initial_direction=p['initial_direction'],both_success=both,first_direction_changed=changed,full_reference_equal=equal,nominal_behavior_eligibility_pending_geometry=both and changed,interface_pending_independent_verification=True,physical_control_changed=False,physical_candidate_trajectory_available=False,new_simulations=0,new_model_inference=0,formal_navigation_tested=False,whole_research_complete=False)
 (H/'direction_root_cost_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
 status.update(status='finished',ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exit_code=0);(H/'direction_root_cost_status.json').write_text(json.dumps(status,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k!='problem_meta'},indent=2))
if __name__=='__main__':main()
