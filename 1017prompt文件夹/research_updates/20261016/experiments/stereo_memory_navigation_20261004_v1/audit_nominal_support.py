"""Exactly one current snapshot, original+one planner-only support candidate."""
from pathlib import Path
import sys,json,hashlib,datetime,math,time
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from direction_root_cost import snapshot_problem,prepare_request,objective_summary
from direction_root_planning import plan_kinodynamic_path,sample_motion_primitive,primitive_is_collision_free
from outgoing_lookahead import make_endpoint_oracle
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference
from training.action_protocol import load_action_protocol
from nominal_support import make_support_drive,common_support
from bank_mapping import BankMap
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serialize(plan):
 return dict(status=plan.status,pose=[[w.x_m,w.y_m,w.yaw_rad] for w in plan.waypoints],direction=[1 if w.drive_direction=='forward' else -1 for w in plan.waypoints],command=[[w.main_command_rad_s,w.auxiliary_angle_rad] for w in plan.waypoints],expanded_nodes=plan.expanded_nodes,path_length_m=plan.path_length_m,direction_changes=plan.direction_changes)
def summarize(plan,req,drive,base,bm,initial):
 d=serialize(plan);d['objective']=objective_summary(plan,req,initial);d['primitive_checks']=[]
 if plan.status!='success' or len(plan.waypoints)<2:return d
 d['local_goal_distance_m']=float(np.linalg.norm(np.array(d['pose'][-1][:2])-req.goal_xy));occupied=0;unsupported=0
 for idx,w in enumerate(plan.waypoints[:-1]):
  main=w.main_command_rad_s;aux=w.auxiliary_angle_rad;k=drive.curvature(main,aux);oldk=base.curvature(main,aux);lo,hi=common_support(bm,main)
  samples=sample_motion_primitive(start_pose=[w.x_m,w.y_m,w.yaw_rad],signed_arc_length_m=np.sign(main)*req.primitive_length_m,curvature_1_m=k,sample_spacing_m=req.collision_sample_spacing_m)
  good=primitive_is_collision_free(samples,occupancy_grid=req.occupancy_grid,origin_xy=req.origin_xy,resolution_m=req.resolution_m);occupied+=not good
  np.testing.assert_allclose(samples[-1],d['pose'][idx+1],atol=1e-10,rtol=0)
  speed=base.speed(main,aux);choices=[bm.query(main,speed,k)]+[bm.query(main,row['speed_m_s'],k) for row in bm.directions['forward' if main>0 else 'reverse'].values()]
  bad=not lo<=k<=hi;unsupported+=bad
  d['primitive_checks'].append(dict(index=idx,main=main,aux=aux,nominal_curvature_1_m=k,uncapped_v1_curvature_1_m=oldk,nominal_v1_speed_m_s=speed,common_domain=[lo,hi],outside_common_domain=bad,production_map_clamped_queries=sum(x['curvature_clamped'] for x in choices),nominal_speed_clamped=choices[0]['speed_clamped'],raster_sample_pass=good,sample_count=len(samples)))
 d.update(occupied_or_outside_primitives=occupied,unsupported_nominal_primitives=unsupported);return d

def current_query(plan,cur,current_tracking,protocol,bm):
 if plan.status!='success' or len(plan.waypoints)<2:return None
 t=make_endpoint_oracle(protocol);raw=serialize(plan);p=reference_from_arrays(raw['pose'],raw['direction'],raw['command'],plan.path_length_m,plan.expanded_nodes);install_local_reference(t,p)
 t._path_index=0;t._tracking_audit={};target=t._lookahead_target(current_tracking['lookahead_requested_m'])
 xy=np.asarray(cur['position_world_xy']);yaw=cur['yaw_world_rad'];delta=target-xy;c,s=math.cos(yaw),math.sin(yaw);b=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]]);k=0. if b@b<=1e-9 else float(2*b[1]/(b@b));main=plan.waypoints[0].main_command_rad_s
 # Only geometry and mapper-query descriptors, not full scheduled/action/smoother output.
 choice=bm.query(main,current_tracking['world_xy_speed_m_s'],k)
 return dict(first_direction=plan.waypoints[0].drive_direction,accepted_direction=cur['accepted_direction'],direction_change_required=plan.waypoints[0].drive_direction!=cur['accepted_direction'],target_world_xy=target.tolist(),lookahead_index=t._tracking_audit['lookahead_index'],geometric_pp_body_curvature_1_m=k,planned_main_rad_s=main,bank_map=choice,current_geometry_only=True,full_action_reconstructed=False,physical_prediction=False)

def main():
 out=H/'nominal_support_assessment.json';assert not out.exists()
 p=json.loads((H/'nominal_support_protocol.json').read_text());start=datetime.datetime.now(datetime.timezone.utc).isoformat()
 for group in ['source_sha256','input_sha256']:
  for f,d in p[group].items():assert sha(f)==d,f
 evidence=json.loads((H/'nominal_support_current175.json').read_text());cur=evidence['current'];mp=np.load(H/'endpoint_navigation_run/maps/frame_175.npz');snapshot=snapshot_problem({k:mp[k] for k in mp.files},cur);req,meta=prepare_request(snapshot);assert req is not None
 protocol=load_action_protocol(P/'configs/data_contract.yaml');tracker=make_endpoint_oracle(protocol);base=tracker.drive_model;original_source=type(base).__name__
 bm=BankMap(json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text()));candidate=make_support_drive(base,bm)
 plans=[];arms=[]
 for name,drive in [('original',base),('candidate',candidate)]:
  before=datetime.datetime.now(datetime.timezone.utc).isoformat();plan=plan_kinodynamic_path(req,drive,initial_direction=cur['accepted_direction']);end=datetime.datetime.now(datetime.timezone.utc).isoformat();plans.append(plan)
  arm=summarize(plan,req,drive,base,bm,cur['accepted_direction']);arm.update(name=name,started_utc=before,completed_utc=end,current_query=current_query(plan,cur,evidence['current_tracking'],protocol,bm));arms.append(arm)
 archived=np.load(H/'endpoint_navigation_run/local_plans/frame_175.npz');b,c=arms
 np.testing.assert_allclose(b['pose'],archived['pose'],atol=1e-10,rtol=0);np.testing.assert_equal(b['direction'],archived['direction']);np.testing.assert_equal(b['command'],archived['command']);assert b['expanded_nodes']==evidence['original_plan_metadata']['expanded_nodes']
 assert tracker.drive_model is base and type(tracker.drive_model).__name__==original_source
 valid=c['status']=='success' and len(c['pose'])>=2
 gates=dict(original_archived_reference_exact=True,candidate_success=valid,candidate_local_goal_tolerance=valid and c['local_goal_distance_m']<=req.goal_tolerance_m+1e-10,candidate_raster_geometry=valid and c['occupied_or_outside_primitives']==0,candidate_all_nominal_common_support=valid and c['unsupported_nominal_primitives']==0 and all(z['production_map_clamped_queries']==0 for z in c['primitive_checks']),candidate_no_more_direction_changes=valid and c['direction_changes']<=b['direction_changes'],candidate_path_no_more_one_primitive=valid and c['path_length_m']<=b['path_length_m']+.3+1e-10,candidate_objective_no_more_one_primitive=valid and c['objective']['nominal_objective_m']<=b['objective']['nominal_objective_m']+.3+1e-10,candidate_strict_nominal_support_improvement=valid and c['unsupported_nominal_primitives']<b['unsupported_nominal_primitives'])
 result=dict(started_utc=start,completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),current_frame=175,current_clock_s=cur['time_s'],current=cur,request_metadata=meta,request_parameters={k:getattr(req,k) for k in ['primitive_length_m','collision_sample_spacing_m','reverse_cost_multiplier','direction_change_cost_m','goal_tolerance_m','node_budget']},arms=arms,gates=gates,cpu_geometry_eligibility_pass=all(gates.values()),planner_searches=2,candidate_count=1,policy_act_calls=0,new_physical_sequences=0,model_fits=0,tracker_drive_model_unchanged=True,no_future_actual_labels=True,physical_model_admitted=False,navigation_admitted=False,whole_research_complete=False,automation='ACTIVE')
 out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['arms','request_metadata','current']},indent=2));print('ARMS',[(a['name'],a['status'],a['path_length_m'],a['direction_changes'],a.get('unsupported_nominal_primitives')) for a in arms])
if __name__=='__main__':main()
