"""One isolated current175 candidate search; archived B0 is not re-searched."""
from pathlib import Path
import sys,json,hashlib,datetime,math,ast
import numpy as np
H=Path(__file__).resolve().parent;W=H.parents[1];P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from direction_root_cost import snapshot_problem,prepare_request
from direction_root_planning import KinodynamicPlanResult,KinodynamicWaypoint
from time_cost_planning import plan_kinodynamic_path
from audit_nominal_support import summarize
from outgoing_lookahead import make_endpoint_oracle
from training.action_protocol import load_action_protocol
from bank_mapping import BankMap
from benchmark.drive_model import DriveModel
from nominal_time_cost import nominal_edge_cost
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def restore(d):
 return KinodynamicPlanResult(d['status'],tuple(KinodynamicWaypoint(*p,'forward' if di>0 else 'reverse',*cmd) for p,di,cmd in zip(d['pose'],d['direction'],d['command'])),d['expanded_nodes'],d['path_length_m'],d['direction_changes'])
def proxies(plan,request,drive,initial):
 vmax=max(s.speed_m_s for s in drive.steering_samples)
 if plan.status!='success' or len(plan.waypoints)<2:return None
 weighted=0.;arc_time=0.;chord_time=0.;rows=[];prev=initial
 for i,(a,b) in enumerate(zip(plan.waypoints,plan.waypoints[1:])):
  speed=drive.speed(a.main_command_rad_s,a.auxiliary_angle_rad);base=nominal_edge_cost(request.primitive_length_m,speed,vmax);reverse=a.drive_direction=='reverse';switch=prev is not None and prev!=a.drive_direction
  weighted+=base*(request.reverse_cost_multiplier if reverse else 1.)+request.direction_change_cost_m*switch
  arc_time+=request.primitive_length_m/max(.05,speed);chord=math.hypot(b.x_m-a.x_m,b.y_m-a.y_m);chord_time+=chord/speed
  rows.append(dict(index=i,nominal_v1_speed_m_s=speed,base_normalized_cost_m=base,arc_length_m=request.primitive_length_m,chord_length_m=chord,reverse=reverse,entering_direction_change=switch));prev=a.drive_direction
 return dict(maximum_v1_speed_m_s=vmax,weighted_normalized_objective_m=weighted,weighted_arc_time_proxy_s=weighted/vmax,unweighted_arc_time_proxy_s=arc_time,archived_hybrid_chord_plus_internal_switch_proxy_s=chord_time+plan.direction_changes,archived_hybrid_internal_switch_penalty_s=plan.direction_changes,rows=rows,physical_prediction=False,dynamic_deadline_admission=False)
def main():
 assert not (H/'time_cost_assessment.json').exists()
 protocol=json.loads((H/'time_cost_protocol.json').read_text());started=now()
 for group in ['source_sha256','input_sha256']:
  for f,d in protocol[group].items():assert sha(f)==d,f
 evidence=json.loads((H/'nominal_support_current175.json').read_text());cur=evidence['current'];mp=np.load(H/'endpoint_navigation_run/maps/frame_175.npz');snapshot=snapshot_problem({k:mp[k] for k in mp.files},cur);req,meta=prepare_request(snapshot);assert req is not None
 tracker=make_endpoint_oracle(load_action_protocol(P/'configs/data_contract.yaml'));drive=tracker.drive_model
 bm=BankMap(json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text()))
 archived=json.loads((H/'nominal_support_assessment.json').read_text());b0=restore(archived['arms'][0]);oldfile=np.load(H/'endpoint_navigation_run/local_plans/frame_175.npz')
 np.testing.assert_allclose([[w.x_m,w.y_m,w.yaw_rad] for w in b0.waypoints],oldfile['pose'],atol=1e-10,rtol=0)
 np.testing.assert_equal([1 if w.drive_direction=='forward' else -1 for w in b0.waypoints],oldfile['direction']);np.testing.assert_equal([[w.main_command_rad_s,w.auxiliary_angle_rad] for w in b0.waypoints],oldfile['command'])
 search_start=now();candidate=plan_kinodynamic_path(req,drive,initial_direction=cur['accepted_direction']);search_end=now()
 arms=[]
 for name,plan in [('archived_original',b0),('time_cost_candidate',candidate)]:
  d=summarize(plan,req,drive,drive,bm,cur['accepted_direction']);d.update(name=name,time_proxies=proxies(plan,req,drive,cur['accepted_direction']));arms.append(d)
 b,c=arms;valid=c['status']=='success' and len(c['pose'])>=2
 gates=dict(original_archived_reference_exact=True,candidate_success=valid,candidate_local_goal_tolerance=valid and c['local_goal_distance_m']<=req.goal_tolerance_m+1e-10,candidate_raster_geometry=valid and c['occupied_or_outside_primitives']==0,candidate_all_nominal_common_support=valid and c['unsupported_nominal_primitives']==0 and all(z['production_map_clamped_queries']==0 for z in c['primitive_checks']),candidate_no_more_direction_changes=valid and c['direction_changes']<=b['direction_changes'],candidate_path_no_more_one_primitive=valid and c['path_length_m']<=b['path_length_m']+.3+1e-10,candidate_original_objective_no_more_one_primitive=valid and c['objective']['nominal_objective_m']<=b['objective']['nominal_objective_m']+.3+1e-10,candidate_strict_nominal_support_improvement=valid and c['unsupported_nominal_primitives']<b['unsupported_nominal_primitives'],candidate_weighted_time_proxy_gain=valid and c['time_proxies']['weighted_arc_time_proxy_s']<=b['time_proxies']['weighted_arc_time_proxy_s']-.1+1e-10)
 # Literal inspection of archived time source; never importing simulation worker.
 olddir=H.parent/'oracle_feasible_route_20260929_v9/hybrid';sources={str(olddir/n):sha(olddir/n) for n in ['time_planning.py','hybrid_planning.py','run_navigation_worker.py']};time_src=(olddir/'time_planning.py').read_text();hybrid_src=(olddir/'hybrid_planning.py').read_text()
 assert 'max_reference_curvature_1_m = 0.75' in time_src and 'EPISODE_BUDGET_S - TRANSITION_RESERVE_S' in hybrid_src
 result=dict(started_utc=started,completed_utc=now(),search_started_utc=search_start,search_completed_utc=search_end,current=cur,current_frame=175,current_clock_s=cur['time_s'],original_episode_remaining_s=35-cur['time_s'],request_metadata=meta,request_parameters={k:getattr(req,k) for k in ['primitive_length_m','collision_sample_spacing_m','reverse_cost_multiplier','direction_change_cost_m','goal_tolerance_m','node_budget']},arms=arms,gates=gates,cpu_geometry_eligibility_pass=all(gates.values()),target_candidate_hybrid_searches=1,target_original_hybrid_searches=0,shared_grid_route_constructions=1,candidate_count=1,policy_act_calls=0,new_physical_sequences=0,model_fits=0,source_audit=sources,archived_time_source_bundles_curvature_filter=True,candidate_keeps_all_42_templates=True,tracker_drive_model_unchanged=True,old_cap_not_installed=True,no_future_actual_labels=True,physical_model_admitted=False,navigation_admitted=False,whole_research_complete=False,automation='ACTIVE')
 (H/'time_cost_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k not in ['arms','current','request_metadata']},indent=2));print('ARMS',[(a['name'],a['status'],a['expanded_nodes'],a['path_length_m'],a['direction_changes'],a.get('unsupported_nominal_primitives'),a.get('time_proxies')) for a in arms])
if __name__=='__main__':main()
