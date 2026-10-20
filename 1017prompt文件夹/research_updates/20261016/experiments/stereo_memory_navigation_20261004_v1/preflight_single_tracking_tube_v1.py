"""One same-current-map hypothetical planning query; never a physical rollout."""
from pathlib import Path
import sys,json,datetime,hashlib,time,math
import numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark.grid_planning import astar_path
from benchmark.kinodynamic_planning import KinodynamicPlanRequest,plan_kinodynamic_path
from training.action_protocol import load_action_protocol
from oracle_rgb_shadow import make_oracle
from route_geometry_audit import hazard_clearance
D=H/'bridge_consumer_C2_run_v1';i=60;m=np.load(D/'maps'/f'frame_{i:03d}.npz');inp=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz');labels=m['labels'];origin=m['origin'];res=float(m['resolution']);xy=inp['position_world_xy'];yaw=float(inp['yaw_world_rad']);goal=np.array([2.2,0.]);assert abs(float(m['time_s'])-6.)<1e-5
field=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res;start=np.rint((xy-origin)/res).astype(int);end=np.rint((goal-origin)/res).astype(int);hazards=origin+np.argwhere(labels==1)*res;drive=make_oracle(load_action_protocol(P/'configs/data_contract.yaml')).drive_model
results={};margin=.6+.2+res*math.sqrt(2)/2
for name,clear in [('original_planning',.6),('empirical_tube',margin)]:
 before=time.perf_counter();entry=dict(raster_clearance_m=clear,start_clearance_m=float(field[tuple(start)]),goal_clearance_m=float(field[tuple(end)]),start_and_goal_pass=bool(field[tuple(start)]>=clear-1e-8 and field[tuple(end)]>=clear-1e-8),full_route_length_m=None,nominal_status='not_exercised')
 if entry['start_and_goal_pass']:
  path=astar_path((field<clear-1e-8).astype(np.uint8),start=tuple(start),goal=tuple(end));route=origin+path*res;arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))];target=route[min(len(route)-1,int(np.searchsorted(arc,2.4)))];low=np.maximum(start-40,0);high=np.minimum(start+41,np.asarray(labels.shape));occupied=(field<clear-1e-8).astype(np.uint8)[low[0]:high[0],low[1]:high[1]];req=KinodynamicPlanRequest(occupied,res,tuple(origin+low*res),tuple(xy),yaw,tuple(target),node_budget=3000);plan=plan_kinodynamic_path(req,drive);wp=np.asarray([[p.x_m,p.y_m] for p in plan.waypoints]);entry.update(full_route_length_m=float(arc[-1]),nominal_status=plan.status,nominal_waypoints=len(wp),nominal_local_reference_clearance_m=hazard_clearance(wp,hazards) if len(wp) else None,nominal_node_count=plan.expanded_nodes,full_route_xy=route.tolist(),nominal_reference_xy=wp.tolist())
 entry['wall_s']=time.perf_counter()-before;results[name]=entry
r=results['empirical_tube'];checks=dict(start_goal=r['start_and_goal_pass'],route_within_frozen7m=r['full_route_length_m'] is not None and r['full_route_length_m']<=7.,nominal_reference_success=r['nominal_status']=='success' and r.get('nominal_waypoints',0)>=2,continuous_reference_clearance_ge_point8=r.get('nominal_local_reference_clearance_m') is not None and r['nominal_local_reference_clearance_m']>=.8-1e-8)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_SINGLE_TRACKING_TUBE_PREFLIGHT_ONLY' if all(checks.values()) else 'REJECT_SINGLE_TRACKING_TUBE_PREFLIGHT',checks=checks,results=results,empirical_tube_m=.2,empirical_not_certified=True,current_frame=i,source_map_is_model_hypothesis=True,GT_and_future_feedback_read=False,no_stateful_policy_act=True,new_actual_sequences=0,old_bridge_and_pursuit_reject_preserved=True,formal35_not_triggered=True,whole_research_complete=False,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());f=H/'single_tracking_tube_preflight_v1.json';assert not f.exists();f.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='results'},indent=2));print(json.dumps({n:{k:v for k,v in e.items() if not k.endswith('_xy')} for n,e in results.items()},indent=2))
