from pathlib import Path
import json,hashlib,datetime
import numpy as np
from prior_rgb_navigation import PriorRGBGrid
from root_direction_consumer import local_curve_for_direction
from outgoing_lookahead import make_endpoint_oracle
from reference_continuity_audit import reference_geometry_status
from test_depth_local_curve import protocol
H=Path(__file__).resolve().parent;O=H/'occluded_obstacle_memory_navigation_history_run';W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
inputs=[O/'policy_records.json',O/'states.json',O/'maps/frame_200.npz',O/'maps/frame_205.npz',O/'local_plans/frame_200.npz',O/'local_plans/frame_205.npz',O/'policy_inputs/frame_200.npz',O/'policy_inputs/frame_205.npz']
sources=[Path(__file__),H/'reference_continuity_audit.py',H/'root_direction_consumer.py',H/'risk_grid.py',H/'direction_root_planning.py',H/'outgoing_lookahead.py',W/'docs/superpowers/specs/2026-10-07-memory-route-side-cause-design.md']
identity=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs},source_sha256={str(p):sha(p) for p in sources},maximum_planner_calls=2,maximum_guards=4,policy_act_calls=0,newphysicalruns=0,oldgatesunchanged=True)
seal=H/'memory_route_side_cause_protocol.json';assert not seal.exists();seal.write_text(json.dumps(identity,indent=2)+'\n')
rows=json.loads((O/'policy_records.json').read_text());maps={i:np.load(O/'maps'/f'frame_{i:03d}.npz') for i in [200,205]};observations={i:np.load(O/'policy_inputs'/f'frame_{i:03d}.npz') for i in [200,205]};oldpath=np.load(O/'local_plans/frame_200.npz')['pose'][:,:2];guards={}
for pose_i in [200,205]:
 for map_i in [200,205]:
  m=maps[map_i];guards[f'pose{pose_i}_map{map_i}']=reference_geometry_status(observations[pose_i]['position_world_xy'],oldpath,m['labels'],m['origin'],float(m['resolution']))
assert guards['pose205_map205']==rows[205]['local']['reference_decision']['guard']
engine=make_endpoint_oracle(protocol());o=observations[205];xy=o['position_world_xy'];yaw=float(o['yaw_world_rad']);c,s=np.cos(yaw),np.sin(yaw);worldgoal=xy+np.array([[c,-s],[s,c]])@o['pointgoal_base_xy'];initial=rows[205]['local']['reference_decision']['state_before']['active_direction'];plans={}
for map_i in [200,205]:
 m=maps[map_i];g=PriorRGBGrid(m['origin'],m['labels'].shape,float(m['resolution']),'history');g.labels=m['labels'].copy();plan,meta=local_curve_for_direction(g,xy,yaw,worldgoal,engine.drive_model,initial_direction=initial);plans[str(map_i)]=dict(meta=meta,waypoints=[vars(x) for x in plan.waypoints] if plan else None)
 if map_i==205:
  real=rows[205]['local']['plan'];assert meta['local_target']==real['local_target'];assert meta['route_xy']==real['route_xy'];assert meta['direction_changes']==real['direction_changes'];actual=np.load(O/'local_plans/frame_205.npz');np.testing.assert_equal(np.array([[p.x_m,p.y_m,p.yaw_rad] for p in plan.waypoints]),actual['pose']);np.testing.assert_equal(np.array([1 if p.drive_direction=='forward' else -1 for p in plan.waypoints]),actual['direction'])
added=(maps[205]['labels']==1)&(maps[200]['labels']!=1);removed=(maps[200]['labels']==1)&(maps[205]['labels']!=1)
result=dict(protocol_sha256=sha(seal),guards=guards,plans=plans,current_replay_target_route_waypoints_direction_exact=True,actual_guard_exact=True,new_hazard_cells=int(added.sum()),removed_hazard_cells=int(removed.sum()),added_hazard_world_xy=(maps[205]['origin']+np.argwhere(added)*maps[205]['resolution']).tolist(),accepted_direction=initial,oldtarget=rows[200]['local']['plan']['local_target'],actual_newtarget=rows[205]['local']['plan']['local_target'],old_model_not_certified_physics=True,posthoc_notonline=True,whole_research_complete=False)
for p,v in identity['input_sha256'].items():assert sha(p)==v
(H/'memory_route_side_cause_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['guards','plans']},indent=2));print('four guards',[(k,v['retention_necessary_screen_pass'],v['reject_reasons']) for k,v in guards.items()]);print('two samepose targets',[(k,v['meta']['local_target'],v['meta']['direction_changes']) for k,v in plans.items()])
