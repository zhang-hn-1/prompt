"""One planning-only detour-side screen, two factual maps, no control update."""
from pathlib import Path
import json,numpy as np
from scipy.ndimage import distance_transform_edt,label
from scipy.spatial.transform import Rotation
from partial_point_risk_grid import PartialPointRiskGrid
from execution_domain_four_point_consumer_v2 import planning_clearance,local_curve_for_direction_tube
from risk_grid import astar_path
from prior_rgb_navigation import PriorRGBPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent;D=H/'original_E1_static_sparse_memory_navigation_run_v1';O=H/'E1_route_side_continuation_geometry_v1';O.mkdir(exist_ok=False);states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());goal=np.array([3.300000005960465,5.960465010446114e-9]);axis_start=np.asarray(states[60]['position'])[:2];axis=(goal-axis_start)/np.linalg.norm(goal-axis_start);normal=np.array([-axis[1],axis[0]]);first=np.load(D/'maps/frame_060.npz');g=PartialPointRiskGrid(first['origin'],10);g.labels=first['labels'];clearance=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;blocked=clearance<planning_clearance(g)-1e-8;components,n=label(blocked,np.ones((3,3)));sample=axis_start+np.arange(0,np.linalg.norm(goal-axis_start),.05)[:,None]*axis;cells=g.cells(sample);ids=components[tuple(cells.T)];hits=ids[ids>0];assert len(hits),'no observedrisk component on directray';component=components==hits[0];coords=np.argwhere(component);world=g.origin+coords*g.resolution;long=(world-axis_start)@axis;span=[float(long.min()),float(long.max())];route=np.asarray(rows[60]['plan']['route_xy']);r_long=(route-axis_start)@axis;r_lat=(route-axis_start)@normal;mid=float(np.mean(span));j=int(np.argmin(np.abs(r_long-mid)));side=int(np.sign(r_lat[j]));assert side!=0,'initial route has no side commitment';result=[]
class QueryGrid(PartialPointRiskGrid):
 def plan(self,xy,worldgoal):
  start=self.cells(np.asarray(xy)[None])[0];end=self.cells(np.asarray(worldgoal)[None])[0];dist=distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution;traverse=dist>=planning_clearance(self)-1e-8
  index=np.indices(self.labels.shape).reshape(2,-1).T;w=self.origin+index*self.resolution;s=(w-axis_start)@axis;l=(w-axis_start)@normal;constraint=((s>=span[0])&(s<=span[1])&(side*l<-.5*self.resolution)).reshape(self.labels.shape);traverse &= ~constraint
  if not traverse[tuple(start)] or not traverse[tuple(end)]:return None,'planning_commitment_blocks_endpoint'
  try:path=astar_path((~traverse).astype(np.uint8),start=tuple(start),goal=tuple(end))
  except ValueError:return None,'planning_commitment_no_route'
  self.exploration_unknown_fraction=1.;return self.origin+path*self.resolution,'unknown_exploration'
protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,'current').tracker.drive_model
for i in [60,155]:
 saved=np.load(D/'maps'/f'frame_{i:03d}.npz');xy=np.asarray(states[i]['position'])[:2];R=Rotation.from_quat(np.asarray(states[i]['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));pair=[]
 for name,Class in [('factual',PartialPointRiskGrid),('side_continuation_query',QueryGrid)]:
  grid=Class(saved['origin'],10);grid.labels=saved['labels'].copy();before=grid.labels.copy();plan,meta=local_curve_for_direction_tube(grid,xy,yaw,goal,drive,initial_direction=rows[i]['plan']['initial_direction']);assert np.array_equal(before,grid.labels)
  if name=='factual':np.testing.assert_array_equal([[w.x_m,w.y_m] for w in plan.waypoints],rows[i]['installed_reference_xy_after_act'])
  reverse=sum(np.hypot(b.x_m-a.x_m,b.y_m-a.y_m) for a,b in zip(plan.waypoints[:-1],plan.waypoints[1:]) if a.drive_direction=='reverse') if plan is not None else None
  pair.append(dict(name=name,status=meta['status'],local_target=meta['local_target'],path_length_m=None if plan is None else plan.path_length_m,direction_changes=None if plan is None else plan.direction_changes,nominal_outgoing_reverse_length_m=reverse,waypoints=None if plan is None else [vars(w) for w in plan.waypoints],not_alternate_actual_future=True))
 result.append(dict(frame=i,queries=pair))
b,c=result[1]['queries'];passed=c['status']=='success' and b['nominal_outgoing_reverse_length_m']-c['nominal_outgoing_reverse_length_m']>=.6-1e-8 and c['direction_changes']<=b['direction_changes'];a=dict(decision='PASS_ONE_NOMINAL_SIDE_CONTINUATION_SCREEN_ONLY' if passed else 'REJECT_UNIQUE_SIDE_CONTINUATION_NOMINAL_SCREEN',axis_start=axis_start.tolist(),axis=axis.tolist(),observed_inflated_component_span_m=span,chosen_side_from_first_actual_route=side,first_component_cell_count=int(component.sum()),queries=result,no_grid_label_mutation=True,planning_prior_not_observation=True,no_new_physics_NN_or_model=True,not_control_or_navigation_admission=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k!='queries'},indent=2));print([(r['frame'],[(v['name'],v['status'],v['path_length_m'],v['nominal_outgoing_reverse_length_m'],v['local_target']) for v in r['queries']]) for r in result])
