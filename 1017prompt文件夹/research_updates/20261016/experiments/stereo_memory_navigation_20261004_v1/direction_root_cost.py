"""Strict current snapshot, original request construction and nominal objective."""
from pathlib import Path
import math,sys
import numpy as np
from scipy.ndimage import distance_transform_edt
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from provenance_grid import ProvenanceGrid
from direction_root_planning import KinodynamicPlanRequest
MAP_FIELDS={'labels','provenance','floor_time','hazard_time','origin','resolution','time_s'}
CURRENT_FIELDS={'frame','time_s','position_world_xy','yaw_world_rad','pointgoal_base_xy','accepted_direction'}

def _finite(v):
 value=float(v)
 if not math.isfinite(value):raise ValueError('finite metric state required')
 return value

def snapshot_problem(map_arrays,current):
 if set(map_arrays)!=MAP_FIELDS:raise ValueError('strict map fields required')
 if set(current)!=CURRENT_FIELDS:raise ValueError('strict current fields required')
 t=_finite(current['time_s']);mt=_finite(map_arrays['time_s']);frame=int(current['frame'])
 if t<0 or frame!=current['frame'] or frame<0 or abs(t-mt)>1e-6:raise ValueError('current map clock required')
 direction=current['accepted_direction']
 if direction not in ('forward','reverse'):raise ValueError('accepted direction required')
 labels=np.asarray(map_arrays['labels']);shape=labels.shape
 if labels.ndim!=2 or 0 in shape or not np.isin(labels,[-1,0,1]).all():raise ValueError('invalid observed map')
 result_map={}
 for k in ['labels','provenance','floor_time','hazard_time']:
  a=np.asarray(map_arrays[k])
  if a.shape!=shape:raise ValueError('map shapes must agree')
  if k in ('floor_time','hazard_time'):
   relevant=labels==(0 if k=='floor_time' else 1)
   if not np.isfinite(a[relevant]).all() or np.any(a[relevant]>t+1e-6):raise ValueError('future or invalid observation time')
  result_map[k]=a.copy();result_map[k].setflags(write=False)
 origin=np.asarray(map_arrays['origin'],float);resolution=_finite(map_arrays['resolution'])
 if origin.shape!=(2,) or not np.isfinite(origin).all() or resolution<=0:raise ValueError('valid metric grid required')
 result_map.update(origin=origin.copy(),resolution=resolution,time_s=mt);result_map['origin'].setflags(write=False)
 xy=np.asarray(current['position_world_xy'],float);goal=np.asarray(current['pointgoal_base_xy'],float);yaw=_finite(current['yaw_world_rad'])
 if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(goal).all():raise ValueError('finite current geometry required')
 result_current=dict(frame=frame,time_s=t,position_world_xy=xy.copy(),yaw_world_rad=yaw,pointgoal_base_xy=goal.copy(),accepted_direction=direction)
 result_current['position_world_xy'].setflags(write=False);result_current['pointgoal_base_xy'].setflags(write=False)
 return dict(map=result_map,current=result_current)

def prepare_request(snapshot):
 raw=snapshot['map'];cur=snapshot['current'];xy=cur['position_world_xy'];yaw=cur['yaw_world_rad'];c,s=math.cos(yaw),math.sin(yaw)
 goal=xy+np.array([[c,-s],[s,c]])@cur['pointgoal_base_xy']
 grid=ProvenanceGrid(raw['origin'],shape=raw['labels'].shape,resolution=raw['resolution']);grid.labels=raw['labels'].copy()
 route,status=grid.plan(xy,goal)
 meta=dict(status=status,certified_safe=False,nominal_geometry_only=True,world_goal_xy=goal.tolist(),unknown_fraction=1.)
 if route is None:return None,meta
 arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))];index=min(len(route)-1,int(np.searchsorted(arc,2.4)));target=route[index]
 center=grid.cells(xy[None])[0];low=np.maximum(center-40,0);high=np.minimum(center+41,np.asarray(grid.labels.shape))
 clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
 occupied=(clearance<.6-1e-8).astype(np.uint8)[low[0]:high[0],low[1]:high[1]];origin=grid.origin+low*grid.resolution
 request=KinodynamicPlanRequest(occupied,grid.resolution,tuple(origin),tuple(xy),yaw,tuple(target),node_budget=3000)
 meta.update(route_xy=route.tolist(),local_target=target.tolist(),unknown_fraction=float(grid.exploration_unknown_fraction),crop_origin=origin.tolist(),crop_shape=list(occupied.shape))
 return request,meta

def objective_summary(plan,request,initial_direction):
 if initial_direction not in (None,'forward','reverse'):raise ValueError('initial_direction required')
 if plan.status!='success':return dict(status=plan.status,nominal_objective_m=None,physical_prediction=False)
 directions=['forward' if p.main_command_rad_s>0 else 'reverse' for p in plan.waypoints[:-1]]
 if not directions or any(p.main_command_rad_s==0 for p in plan.waypoints[:-1]):raise ValueError('nonzero primitives required')
 changes=sum(a!=b for a,b in zip(directions,directions[1:]));initial=int(initial_direction is not None and directions[0]!=initial_direction)
 base=request.primitive_length_m*(directions.count('forward')+request.reverse_cost_multiplier*directions.count('reverse'))
 return dict(status=plan.status,nominal_objective_m=base+request.direction_change_cost_m*(changes+initial),primitive_distance_cost_m=base,initial_direction_changes=initial,internal_direction_changes=changes,initial_switch_cost_m=initial*request.direction_change_cost_m,internal_switch_cost_m=changes*request.direction_change_cost_m,first_direction=directions[0],primitive_directions=directions,physical_prediction=False)
