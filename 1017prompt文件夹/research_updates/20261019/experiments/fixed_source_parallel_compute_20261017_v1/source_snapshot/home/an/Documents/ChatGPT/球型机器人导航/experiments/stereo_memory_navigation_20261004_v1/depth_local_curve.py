"""Privileged depth/current-past local geometry adapter; no full-map input."""
from pathlib import Path
import sys,math,time,copy
import numpy as np
from scipy.ndimage import distance_transform_edt
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark.kinodynamic_planning import KinodynamicPlanRequest,plan_kinodynamic_path
from benchmark.policies import EpisodeContext
from provenance_grid import ProvenanceGrid
from oracle_rgb_shadow import make_oracle

def finite(value):
 v=float(value)
 if not math.isfinite(v):raise ValueError('finite clock/state required')
 return v

def sanitize_packet(packet):
 if set(packet)!={'frame','time_s','points_world','root_xyz'}:raise ValueError('current depth points and root/clock only')
 t=finite(packet['time_s']);i=int(packet['frame']);p=np.asarray(packet['points_world'],float);root=np.asarray(packet['root_xyz'],float)
 if i!=packet['frame'] or i<0 or t<0 or p.ndim!=2 or p.shape[1]!=3 or root.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(root).all():raise ValueError('invalid packet')
 return dict(frame=i,time_s=t,points_world=p.copy(),root_xyz=root.copy())

def causal_packets(packets,time_s):
 t=finite(time_s)
 for packet in packets:
  if finite(packet['time_s'])<=t:yield sanitize_packet(packet)

def local_curve(grid,xy,yaw,worldgoal,drive_model):
 xy=np.asarray(xy,float);goal=np.asarray(worldgoal,float);yaw=finite(yaw)
 if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(goal).all():raise ValueError('finite XY goal required')
 started=time.perf_counter();route,status=grid.plan(xy,goal)
 meta=dict(status=status,certified_safe=False,unknown_fraction=1.,local_target=None,nominal_geometry_only=True)
 if route is None:return None,meta
 arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
 index=min(len(route)-1,int(np.searchsorted(arc,2.4)))
 target=route[index];meta['local_target']=target.tolist();meta['route_xy']=route.tolist()
 center=grid.cells(xy[None])[0];low=np.maximum(center-40,0);high=np.minimum(center+41,np.asarray(grid.labels.shape))
 # Clearance computed against ALL observed hazards, then cropped; map labels are never changed.
 clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
 occupied=(clearance<.6-1e-8).astype(np.uint8)[low[0]:high[0],low[1]:high[1]]
 origin=grid.origin+low*grid.resolution
 req=KinodynamicPlanRequest(occupied,grid.resolution,tuple(origin),tuple(xy),yaw,tuple(target),node_budget=3000)
 plan=plan_kinodynamic_path(req,drive_model)
 meta.update(status=plan.status,expanded_nodes=plan.expanded_nodes,local_path_length_m=plan.path_length_m,direction_changes=plan.direction_changes,unknown_fraction=float(grid.exploration_unknown_fraction),crop_origin=origin.tolist(),crop_shape=list(occupied.shape),seconds=time.perf_counter()-started)
 if plan.status!='success' or len(plan.waypoints)<2:return None,meta
 return plan,meta

def install_local_reference(tracker,plan):
 points=tuple(plan.waypoints);xy=np.array([[p.x_m,p.y_m] for p in points])
 if len(points)<2:raise ValueError('local reference needs at least2 points')
 if tracker._context is None:
  tracker.reset(EpisodeContext('local_depth_only',True,xy,tracker.protocol.protocol_id,points))
 else:
  # Replace geometry only: preserve all direction confirmation/failure/control state.
  tracker._context=EpisodeContext('local_depth_only',True,xy,tracker.protocol.protocol_id,points)
  tracker._oracle_path_xy=xy.copy();tracker._reference=points;tracker._reference_xy=xy.copy();tracker._path_index=0

class DepthLocalCurvePolicy:
 policy_name='privileged_depth_local_curve_diagnostic'
 # Runner flag denies GLOBAL Oracle context; the sensor layer is still explicitly privileged depth/pose.
 privileged=False
 uses_shared_tracker=True
 online_input_permissions=frozenset({'current_renderer_depth_points','current_known_pose','current_proprio','pointgoal','past_source_memory'})
 def __init__(self,protocol):self.protocol=protocol;self.reset(None)
 def reset(self,context):
  if context is not None and (context.privileged or context.oracle_path_xy is not None or context.kinodynamic_reference is not None or context.dynamic_reference is not None):raise PermissionError('global Oracle references forbidden')
  self.tracker=make_oracle(self.protocol);self.tracker.mapping_records=[]
  self.grid=None;self._packet=None;self.records=[];self.last_frame=-1;self.time=-np.inf;self.last_plan=-np.inf;self.plan=None;self.plan_meta={};self.local_reference_version=0
 @property
 def diagnostics(self):return self.tracker.diagnostics
 @property
 def request_diagnostics(self):return self.tracker.request_diagnostics
 @property
 def tracking_diagnostics(self):return self.tracker.tracking_diagnostics
 def set_current_packet(self,packet):
  p=sanitize_packet(packet)
  if self._packet is not None or p['frame']!=self.last_frame+1 or p['time_s']<=self.time:raise ValueError('one consecutive current packet required')
  self._packet=p
 def act(self,observation,goal):
  if self._packet is None:raise RuntimeError('current packet required')
  packet=self._packet;t=packet['time_s'];xy=np.asarray(observation.position_world_xy,float);yaw=finite(observation.yaw_world_rad);goal=np.asarray(goal,float)
  if np.linalg.norm(packet['root_xyz'][:2]-xy)>1e-5:raise ValueError('current state/packet mismatch')
  if goal.shape!=(2,) or not np.isfinite(goal).all():raise ValueError('invalid goal')
  if self.grid is None:self.grid=ProvenanceGrid(xy-20.,memory_mode='history')
  self.grid.update_points(packet['points_world'],packet['root_xyz'],t)
  c,s=math.cos(yaw),math.sin(yaw);worldgoal=xy+np.array([[c,-s],[s,c]])@goal
  created=False
  if t>=6.-1e-6 and t-self.last_plan>=.5-1e-6:
   self.plan,self.plan_meta=local_curve(self.grid,xy,yaw,worldgoal,self.tracker.drive_model);self.last_plan=t;created=True;self.local_reference_version+=1
   if self.plan is not None:install_local_reference(self.tracker,self.plan)
  if t<6.-1e-6 or self.plan is None:
   self.tracker._begin_tracking_audit(observation)
   action=self.tracker._zero_action('observation_settling' if t<6.-1e-6 else 'local_no_route')
  else:action=self.tracker.act(observation,goal)
  self._packet=None;self.last_frame=packet['frame'];self.time=t
  self.records.append(dict(frame=self.last_frame,time_s=t,local_plan_created=created,local_reference_version=self.local_reference_version,plan=copy.deepcopy(self.plan_meta),unknown_fraction=self.plan_meta.get('unknown_fraction',1.),certified_safe=False,request=action.tolist(),depth_pose_privileged=True,global_reference_consumed=False,**self.grid.diagnostics()))
  return action
