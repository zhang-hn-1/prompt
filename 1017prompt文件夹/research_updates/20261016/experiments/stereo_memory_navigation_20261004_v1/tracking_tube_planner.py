"""One empirical tracking tube for planner queries; original guard/PID retained."""
import math
"""Current accepted-direction root cost only; frozen guard/act/tracker retained."""
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from depth_local_curve import finite,install_local_reference
from guarded_reference import GuardedReferencePolicy,boundary_state
from reference_continuity_audit import reference_geometry_status
from direction_root_planning import KinodynamicPlanRequest,plan_kinodynamic_path
from benchmark.kinodynamic_planning import KinodynamicWaypoint as RuntimeWaypoint,KinodynamicPlanResult as RuntimePlanResult

from risk_grid import astar_path
from typed_hypothesis_grid import TypedHypothesisGrid

def planning_clearance(grid):
 if abs(grid.resolution-.1)>1e-9:raise ValueError("only frozen .1m grid domain")
 return .6+.2+grid.resolution*math.sqrt(2)/2

class TrackingTubeGrid(TypedHypothesisGrid):
    def plan(self,xy,goal):
        start=self.cells(np.asarray(xy)[None])[0];end=self.cells(np.asarray(goal)[None])[0]
        if not self.inside(start[None])[0] or not self.inside(end[None])[0]:return None,'outside_fixed_grid'
        clearance=distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution
        traversable=clearance>=planning_clearance(self)-1e-8
        if not traversable[tuple(start)] or not traversable[tuple(end)]:return None,'observed_obstacle_blocks_route'
        try:path=astar_path((~traversable).astype(np.uint8),start=tuple(start),goal=tuple(end))
        except ValueError:return None,'observed_obstacle_blocks_route'
        world=self.origin+path*self.resolution
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(world,axis=0),axis=1))]
        near=path[arc<=1.+1e-8]
        offsets=np.array([(i,j) for i in range(-6,7) for j in range(-6,7) if np.hypot(i,j)*self.resolution<=.6+1e-8])
        sampled=(near[:,None,:]+offsets[None,:,:]).reshape(-1,2)
        inside=self.inside(sampled)
        unknown=~inside
        unknown[inside]=self.labels[sampled[inside,0],sampled[inside,1]]<0
        self.exploration_unknown_fraction=float(unknown.mean())
        return world,'unknown_exploration' if unknown.any() else 'observed_route'

def local_curve_for_direction_tube(grid,xy,yaw,worldgoal,drive_model,*,initial_direction):
 if initial_direction not in (None,'forward','reverse'):raise ValueError('valid accepted direction required')
 xy=np.asarray(xy,float);goal=np.asarray(worldgoal,float);yaw=finite(yaw)
 if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(goal).all():raise ValueError('finite XY goal required')
 started=time.perf_counter();route,status=grid.plan(xy,goal)
 meta=dict(status=status,certified_safe=False,unknown_fraction=1.,local_target=None,nominal_geometry_only=True,initial_direction=initial_direction,planning_clearance_m=planning_clearance(grid),empirical_tracking_error_m=.2,empirical_tracking_error_certified=False)
 if route is None:return None,meta
 arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
 index=min(len(route)-1,int(np.searchsorted(arc,2.4)))
 target=route[index];meta['local_target']=target.tolist();meta['route_xy']=route.tolist()
 center=grid.cells(xy[None])[0];low=np.maximum(center-40,0);high=np.minimum(center+41,np.asarray(grid.labels.shape))
 # Clearance computed against ALL observed hazards, then cropped; map labels are never changed.
 clearance=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
 occupied=(clearance<planning_clearance(grid)-1e-8).astype(np.uint8)[low[0]:high[0],low[1]:high[1]]
 origin=grid.origin+low*grid.resolution
 req=KinodynamicPlanRequest(occupied,grid.resolution,tuple(origin),tuple(xy),yaw,tuple(target),node_budget=3000)
 copied=plan_kinodynamic_path(req,drive_model,initial_direction=initial_direction)
 plan=RuntimePlanResult(copied.status,tuple(RuntimeWaypoint(**vars(w)) for w in copied.waypoints),copied.expanded_nodes,copied.path_length_m,copied.direction_changes)
 meta.update(status=plan.status,expanded_nodes=plan.expanded_nodes,local_path_length_m=plan.path_length_m,direction_changes=plan.direction_changes,unknown_fraction=float(grid.exploration_unknown_fraction),crop_origin=origin.tolist(),crop_shape=list(occupied.shape),seconds=time.perf_counter()-started)
 if plan.status!='success' or len(plan.waypoints)<2:return None,meta
 return plan,meta


def with_tracking_tube(base_class):
 class TrackingTube(base_class):
  def plan_boundary(self,xy,yaw,worldgoal,t):
   before=boundary_state(self.tracker);guard=None
   if self.plan is not None:
    guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
   keep=guard is not None and guard['retention_necessary_screen_pass']
   initial=None
   if not keep:
    # Constructor's default latch is not measured accepted motion before first installation.
    initial=None if self.tracker._context is None else before['active_direction']
    if initial not in (None,'forward','reverse'):raise ValueError('valid accepted direction required')
    self.plan,self.plan_meta=local_curve_for_direction_tube(self.grid,xy,yaw,worldgoal,self.tracker.drive_model,initial_direction=initial)
    self.local_reference_version+=1
    if self.plan is not None:install_local_reference(self.tracker,self.plan)
    self.reference_installed_time=t if self.plan is not None else None
   self.last_plan=t
   decision=dict(decision='retain' if keep else 'rebuild',measured_time_s=t,guard=guard,initial_or_no_route=guard is None,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,direction_cost_input=initial)
   self.reference_decision=decision
   return not keep,decision
 return TrackingTube
