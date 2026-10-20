"""One global-route decision prior. No changes to observed risk or local primitives."""
import numpy as np
from scipy.ndimage import distance_transform_edt,label
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
from risk_grid import astar_path

class RouteSideCommitmentGrid(StaticSparsePointRiskGrid):
 def __init__(self,origin,retention_s,*,static_scene_assumption):
  super().__init__(origin,retention_s,static_scene_assumption=static_scene_assumption)
  self.commitment=None;self.commitment_released=False;self._axis_start=None;self._goal=None;self._axis=None;self._normal=None
 def _clearance(self):return distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution
 def _try_commit(self,route,clearance):
  sample=self._axis_start+np.arange(0,np.linalg.norm(self._goal-self._axis_start),.05)[:,None]*self._axis
  cells=self.cells(sample);cells=cells[self.inside(cells)]
  components,_=label(clearance<planning_clearance(self)-1e-8,np.ones((3,3)));ids=components[tuple(cells.T)];hits=ids[ids>0]
  if not len(hits):return
  component=components==hits[0];coordinates=self.origin+np.argwhere(component)*self.resolution
  longitudinal=(coordinates-self._axis_start)@self._axis;span=[float(longitudinal.min()),float(longitudinal.max())]
  distance=(route-self._axis_start)@self._axis;sideways=(route-self._axis_start)@self._normal;j=int(np.argmin(np.abs(distance-np.mean(span))));side=int(np.sign(sideways[j]))
  if side==0:return
  birth=self.hazard_acquisition_time[component];birth=birth[np.isfinite(birth)]
  if not len(birth):return
  assert float(birth.max())<=self.time
  self.commitment=dict(side=side,longitudinal_span_m=span,axis_start=self._axis_start.tolist(),axis=self._axis.tolist(),decision_time_s=float(self.time),observed_risk_birth_min_s=float(birth.min()),observed_risk_birth_max_s=float(birth.max()),inflated_component_cells=int(component.sum()),planning_prior_not_observation=True,not_completed_obstacle_volume=True)
 def plan(self,xy,goal):
  xy=np.asarray(xy,float);goal=np.asarray(goal,float)
  if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(goal).all() or np.linalg.norm(goal-xy)==0:raise ValueError('finite distinct query and fixed goal required')
  if self._goal is not None and not np.allclose(goal,self._goal,atol=1e-5,rtol=0):raise ValueError('changed goal requires fresh episode decision state')
  if self.commitment is None or self.commitment_released:
   route,status=super().plan(xy,goal)
   if route is not None and not self.commitment_released:
    if self._goal is None:
     self._axis_start=xy.copy();self._goal=goal.copy();self._axis=(goal-xy)/np.linalg.norm(goal-xy);self._normal=np.array([-self._axis[1],self._axis[0]])
    self._try_commit(route,self._clearance())
   return route,status
  span=self.commitment['longitudinal_span_m']
  if float((xy-self._axis_start)@self._axis)>span[1]:
   self.commitment_released=True;return super().plan(xy,goal)
  start=self.cells(xy[None])[0];end=self.cells(goal[None])[0]
  if not self.inside(start[None])[0] or not self.inside(end[None])[0]:return None,'outside_fixed_grid'
  traverse=self._clearance()>=planning_clearance(self)-1e-8
  cells=np.indices(self.labels.shape).reshape(2,-1).T;world=self.origin+cells*self.resolution;long=(world-self._axis_start)@self._axis;side=(world-self._axis_start)@self._normal
  prior=((long>=span[0])&(long<=span[1])&(self.commitment['side']*side<-.5*self.resolution)).reshape(self.labels.shape)
  traverse &= ~prior
  if not traverse[tuple(start)] or not traverse[tuple(end)]:return None,'planning_commitment_blocks_endpoint'
  try:path=astar_path((~traverse).astype(np.uint8),start=tuple(start),goal=tuple(end))
  except ValueError:return None,'planning_commitment_no_route'
  self.exploration_unknown_fraction=1.
  return self.origin+path*self.resolution,'unknown_exploration'
 def diagnostics(self):
  d=super().diagnostics();d.update(route_side_commitment=self.commitment,route_side_commitment_released=self.commitment_released,route_side_constraint_is_observation=False)
  return d
