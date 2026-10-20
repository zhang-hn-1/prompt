"""One consumer change: partial route-side hypothesis cannot exclude unknown space.
Observed hazard cells remain binding at the existing planning clearance.
"""
import numpy as np
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid

class NonbindingSideHypothesisGrid(RouteSideCommitmentGrid):
 def plan(self,xy,goal):
  xy=np.asarray(xy,float);goal=np.asarray(goal,float)
  if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(xy).all() or not np.isfinite(goal).all() or np.linalg.norm(goal-xy)==0:
   raise ValueError('finite distinct query and fixed goal required')
  if self._goal is not None and not np.allclose(goal,self._goal,atol=1e-5,rtol=0):
   raise ValueError('changed goal requires fresh episode decision state')
  if self.commitment is None or self.commitment_released:
   return super().plan(xy,goal)
  if float((xy-self._axis_start)@self._axis)>self.commitment['longitudinal_span_m'][1]:
   self.commitment_released=True
  return StaticSparsePointRiskGrid.plan(self,xy,goal)
 def diagnostics(self):
  d=super().diagnostics();d.update(route_side_hypothesis_is_binding=False,route_side_constraint_is_observation=False,route_side_hypothesis_retained_for_audit=True)
  return d
