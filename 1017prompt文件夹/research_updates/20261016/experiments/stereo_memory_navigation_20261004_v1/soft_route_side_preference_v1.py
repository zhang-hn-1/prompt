"""Finite side-decision prior, not an observation or excluded unknown region.
Weighted A* retains official graph order/corner rules; no speed/model changes.
"""
from __future__ import annotations
import heapq,itertools,math
from collections.abc import Sequence
import numpy as np
from scipy.ndimage import distance_transform_edt
from benchmark.grid_planning import _as_grid,_cell,_require_inside,_NEIGHBORS,GridCell
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
from execution_domain_four_point_consumer_v2 import planning_clearance

def weighted_astar(
    grid: np.ndarray,
    weights: np.ndarray,
    *,
    start: Sequence[int],
    goal: Sequence[int],
    occupied: int | bool = 1,
) -> np.ndarray:
    """Return a deterministic shortest 8-neighbor path without corner cutting."""

    array = _as_grid(grid)
    weights = np.asarray(weights, float)
    if weights.shape != array.shape or not np.isfinite(weights).all() or (weights < 1.).any():
        raise ValueError("finite cost weights >=1 with map shape required")
    start_cell = _cell(start, "start")
    goal_cell = _cell(goal, "goal")
    _require_inside(array, start_cell, "start")
    _require_inside(array, goal_cell, "goal")
    blocked = array == occupied
    if blocked[start_cell]:
        raise ValueError("start cell is occupied")
    if blocked[goal_cell]:
        raise ValueError("goal cell is occupied")
    if start_cell == goal_cell:
        return np.asarray((start_cell,), dtype=np.int64)

    def heuristic(cell: GridCell) -> float:
        return math.hypot(cell[0] - goal_cell[0], cell[1] - goal_cell[1])

    tie = itertools.count()
    frontier: list[tuple[float, int, GridCell]] = [(heuristic(start_cell), next(tie), start_cell)]
    costs: dict[GridCell, float] = {start_cell: 0.0}
    parents: dict[GridCell, GridCell] = {}
    closed: set[GridCell] = set()

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal_cell:
            path = [current]
            while path[-1] != start_cell:
                path.append(parents[path[-1]])
            path.reverse()
            return np.asarray(path, dtype=np.int64)
        closed.add(current)

        row, column = current
        for row_step, column_step, step_cost in _NEIGHBORS:
            neighbor = row + row_step, column + column_step
            neighbor_row, neighbor_column = neighbor
            if (
                neighbor_row < 0
                or neighbor_row >= array.shape[0]
                or neighbor_column < 0
                or neighbor_column >= array.shape[1]
                or blocked[neighbor]
            ):
                continue
            if row_step and column_step:
                if blocked[row + row_step, column] or blocked[row, column + column_step]:
                    continue

            candidate_cost = costs[current] + step_cost * (weights[current]+weights[neighbor])/2.
            if candidate_cost + 1e-12 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = candidate_cost
            parents[neighbor] = current
            priority = candidate_cost + heuristic(neighbor)
            heapq.heappush(frontier, (priority, next(tie), neighbor))

    raise ValueError("goal is unreachable on the occupancy grid")


def preference_weights(shape,origin,resolution,commitment,*,current_time):
 c=commitment
 clocks=np.asarray([c['decision_time_s'],c['observed_risk_birth_max_s'],current_time],float)
 if not np.isfinite(clocks).all():raise ValueError('finite current/past prior clocks required')
 if c['decision_time_s']>current_time+1e-7 or c['observed_risk_birth_max_s']>current_time+1e-7 or not c['planning_prior_not_observation']:
  raise ValueError('current/past traced planning prior only')
 side=c['side'];span=np.asarray(c['longitudinal_span_m'],float);axis=np.asarray(c['axis'],float);start=np.asarray(c['axis_start'],float);origin=np.asarray(origin,float)
 if side not in (-1,1) or span.shape!=(2,) or axis.shape!=(2,) or start.shape!=(2,) or origin.shape!=(2,) or not np.isfinite(np.r_[span,axis,start,origin,resolution,current_time]).all() or resolution<=0 or span[0]>span[1] or abs(np.linalg.norm(axis)-1.)>1e-6:
  raise ValueError('finite normalized prior geometry required')
 cells=np.indices(shape).reshape(2,-1).T;world=origin+cells*resolution;long=(world-start)@axis;normal=np.array([-axis[1],axis[0]]);lateral=(world-start)@normal
 opposite=(long>=span[0])&(long<=span[1])&(side*lateral<-.5*resolution)
 weights=np.ones(len(cells));weights[opposite]=1.25
 return weights.reshape(shape)

class SoftRouteSidePreferenceGrid(NonbindingSideHypothesisGrid):
 def plan(self,xy,goal):
  xy=np.asarray(xy,float);goal=np.asarray(goal,float)
  if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(np.r_[xy,goal]).all() or np.linalg.norm(goal-xy)==0:raise ValueError('finite distinct task positions')
  if self._goal is not None and not np.allclose(goal,self._goal,atol=1e-5,rtol=0):raise ValueError('changed task goal requires reset')
  if self.commitment is None or self.commitment_released:return super().plan(xy,goal)
  weights=preference_weights(self.labels.shape,self.origin,self.resolution,self.commitment,current_time=self.time)
  if float((xy-self._axis_start)@self._axis)>self.commitment['longitudinal_span_m'][1]:return super().plan(xy,goal)
  start=self.cells(xy[None])[0];end=self.cells(goal[None])[0]
  if not self.inside(start[None])[0] or not self.inside(end[None])[0]:return None,'outside_fixed_grid'
  clearance=distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution
  traversable=clearance>=planning_clearance(self)-1e-8
  if not traversable[tuple(start)] or not traversable[tuple(end)]:return None,'observed_obstacle_blocks_route'
  try:path=weighted_astar((~traversable).astype(np.uint8),weights,start=tuple(start),goal=tuple(end))
  except ValueError:return None,'observed_obstacle_blocks_route'
  self.exploration_unknown_fraction=1.
  return self.origin+path*self.resolution,'unknown_exploration'
 def diagnostics(self):
  d=super().diagnostics();d.update(route_side_finite_cost_prior=True,route_side_hard_exclusion=False,route_side_cost_multiplier=1.25,route_side_prior_not_observation=True)
  return d
