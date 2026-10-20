"""Consume current bridge screen only; unchanged planner/tracker/action transaction."""
from reference_continuity_audit import reference_geometry_status
from guarded_reference import boundary_state
from raster_consistent_bridge import bridge_screen

def with_bridge_consumer(base):
 class RasterBridgeConsumer(base):
  def plan_boundary(self,xy,yaw,worldgoal,t):
   if self.plan is not None:
    guard=reference_geometry_status(xy,self.tracker._reference_xy,self.grid.labels,self.grid.origin,self.grid.resolution)
    screen=bridge_screen(dict(time_s=t,guard=guard),t,self.grid.resolution)
    if screen['alias_only_current_bridge']:
     before=boundary_state(self.tracker);self.last_plan=t
     decision=dict(decision='retain_raster_consistent_current_bridge',measured_time_s=t,guard=guard,bridge_screen=screen,initial_or_no_route=False,state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,direction_cost_input=None)
     self.reference_decision=decision
     return False,decision
   return super().plan_boundary(xy,yaw,worldgoal,t)
 return RasterBridgeConsumer
