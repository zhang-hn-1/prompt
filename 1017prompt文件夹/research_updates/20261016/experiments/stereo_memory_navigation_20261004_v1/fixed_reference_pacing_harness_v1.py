"""Shared fixed-reference harness; one initial ordinary install, never rebuild/cursor jump."""
import copy
from benchmark.kinodynamic_planning import KinodynamicWaypoint,KinodynamicPlanResult
from depth_local_curve import install_local_reference
from guarded_reference import boundary_state

def with_frozen_reference(base,frozen):
 plan=KinodynamicPlanResult(frozen['status'],tuple(KinodynamicWaypoint(**v) for v in frozen['waypoints']),frozen['expanded_nodes'],frozen['path_length_m'],frozen['direction_changes'])
 class FrozenReference(base):
  def plan_boundary(self,xy,yaw,worldgoal,t):
   before=boundary_state(self.tracker);created=self.plan is None
   if created:
    self.plan=plan;self.plan_meta=copy.deepcopy(frozen['plan_meta']);self.local_reference_version=1
    install_local_reference(self.tracker,plan);self.reference_installed_time=t
   self.last_plan=t
   decision=dict(decision='fixed_initial_install' if created else 'fixed_retain',measured_time_s=t,
    state_before=before,state_after=boundary_state(self.tracker),certified_safe=False,
    shared_full_frozen_reference=True,not_visual_planning_validation=True)
   self.reference_decision=decision
   return created,decision
 return FrozenReference
