"""Consume a capture clock without invoking policy/planner/tracker or physics."""
import numpy as np
from guarded_reference import boundary_state

def observe_clock_without_act(policy,packet,now):
 t=float(packet['time_s'])
 if not np.isfinite(now) or not np.isfinite(t) or abs(t-now)>1e-9:raise ValueError('current observation clock before payload')
 before=boundary_state(policy.tracker);n=len(policy.records);plan=policy.plan;last_plan=policy.last_plan
 policy.set_current_packet(packet)
 observed=policy._packet
 policy.last_frame=observed['frame'];policy.time=observed['time_s'];policy._packet=None
 assert len(policy.records)==n and policy.plan is plan and policy.last_plan==last_plan and boundary_state(policy.tracker)==before
