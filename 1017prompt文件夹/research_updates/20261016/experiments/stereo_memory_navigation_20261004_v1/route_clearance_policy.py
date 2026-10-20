"""The unchanged bank-hold policy, with one independent belief.plan override."""
import numpy as np
from online_bank_hold_policy import InformationPolicy as FrozenPolicy
from route_clearance_consumer import RouteClearanceGrid

class InformationPolicy(FrozenPolicy):
    def act(self,observation,goal):
        if self.belief is None:
            self.belief=RouteClearanceGrid(np.asarray(observation.position_world_xy)-20.,memory_mode=self.memory_mode)
        action=super().act(observation,goal)
        info=dict(self.belief.clearance_record)
        info['entered_this_call']=bool(info.get('entered') and self._last_plan==self.records[-1]['time_s'])
        info['entered_this_plan']=info.pop('entered',False)
        info['geometry_measured_at_plan_time_s']=self._last_plan
        info['geometry_current_for_this_call']=bool(self._last_plan==self.records[-1]['time_s'])
        self.records[-1]['route_clearance']=info
        return action
