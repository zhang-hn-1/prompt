"""Exact information tracker/latch with an explicit online provenance consumer."""
import numpy as np
from risk_policy import module
from risk_grid import limit_action
from provenance_grid import ProvenanceGrid


class InformationPolicy(module.InformationPolicy):
    def __init__(self,protocol,mode,representation='points',memory_mode='history'):
        if mode!='visible' or representation!='points':raise ValueError('privileged depth-point diagnostic only')
        self.memory_mode=memory_mode
        super().__init__(protocol,mode,representation)
        self.policy_name='provenance_depth_'+memory_mode
    def act(self,observation,goal):
        if self.belief is None:
            self.belief=ProvenanceGrid(np.asarray(observation.position_world_xy)-20.,memory_mode=self.memory_mode)
        action=super().act(observation,goal)
        fraction=self.belief.exploration_unknown_fraction
        if self.records[-1]['time_s']<6.-1e-5:action[:]=0
        else:action=limit_action(action,fraction)
        self.records[-1].update(self.belief.diagnostics(),request=action.tolist(),unknown_swept_support_fraction=fraction,
            action_class='hold' if not action.any() else 'explicit_risk_exploration' if fraction>0 else 'observed_floor_tracking')
        return action
