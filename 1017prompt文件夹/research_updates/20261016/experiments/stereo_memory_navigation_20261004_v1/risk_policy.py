"""Frozen diagnostic extension of the exact previous information consumer."""
import importlib.util
from pathlib import Path
from risk_grid import ExplorationGrid,limit_action
OLD=Path(__file__).resolve().parent.parent/'visible_surface_connectivity_20261004_v1'
spec=importlib.util.spec_from_file_location('frozen_information_policy',OLD/'information_policy.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.BeliefGrid=ExplorationGrid

class InformationPolicy(module.InformationPolicy):
    def act(self,observation,goal):
        action=super().act(observation,goal)
        fraction=self.belief.exploration_unknown_fraction
        # Fixed six-second settling/observation prefix counts in the35s budget.
        if self.records[-1]['time_s']<6.-1e-5:action[:]=0
        else:action=limit_action(action,fraction)
        self.records[-1].update(request=action.tolist(),unknown_swept_support_fraction=fraction,
            action_class='hold' if not action.any() else 'explicit_risk_exploration' if fraction>0 else 'observed_floor_tracking',
            full_3d_certified=False)
        return action
