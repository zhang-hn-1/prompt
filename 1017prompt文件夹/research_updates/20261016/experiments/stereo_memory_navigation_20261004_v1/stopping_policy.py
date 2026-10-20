"""Single local stopping-envelope consumption after frozen history B0 act."""
from provenance_policy import InformationPolicy as FrozenPolicy
from stopping_consumer import StoppingConsumer

class InformationPolicy(FrozenPolicy):
    def reset(self,context):
        super().reset(context)
        self.stopping=StoppingConsumer()
    def act(self,observation,goal):
        proposed=super().act(observation,goal)
        record=self.records[-1];grid=self.belief
        final,info=self.stopping.apply(proposed,observation.position_world_xy,
            record['speed_xy'],grid.labels,grid.origin,grid.resolution)
        record.update(proposed_request=proposed.tolist(),request=final.tolist(),stopping_consumer=info)
        if info['stop_veto']:record['action_class']='observed_hazard_stopping_veto'
        return final
