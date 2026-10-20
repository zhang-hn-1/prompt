"""Only replace the bank source after the unchanged stopping policy acts once."""
from stopping_policy import InformationPolicy as FrozenPolicy
from online_bank_hold import OnlineBankHold

def common_policy_state(policy):
    return dict(path_xy=None if policy._path_xy is None else policy._path_xy.tolist(),last_plan=policy._last_plan,reason=policy._reason,latch=dict(policy.latch.__dict__),stopping_low_count=policy.stopping.low_count,packet_consumed=policy._packet is None)

class InformationPolicy(FrozenPolicy):
    def reset(self,context):
        super().reset(context)
        self.bank_stop=OnlineBankHold()
        self.common_states=[]

    def act(self,observation,goal):
        parent_request=super().act(observation,goal)
        record=self.records[-1]
        self.common_states.append(common_policy_state(self))
        before=self.bank_stop.snapshot()
        final,info=self.bank_stop.apply(parent_request)
        record.update(baseline_final_request=parent_request.tolist(),request=final.tolist(),bank_stop_hold=info,bank_stop_state_before=before)
        return final
