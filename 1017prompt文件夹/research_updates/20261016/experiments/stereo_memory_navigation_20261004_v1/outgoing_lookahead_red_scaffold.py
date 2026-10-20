"""Callable baseline scaffold for RED tests; no correction yet."""
from cusp_projection import make_cusp_oracle,CuspProjectionPolicy

def with_outgoing_endpoint(base_class):
    return base_class

def make_endpoint_oracle(protocol):
    original=make_cusp_oracle(protocol)
    return with_outgoing_endpoint(type(original))(protocol,lookahead_m=.5)

class EndpointLookaheadPolicy(CuspProjectionPolicy):
    policy_name='privileged_depth_outgoing_endpoint_diagnostic'
    def reset(self,context):
        super().reset(context)
        self.tracker=make_endpoint_oracle(self.protocol)
        self.tracker.mapping_records=[]
