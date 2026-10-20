"""Consume each waypoint outgoing edge before stopping at its endpoint."""
import numpy as np
from cusp_projection import make_cusp_oracle,CuspProjectionPolicy

def with_outgoing_endpoint(base_class):
    class OutgoingEndpoint(base_class):
        def _lookahead_target(self,lookahead_m):
            reference,xy=self._require_reference()
            direction=reference[self._path_index].drive_direction
            target_index=self._path_index
            accumulated=0.0
            while target_index+1<len(reference):
                # Direction belongs to this OUTGOING edge, not its endpoint.
                if reference[target_index].drive_direction!=direction:
                    break
                accumulated+=float(np.linalg.norm(xy[target_index+1]-xy[target_index]))
                target_index+=1
                if accumulated>=lookahead_m:
                    break
            self._tracking_audit.update({'lookahead_index':int(target_index),
                'lookahead_accumulated_m':float(accumulated),
                'lookahead_requested_m':float(lookahead_m)})
            return xy[target_index]
    return OutgoingEndpoint

def make_endpoint_oracle(protocol):
    original=make_cusp_oracle(protocol)
    return with_outgoing_endpoint(type(original))(protocol,lookahead_m=.5)

class EndpointLookaheadPolicy(CuspProjectionPolicy):
    policy_name='privileged_depth_outgoing_endpoint_diagnostic'
    def reset(self,context):
        super().reset(context)
        self.tracker=make_endpoint_oracle(self.protocol)
        self.tracker.mapping_records=[]
