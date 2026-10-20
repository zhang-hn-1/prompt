from dataclasses import asdict
from test_current_request_context_v1 import emission
from current_request_context_v1 import build_request_context
from request_context_layer_check_v1 import layers_match

def test_raw_protocol_check_rejects_corrupted_raw_even_if_issued_unchanged():
 t,o,a=emission();c=asdict(build_request_context(frame=5,time_s=.5,observation=o,tracker=t,before_emission_count=0,issued_request=a))
 assert layers_match(c,t.tracking_diagnostics,t.protocol)
 c['raw_tracker_component']=(.7,c['raw_tracker_component'][1])
 assert not layers_match(c,t.tracking_diagnostics,t.protocol)
