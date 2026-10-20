"""Independent stored tracker emission and official normalization comparison."""
import numpy as np
from benchmark.policies import normalize_action_request

def layers_match(context,audit,protocol):
 if not isinstance(audit,dict):return False
 raw=tuple(audit[k] for k in ['policy_raw_main_command_rad_s','policy_raw_auxiliary_target_rad'])
 normalized=tuple(audit[k] for k in ['policy_protocol_main_command_rad_s','policy_protocol_auxiliary_target_rad'])
 official=normalize_action_request(protocol,main_velocity=raw[0],auxiliary_angle=raw[1],source='context_assessment_only')
 return (np.array_equal(context['raw_tracker_component'],raw) and np.array_equal(context['protocol_tracker_component'],normalized) and np.array_equal(normalized,official.protocol_component))
