"""Same runtime class chain in both arms; one explicit qualification selector."""
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from continuous_cell_tube_consumer_v1 import with_continuous_cell_tube_after_measured_reversal
from current_forward_intent_qualification_v1 import with_current_forward_intent
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from command_domain_cap import with_command_domain_cap
from rolling_endpoint_qualification_v1 import with_rolling_endpoint

def policy_class(arm):
 if arm not in ('b0','candidate'):raise ValueError('one frozen variant')
 result=with_rolling_endpoint(with_command_domain_cap(with_cusp_coast_acquisition(with_current_forward_intent(with_continuous_cell_tube_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)),candidate=arm=='candidate'))))
 assert sum(c.__name__=='FirstPrimitiveDirection' for c in result.__mro__)==1
 assert sum(c.__name__=='CurrentForwardIntent' for c in result.__mro__)==1
 return result
