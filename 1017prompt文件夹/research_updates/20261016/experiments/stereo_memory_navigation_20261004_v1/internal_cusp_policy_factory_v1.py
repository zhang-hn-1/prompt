"""Both arms share the rejected first-forward parent; one final boost qualifier."""
from forward_intent_policy_factory_v1 import policy_class as parent_class
from internal_cusp_reserve_v1 import with_internal_cusp_reserve
def policy_class(arm):
 if arm not in ('b0','candidate'):raise ValueError('frozen arm')
 return with_internal_cusp_reserve(parent_class('candidate'),enabled=arm=='candidate')
