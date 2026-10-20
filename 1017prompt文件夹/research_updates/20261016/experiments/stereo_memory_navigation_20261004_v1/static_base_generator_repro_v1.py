import numpy as np
from dynamic_self_mask import RobotSelfMask
from static_base_cached_self_mask import StaticBaseCachedSelfMask
def test_nondefault_generator_matches_original():
 q=[.1,-.2,.3];old=RobotSelfMask();new=StaticBaseCachedSelfMask();a=old.project(q,parts=(p for p in ['base_link']),include_frozen=False);b=new.project(q,parts=(p for p in ['base_link']),include_frozen=False)
 for k in a:np.testing.assert_array_equal(a[k],b[k])
