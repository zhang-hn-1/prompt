from copy import deepcopy
import pytest
from test_forward_course_score_evidence_v4 import fixture

def test_parent_field_used_in_fork_is_independently_checked():
 from forward_course_score_evidence_v2 import verify_geometry
 row,state,h=fixture();row['forward_course_geometry']['parent_curvature']+=1.
 with pytest.raises(AssertionError):verify_geometry(row,state,h,enabled=True)
