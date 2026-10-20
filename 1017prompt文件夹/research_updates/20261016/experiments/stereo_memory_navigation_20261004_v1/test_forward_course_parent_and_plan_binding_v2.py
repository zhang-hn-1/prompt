from copy import deepcopy
import pytest
from test_forward_course_score_evidence_v4 import fixture

def test_parent_field_used_in_fork_is_independently_checked():
 from forward_course_score_evidence_v3 import verify_geometry
 row,state,h=fixture();row['forward_course_geometry']['parent_curvature']+=1.
 with pytest.raises(AssertionError):verify_geometry(row,state,h,enabled=True)


def test_same_call_plan_artifact_clock_version_and_waypoints_bound():
 from forward_course_score_evidence_v3 import verify_reference_artifact
 row,state,h=fixture();row['local_reference_version']=3
 plan=dict(frame=70,time_s=7.,reference_version=3,status='success',waypoints=deepcopy(row['current_reference_waypoints_after_act']))
 verify_reference_artifact(row,plan)
 for key,value in [('frame',69),('time_s',7.1),('reference_version',2),('waypoints',[])]:
  bad=deepcopy(plan);bad[key]=value
  with pytest.raises(AssertionError):verify_reference_artifact(row,bad)
