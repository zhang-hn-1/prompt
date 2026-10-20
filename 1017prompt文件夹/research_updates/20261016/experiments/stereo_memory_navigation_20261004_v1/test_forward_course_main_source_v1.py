import numpy as np
import pytest

def test_main_source_uses_same_clock_physics_not_self_reported_history():
 from forward_course_score_evidence_v2 import verify_current_proprio
 physics=dict(simulation_time_s=np.array([0.,.1]),joint_velocity_rad_s=np.array([[.5,0.],[-.5,0.]]),linear_velocity_world_m_s=np.array([[.2,.2,0.],[.2,.2,0.]]))
 prop=np.zeros(13,np.float32);prop[:2]=[.2,.2];prop[12]=.5
 verify_current_proprio(prop,physics,0.)
 with pytest.raises(AssertionError):verify_current_proprio(prop,physics,.1)
 with pytest.raises(AssertionError):verify_current_proprio(prop,physics,.05)
