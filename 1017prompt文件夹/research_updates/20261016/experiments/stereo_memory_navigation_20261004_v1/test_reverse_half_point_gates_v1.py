from reverse_half_point_gates_v1 import point_checks

def test_faster_motion_with_wrong_bank_or_geometry_cannot_pass_point():
    m=dict(accepted=True,actual_speed_m_s=.19,actual_bank_deg=.3,curvature_body_1_m=.004)
    assert all(point_checks(m,contacts=0).values())
    assert not all(point_checks(dict(m,actual_bank_deg=10.),contacts=0).values())
    assert not all(point_checks(dict(m,curvature_body_1_m=.2),contacts=0).values())
    assert not all(point_checks(dict(m,actual_speed_m_s=.10),contacts=0).values())
    assert not all(point_checks(m,contacts=1).values())
    assert not all(point_checks(m,contacts=0,development=dict(m,actual_speed_m_s=.24)).values())
