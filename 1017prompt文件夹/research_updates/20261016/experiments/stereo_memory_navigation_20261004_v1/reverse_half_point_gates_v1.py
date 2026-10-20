"""Pre-data single request point criteria, no navigation/dynamic admission."""
from pathlib import Path
import json
H=Path(__file__).resolve().parent
DESIGN=json.loads((H/'reverse_half_request_design_v1.json').read_text())
OLD=json.loads((H/'reverse_straight_fitted_point_v1.json').read_text())['measured']
def point_checks(measured,*,contacts,development=None):
    g=DESIGN['gates']
    checks=dict(steady=measured['accepted'],contact_free=contacts==0,
        speed_gain=measured['actual_speed_m_s']>=g['minimum_actual_speed_m_s'],
        bank_domain=abs(measured['actual_bank_deg'])<=g['maximum_abs_bank_deg'],
        straight_geometry=abs(measured['curvature_body_1_m']-OLD['curvature_body_1_m'])<=g['maximum_curvature_difference_vs_old_1_m'])
    if development is not None:
        checks.update(speed_consistency=abs(measured['actual_speed_m_s']-development['actual_speed_m_s'])<=g['validation_speed_difference_m_s'],
            curvature_consistency=abs(measured['curvature_body_1_m']-development['curvature_body_1_m'])<=g['validation_curvature_difference_1_m'])
    return checks
