"""Four covered steady points only; no interpolation or transient physics model."""
from pathlib import Path
import json,hashlib,math
from supported_forward_primitive_source import supported_curvature as forward,source_identity as forward_identity
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
POINT=H/'reverse_straight_fitted_point_v1.json';ADMISSION=H/'reverse_straight_domain_admission_v2.json'
_point=json.loads(POINT.read_text());_admit=json.loads(ADMISSION.read_text())
assert _admit['decision']=='PASS_ONE_REVERSE_STRAIGHT_POINT_WHOLE_SEQUENCE_ONLY' and _admit['point_sha256']==sha(POINT)
assert all(sha(p)==s for p,s in _point['provenance_sha256'].items())
for arm,row in _admit['checks'].items():
 assert sha(H/('reverse_straight_'+arm+'_assessment_v1.json'))==row['assessment_sha256']
 assert sha(H/('reverse_straight_'+arm+'_verification_v1.json'))==row['verification_sha256']
_K=float(_point['measured']['curvature_body_1_m']);assert math.isfinite(_K)
def supported_requests():return ((.5,0.),(.5,-math.pi/6),(.5,math.pi/6),(-.3,0.))
def supported_curvature(main,aux):
 if not math.isfinite(main) or not math.isfinite(aux):raise ValueError('finite given request')
 if abs(main+.3)<=1e-12 and abs(aux)<=1e-12:return _K
 return forward(main,aux)
def source_identity():return dict(forward=forward_identity(),reverse_point_sha256=sha(POINT),reverse_admission_sha256=sha(ADMISSION),supported_requests=supported_requests(),not_dynamic_prediction=True,no_interpolation=True,no_unsupported_geometry=True,not_navigation_admission=True)
