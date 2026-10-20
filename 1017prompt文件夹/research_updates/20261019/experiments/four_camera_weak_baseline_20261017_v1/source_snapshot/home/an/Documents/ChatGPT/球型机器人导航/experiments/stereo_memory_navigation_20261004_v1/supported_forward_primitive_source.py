"""Only three frozen BankMap points with current necessary coverage, not dynamics."""
from pathlib import Path
import math,json,hashlib
H=Path(__file__).resolve().parent
TABLE=H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json'
_SHA=hashlib.sha256(TABLE.read_bytes()).hexdigest()
_LEVEL=json.loads(TABLE.read_text())['directions']['forward']['low']

def supported_curvature(main,aux):
 if not math.isfinite(main) or not math.isfinite(aux):raise ValueError('finite given request')
 if abs(main-.5)>1e-12:return None
 for encoded,i in [(0.,2),(-math.pi/6,0),(math.pi/6,4)]:
  if abs(aux-encoded)<=1e-12:return float(_LEVEL['curvature_body_1_m'][i])
 return None

def source_identity():return dict(source_file=str(TABLE),sha256=_SHA,steady_point_source_only=True,no_new_fit=True,no_interpolation=True,unsupported_returns_none=True,not_dynamic_prediction=True)
