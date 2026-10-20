"""One immutable cost producer; old search and motion primitives are unchanged.

The two steady speeds define a nominal preference, not predicted transit time.
No input can admit an unsupported command or change curvature/supervisor/PID.
"""
from pathlib import Path
from dataclasses import replace
import hashlib
import json
import types
import minimum_prefix_transition_planning_v1 as search
import execution_domain_four_point_consumer_v2 as old

H=Path(__file__).resolve().parent
DESIGN=json.loads((H/'nominal_reverse_time_cost_design_v1.json').read_text())
FORWARD=H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json'
REVERSE=H/'reverse_straight_fitted_point_v1.json'
for p in [FORWARD,REVERSE]:
    assert hashlib.sha256(p.read_bytes()).hexdigest()==DESIGN['input_sha256'][str(p)]
V_FORWARD=float(json.loads(FORWARD.read_text())['directions']['forward']['low']['speed_m_s'])
V_REVERSE=float(json.loads(REVERSE.read_text())['measured']['actual_speed_m_s'])
SPEED_RATIO=V_FORWARD/V_REVERSE
assert V_FORWARD>V_REVERSE>0

def nominal_time_request(request):
    return replace(request,reverse_cost_multiplier=request.reverse_cost_multiplier*SPEED_RATIO)

def local_curve_nominal_time(*args,first_primitive_direction=None,enabled=True,**kwargs):
    def planner(req,drive,**kw):
        request=search.KinodynamicPlanRequest(**vars(req))
        if enabled:request=nominal_time_request(request)
        return search.plan_kinodynamic_path(request,drive,
            first_primitive_direction=first_primitive_direction,**kw)
    scope=dict(old.local_curve_for_direction_tube.__globals__,plan_kinodynamic_path=planner)
    fn=types.FunctionType(old.local_curve_for_direction_tube.__code__,scope)
    return fn(*args,**kwargs)
