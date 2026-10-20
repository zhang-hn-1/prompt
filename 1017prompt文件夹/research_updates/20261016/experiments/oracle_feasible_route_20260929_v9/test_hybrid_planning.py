from types import SimpleNamespace
from hybrid_planning import choose_reference, steady_execution_time_s
from benchmark.kinodynamic_planning import KinodynamicPlanResult, KinodynamicWaypoint


def plan(length, main, status='success'):
    waypoints=(KinodynamicWaypoint(0.,0.,0.,'forward',main,0.),KinodynamicWaypoint(length,0.,0.,'forward',0.,0.)) if status=='success' else ()
    return KinodynamicPlanResult(status,waypoints,1,length,0)


def test_quick_frozen_route_keeps_proven_baseline():
    model=SimpleNamespace(speed=lambda main,aux:1.)
    old,new=plan(24.,1.),plan(20.,1.)
    selected,decision=choose_reference(old,new,model)
    assert selected is old and decision['selected_source']=='frozen_measured_time_with_reserve'


def test_slow_frozen_route_selects_time_route():
    model=SimpleNamespace(speed=lambda main,aux:1.)
    old,new=plan(26.,1.),plan(20.,1.)
    selected,decision=choose_reference(old,new,model)
    assert selected is new and decision['frozen_steady_time_s']==26.


def test_failed_time_route_falls_back_without_fabricating_success():
    model=SimpleNamespace(speed=lambda main,aux:1.)
    old,new=plan(26.,1.),plan(0.,1.,status='unreachable')
    selected,decision=choose_reference(old,new,model)
    assert selected is old and decision['selected_source']=='frozen_after_time_route_failure'
