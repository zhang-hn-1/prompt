"""Catch false acceptance from timeout, lost contact evidence, or no intervention."""
import importlib
import pytest

def metrics():
    return dict(arrival=True,raw_contacts_zero=True,net_goal_axis_progress_m=1.,
        cross_rms_m=.1,cross_peak_m=.2,wall_median_s=.2,wall_p90_s=.4)

def gates(a,b,changed=True,integrity=True):
    return importlib.import_module('heading_rate_metrics').navigation_gates(a,b,changed=changed,integrity=integrity)

def test_actual_arrival_and_contact_are_independent_required_gates():
    a=metrics();b=metrics();b['arrival']=False
    assert not gates(a,b)['arrival']
    b=metrics();b['raw_contacts_zero']=False
    assert not gates(a,b)['no_contact']

def test_progress_loss_or_zero_intervention_cannot_be_accepted():
    a=metrics();b=metrics();b['net_goal_axis_progress_m']=.999
    assert not gates(a,b)['net_progress']
    assert not gates(a,metrics(),changed=False)['actual_request_change']
    assert not gates(a,metrics(),integrity=False)['integrity']

def test_cross_error_and_latency_limits_are_applied_separately():
    a=metrics();b=metrics();b.update(cross_rms_m=.21,cross_peak_m=.41,wall_median_s=.31,wall_p90_s=.61)
    g=gates(a,b)
    assert g['arrival'] and not g['cross_rms'] and not g['cross_peak'] and not g['latency']

def test_all_required_gates_allow_identical_success_with_actual_intervention():
    assert all(gates(metrics(),metrics()).values())

def test_nonfinite_metric_is_integrity_failure_not_silent_pass():
    a=metrics();b=metrics();b['wall_median_s']=float('nan')
    with pytest.raises(ValueError):
        gates(a,b)


def test_assessment_corrections_preserve_singular_fallback():
    from heading_rate_metrics import feedback_corrections
    rate=dict(supported=False,world_z_rate_rad_s=.25,yaw_rate_rad_s=None)
    old,new=feedback_corrections(rate,desired=.5,sign=1.,gain=.1)
    assert old == new == -.025

def test_assessment_corrections_use_projected_rate_when_supported():
    from heading_rate_metrics import feedback_corrections
    rate=dict(supported=True,world_z_rate_rad_s=.25,yaw_rate_rad_s=.4)
    old,new=feedback_corrections(rate,desired=.5,sign=1.,gain=.1)
    assert old == -.025
    assert abs(new + .01)<1e-12
