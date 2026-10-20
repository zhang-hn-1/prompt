import math
import numpy as np
import roll_pid as m

def test_physical_bank_is_invariant_to_local_y_shell_rotation():
    assert hasattr(m,'bank_from_quaternion'), 'physical bank feedback missing'
    r=.3;p=1.2
    q=np.array([math.cos(r/2)*math.cos(p/2),math.sin(r/2)*math.cos(p/2),math.cos(r/2)*math.sin(p/2),math.sin(r/2)*math.sin(p/2)])
    assert abs(m.bank_from_quaternion(q)-r)<1e-12

def test_feedback_opposes_excess_bank_and_preserves_main():
    assert hasattr(m,'BankPID'), 'closed loop missing'
    p=m.BankPID()
    u=p.step([1.5,.2],.25)
    assert u[0]==1.5
    assert u[1]<.2

def test_saturation_integral_bounded_and_reset():
    assert hasattr(m,'BankPID'), 'antiwindup missing'
    p=m.BankPID()
    for _ in range(2000): u=p.step([.5,.5],-.5)
    assert abs(u[1])<=math.pi/6+1e-7
    assert abs(p.integral_output)<=math.radians(5)+1e-8
    p.reset()
    assert p.integral_output==0
    assert np.allclose(p.step([0,0],0),[0,0])

def test_downstream_block_prevents_windup_and_derivative_no_setpoint_kick():
    assert hasattr(m,'BankPID'), 'downstream antiwindup missing'
    p=m.BankPID();p.step([.5,0],0)
    p.step([.5,.2],0,downstream_blocked=True)
    assert p.integral_output==0
    assert p.rate==0

def test_invalid_state_rejected():
    assert hasattr(m,'bank_from_quaternion'), 'validation missing'
    import pytest
    with pytest.raises(ValueError):m.bank_from_quaternion([0,0,0,0])
