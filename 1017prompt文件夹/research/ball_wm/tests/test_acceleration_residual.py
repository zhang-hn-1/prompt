"""Catch incorrect integration scale, nonzero initial velocity and future leakage."""
import importlib.util
import numpy as np
import pytest
import torch
from test_motion_residual import batch
from training.motion_residual import CVResidualPredictor,motion_loss


def api():
    assert importlib.util.find_spec('training.acceleration_residual') is not None, 'acceleration model is not implemented'
    from training import acceleration_residual
    return acceleration_residual


def test_constant_acceleration_matches_half_a_t_squared():
    m=api();a=torch.tensor([[[2.,-4.,1.]]*5])
    expected=torch.tensor([[[.01,-.02,.005],[.04,-.08,.02],[.09,-.18,.045],[.16,-.32,.08],[.25,-.50,.125]]])
    torch.testing.assert_close(m.integrate_acceleration(a,.1),expected,atol=1e-7,rtol=1e-6)


def test_opposite_acceleration_stops_extra_velocity_but_keeps_displacement():
    m=api();a=torch.zeros(1,5,3);a[0,0,0]=2;a[0,1,0]=-2
    expected=torch.tensor([.01,.02,.02,.02,.02])
    torch.testing.assert_close(m.integrate_acceleration(a,.1)[0,:,0],expected,atol=1e-7,rtol=1e-6)


def test_zero_output_preserves_prior_and_network_parameters_match_old_structure():
    m=api();torch.manual_seed(3072);old=CVResidualPredictor();torch.manual_seed(3072);new=m.AccelerationResidualPredictor()
    assert old.state_dict().keys()==new.state_dict().keys()
    for k,v in old.state_dict().items():torch.testing.assert_close(v,new.state_dict()[k],rtol=0,atol=0)
    b=batch();torch.testing.assert_close(new(b),b['prior'],rtol=0,atol=0)


def test_model_integrates_acceleration_into_anchor_xy_and_composes_yaw():
    m=api();model=m.AccelerationResidualPredictor();b=batch()
    with torch.no_grad():model.readout.bias.copy_(torch.tensor([2.,-4.,1.]))
    b['prior'][...,2]=1;b['prior'][...,3]=0
    out=model(b)
    torch.testing.assert_close(out[0,:,0]-b['prior'][0,:,0],torch.tensor([.01,.04,.09,.16,.25]),atol=1e-7,rtol=1e-6)
    torch.testing.assert_close(out[0,:,1],torch.tensor([-.02,-.08,-.18,-.32,-.5]),atol=1e-7,rtol=1e-6)
    theta=torch.tensor([.005,.02,.045,.08,.125]);torch.testing.assert_close(out[0,:,2],theta.cos());torch.testing.assert_close(out[0,:,3],-theta.sin())


def test_nonzero_acceleration_predictions_obey_command_prefix_causality():
    m=api();model=m.AccelerationResidualPredictor();b=batch()
    with torch.no_grad():model.readout.weight.fill_(.1)
    original=model(b);altered={k:v.clone() for k,v in b.items()};altered['requests'][:,3:]*=-1
    new=model(altered);torch.testing.assert_close(original[:,:3],new[:,:3],rtol=0,atol=0)
    assert not torch.allclose(original[:,3:],new[:,3:])


def test_acceleration_gradients_train_without_mutating_prior():
    m=api();torch.manual_seed(11);model=m.AccelerationResidualPredictor();b=batch();prior=b['prior'].clone();target=prior.clone();target[...,0]+=.02
    opt=torch.optim.AdamW(model.parameters(),lr=.001);before=motion_loss(model(b),target);before.backward()
    assert model.readout.weight.grad.abs().sum()>0
    opt.step();assert motion_loss(model(b),target)<before
    torch.testing.assert_close(prior,b['prior'],rtol=0,atol=0)


def test_integrator_rejects_bad_time_or_nonfinite_acceleration():
    m=api();a=torch.zeros(1,5,3)
    for dt in [0,-.1,float('nan')]:
        with pytest.raises(ValueError):m.integrate_acceleration(a,dt)
    a[0,2,1]=float('nan')
    with pytest.raises(ValueError):m.integrate_acceleration(a,.1)
