"""Catch baseline drift, future feedback leakage and acausal action decoding."""
import importlib.util
import numpy as np
import pytest
import torch


def api():
    assert importlib.util.find_spec('training.motion_residual') is not None, 'residual predictor is not implemented'
    from training import motion_residual
    return motion_residual


def states():
    n = 50
    return dict(state_timestamp=np.arange(1,n+1)*.02,
        quat=np.tile([2**-.5,0,0,2**-.5],(n,1)),
        pos=np.column_stack((np.zeros(n),np.arange(1,n+1)*.04,np.zeros(n))),
        lin_vel=np.tile([0,2,0],(n,1)), ang_vel=np.zeros((n,3)),
        joint_pos=np.zeros((n,3)), joint_vel=np.zeros((n,3)),
        applied_action=np.zeros((n,2)), commanded_action=np.tile([.7,.1],(n,1)))


def test_prior_uses_measured_anchor_and_known_horizon_in_anchor_coordinates():
    s=states(); window=api().build_window(s,np.zeros((10,192),np.float32),0)
    np.testing.assert_allclose(window['prior'][:,0],[.2,.4,.6,.8,1.0],atol=1e-6)
    np.testing.assert_allclose(window['prior'][:,1:3],0,atol=1e-6)
    np.testing.assert_allclose(window['prior'][:,3],1,atol=1e-6)
    np.testing.assert_allclose(window['proprio'][:,:3],np.tile([2,0,0],(5,1)),atol=1e-6)


def test_future_feedback_and_future_images_cannot_change_inputs():
    m=api(); s=states(); emb=np.zeros((10,192),np.float32)
    before=m.build_window(s,emb,0)
    for key in ['pos','lin_vel','ang_vel','joint_pos','joint_vel','applied_action']:
        s[key][25:]+=10
    s['quat'][25:]=[1,0,0,0]; emb[5:]=999
    after=m.build_window(s,emb,0)
    for key in ['embeddings','proprio','prior','requests','past_requests']:
        torch.testing.assert_close(before[key],after[key],rtol=0,atol=0)
    assert not torch.equal(before['target'],after['target'])


def batch():
    w=api().build_window(states(),np.zeros((10,192),np.float32),0)
    return {k:v.unsqueeze(0) for k,v in w.items()}


def test_zero_residual_initialization_exactly_preserves_constant_velocity():
    m=api(); b=batch(); model=m.CVResidualPredictor()
    torch.testing.assert_close(model(b),b['prior'],rtol=0,atol=0)


def test_later_requests_cannot_change_earlier_predictions_after_nonzero_learning():
    m=api(); b=batch(); model=m.CVResidualPredictor()
    with torch.no_grad(): model.readout.weight.fill_(.1)
    before=model(b); changed={k:v.clone() for k,v in b.items()}
    changed['requests'][:,3:]*=-1
    after=model(changed)
    torch.testing.assert_close(before[:,:3],after[:,:3],rtol=0,atol=0)
    assert not torch.allclose(before[:,3:],after[:,3:])


def test_pose_loss_wraps_yaw_and_keeps_metric_units():
    m=api(); pred=torch.tensor([[[.1,.2,0.,-1.]]]*5).transpose(0,1)
    target=pred.clone(); target[...,0]=0; target[...,1]=0
    assert m.motion_loss(pred,target).item()==pytest.approx(.05)
    theta=np.deg2rad(179); target[...,2]=np.sin(theta);target[...,3]=np.cos(theta)
    pred[...,2]=np.sin(-theta);pred[...,3]=np.cos(-theta)
    assert m.motion_loss(pred,target).item()==pytest.approx(.05+np.deg2rad(2)**2,rel=1e-5)


def test_real_gradient_updates_residual_without_mutating_prior():
    m=api(); b=batch(); model=m.CVResidualPredictor(); prior=b['prior'].clone()
    target=prior.clone(); target[...,0]+=.02
    optimizer=torch.optim.AdamW(model.parameters(),lr=.001)
    before=m.motion_loss(model(b),target)
    before.backward(); assert model.readout.weight.grad.abs().sum()>0
    optimizer.step(); after=m.motion_loss(model(b),target)
    assert torch.isfinite(after) and after<before
    torch.testing.assert_close(prior,b['prior'],rtol=0,atol=0)


def test_invalid_history_or_nonfinite_prior_is_rejected():
    m=api(); b=batch(); model=m.CVResidualPredictor()
    b['embeddings']=b['embeddings'][:,:4]
    with pytest.raises(ValueError): model(b)
    b=batch(); b['prior'][0,0,0]=float('nan')
    with pytest.raises(ValueError): model(b)
