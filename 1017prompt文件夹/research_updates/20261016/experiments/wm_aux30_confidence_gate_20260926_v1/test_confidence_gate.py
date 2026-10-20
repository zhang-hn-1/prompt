"""Behavior tests: causality, exact fallback, angle wrapping and useful learning."""
import importlib.util
import numpy as np
import pytest
import torch

def api():
    assert importlib.util.find_spec('confidence_gate') is not None, 'confidence gate not implemented'
    import confidence_gate
    return confidence_gate

def poses():
    prior=torch.zeros(2,5,4);prior[...,3]=1
    pred=prior.clone();pred[...,0]=2
    target=prior.clone();target[0,:,0]=1;target[1,:,0]=-1
    return prior,pred,target

def test_training_scale_and_energy_are_hand_derived():
    prior,pred,target=poses();label,weight=api().scale_targets(pred,prior,target)
    torch.testing.assert_close(label,torch.tensor([[.5]*5,[0.]*5]))
    torch.testing.assert_close(weight,torch.full((2,5),4.))
    label,weight=api().scale_targets(prior,prior,target)
    assert torch.equal(label,torch.zeros_like(label)) and torch.equal(weight,torch.zeros_like(weight))

def test_gate_features_ignore_labels_future_commands_and_visual_features():
    p=torch.arange(170,dtype=torch.float32).reshape(2,5,17)
    batch={'proprio':p,'target':torch.ones(2,5,4),'requests':torch.ones(2,5,10),'embeddings':torch.ones(2,5,192)}
    before=api().state_features(batch)
    for k in ['target','requests','embeddings']:batch[k]*=999
    torch.testing.assert_close(before,api().state_features(batch),rtol=0,atol=0)
    torch.testing.assert_close(before[0,:17],torch.arange(68,85,dtype=torch.float32))
    torch.testing.assert_close(before[0,17:],torch.full((17,),68.))

def test_blend_has_exact_zero_and_one_endpoints_without_input_mutation():
    prior,pred,_=poses();copy=prior.clone()
    out=api().blend_pose(prior,pred,torch.tensor([[0.]*5,[1.]*5]))
    torch.testing.assert_close(out[0],prior[0],rtol=0,atol=0)
    torch.testing.assert_close(out[1],pred[1],rtol=0,atol=0)
    torch.testing.assert_close(prior,copy,rtol=0,atol=0)

def test_blend_uses_short_wrapped_yaw_arc():
    prior,pred,_=poses();prior[...,2]=np.sin(np.deg2rad(179));prior[...,3]=np.cos(np.deg2rad(179))
    pred[...,2]=np.sin(np.deg2rad(-179));pred[...,3]=np.cos(np.deg2rad(-179))
    out=api().blend_pose(prior,pred,torch.full((2,5),.5))
    torch.testing.assert_close(out[...,0],torch.ones(2,5))
    torch.testing.assert_close(out[...,2],torch.zeros(2,5),atol=1e-6,rtol=0)
    torch.testing.assert_close(out[...,3],-torch.ones(2,5),atol=1e-6,rtol=0)

def test_low_coefficient_falls_back_and_nonfinite_input_is_rejected():
    cls=api().StateConfidenceGate;gate=cls(torch.zeros(34),torch.ones(34))
    with torch.no_grad():gate.linear.weight.zero_();gate.linear.bias.fill_(-2)
    batch={'proprio':torch.zeros(2,5,17)}
    assert torch.equal(gate(batch),torch.zeros(2,5))
    with torch.no_grad():gate.linear.bias.fill_(2)
    assert torch.all(gate(batch)>.5)
    batch['proprio'][0,0,0]=float('nan')
    with pytest.raises(ValueError):gate(batch)

def test_real_fit_can_distinguish_good_and_bad_current_states():
    x=torch.zeros(12,34);x[:6,0]=-1;x[6:,0]=1
    label=torch.zeros(12,5);label[6:]=1;energy=torch.ones_like(label)
    gate,history=api().fit_gate(x,label,energy,steps=200)
    predicted=gate.coefficients(x,apply_cutoff=False)
    assert predicted[:6].mean()<.25 and predicted[6:].mean()>.75
    assert history[-1]['loss']<history[0]['loss']

def test_future_request_suffix_cannot_change_combined_prefix_prediction():
    from training.acceleration_residual import AccelerationResidualPredictor
    base=AccelerationResidualPredictor()
    with torch.no_grad():base.readout.weight.fill_(.1)
    gate=api().StateConfidenceGate(torch.zeros(34),torch.ones(34))
    with torch.no_grad():gate.linear.bias.fill_(2)
    model=api().GatedPredictor(base,gate)
    batch={'embeddings':torch.ones(1,5,192),'proprio':torch.ones(1,5,17),'past_requests':torch.ones(1,5,10),'requests':torch.ones(1,5,10),'prior':torch.tensor([[[0.,0.,0.,1.]]]*5).transpose(0,1)}
    before=model(batch);batch['requests'][:,3:]*=-1;after=model(batch)
    torch.testing.assert_close(before[:,:3],after[:,:3],rtol=0,atol=0)
    assert not torch.equal(before[:,3:],after[:,3:])
