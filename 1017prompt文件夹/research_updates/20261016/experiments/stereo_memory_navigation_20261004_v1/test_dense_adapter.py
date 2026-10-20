import importlib
import numpy as np
import torch
import pytest

def api():return importlib.import_module('dense_adapter')

def test_target_true_axial_depth_is_positive_horizontal_disparity():
    z=np.ones((6,8))*2;mask=np.zeros(z.shape,bool)
    d,v=api().training_target(z,z,mask,mask,focal=4,baseline=1,max_disp=4)
    assert d.shape==(8,6) and np.allclose(d,2)
    assert np.array_equal(v,np.rot90(np.indices(z.shape)[0]>=2))

def test_mask_and_partner_occlusion_invalid_are_not_supervised():
    z=np.ones((6,8))*2;rz=z.copy();lm=np.zeros(z.shape,bool);rm=lm.copy()
    lm[5,2]=True;rm[3,3]=True;rz[3,4]=3;z[5,5]=np.nan;z[5,6]=.3
    d,v=api().training_target(z,rz,lm,rm,focal=4,baseline=1,max_disp=4)
    n=np.rot90(v,-1)
    assert not n[5,2] and not n[5,3] and not n[5,4] and not n[5,5] and not n[5,6]
    assert n[5,1]
    assert np.all(np.isfinite(d))

def test_right_eye_target_has_same_positive_disparity_under_flip_swap():
    z=np.ones((6,8))*2;mask=np.zeros(z.shape,bool)
    d,v=api().training_target(z,z,mask,mask,focal=4,baseline=1,max_disp=4,right_eye=True)
    assert np.allclose(d,2)
    assert np.array_equal(v,np.rot90(np.indices(z.shape)[0]<=3)[:,::-1])

def test_decoder_only_and_running_buffers_stay_frozen():
    m=torch.nn.Module();m.feature=torch.nn.BatchNorm1d(2);m.update_block=torch.nn.Linear(2,1);m.spx_2_gru=torch.nn.Linear(1,1);m.spx_gru=torch.nn.Linear(1,1)
    names=api().select_trainable(m)
    assert set(names)=={n for n,p in m.named_parameters() if n.startswith(('update_block.','spx_2_gru.','spx_gru.'))}
    assert not m.training and not m.feature.training and not m.feature.weight.requires_grad
    assert m.update_block.weight.requires_grad

def test_sequence_loss_does_not_learn_invalid_labels_and_reaches_grads():
    a=torch.tensor([[[[1.,1000.]]]],requires_grad=True);b=torch.tensor([[[[3.,-1000.]]]],requires_grad=True)
    target=torch.tensor([[[[2.,0.]]]]);v=torch.tensor([[[[True,False]]]])
    loss=api().sequence_loss([a,b],target,v)
    assert loss.item()==pytest.approx(.5);loss.backward()
    assert a.grad[0,0,0,0]<0 and b.grad[0,0,0,0]>0 and a.grad[0,0,0,1]==0

def test_empty_and_nonfinite_supervision_raise():
    pred=torch.zeros(1,1,2,2,requires_grad=True);target=pred.detach().clone();v=torch.zeros_like(pred,dtype=torch.bool)
    with pytest.raises(ValueError):api().sequence_loss([pred],target,v)
    v[:]=True;target[:]=float('nan')
    with pytest.raises(ValueError):api().sequence_loss([pred],target,v)
