"""Training-only labels and decoder selection; inference has no truth/pose input."""
import numpy as np
PREFIXES=('update_block.','spx_2_gru.','spx_gru.')

def training_target(depth,partner_depth,left_mask,right_mask,focal=584.8888888888889,baseline=.6,max_disp=416,right_eye=False):
    z=np.asarray(depth,float);rz=np.asarray(partner_depth,float);lm=np.asarray(left_mask,bool);rm=np.asarray(right_mask,bool)
    if not (z.shape==rz.shape==lm.shape==rm.shape) or z.ndim!=2:raise ValueError('label dimensions')
    y,x=np.indices(z.shape);finite=np.isfinite(z)&(z>0);d=np.zeros(z.shape,np.float32)
    d[finite]=focal*baseline/z[finite]
    partner=y+(d if right_eye else -d);iy=np.rint(np.clip(partner,0,z.shape[0]-1)).astype(int)
    pr=rz[iy,x]
    valid=finite&(z>=.5)&(z<=3)&(d>0)&(d<max_disp)&~lm&(partner>=0)&(partner<=z.shape[0]-1)&~rm[iy,x]&np.isfinite(pr)
    valid&=(np.abs(pr-z)<=.03+.01*z)
    d=np.ascontiguousarray(np.rot90(d));valid=np.ascontiguousarray(np.rot90(valid))
    if right_eye:d=np.ascontiguousarray(d[:,::-1]);valid=np.ascontiguousarray(valid[:,::-1])
    return d,valid

def select_trainable(model):
    model.eval();names=[]
    for name,p in model.named_parameters():
        keep=name.startswith(PREFIXES);p.requires_grad_(keep)
        if keep:names.append(name)
    if not names:raise ValueError('no decoder parameter found')
    return names

def sequence_loss(predictions,target,valid):
    import torch
    import torch.nn.functional as F
    if not predictions or not bool(valid.any()) or not bool(torch.isfinite(target[valid]).all()):raise ValueError('empty/nonfinite supervision')
    weights=[.9**(len(predictions)-1-i) for i in range(len(predictions))]
    total=target.new_zeros(())
    for pred,w in zip(predictions,weights):
        if pred.shape!=target.shape or not bool(torch.isfinite(pred[valid]).all()):raise ValueError('prediction shape/nonfinite')
        total=total+w*F.smooth_l1_loss(pred.float()[valid],target.float()[valid],beta=1)
    return total/sum(weights)
